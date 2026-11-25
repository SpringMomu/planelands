"""LTV model-predictive tracking of the reference trajectory (OSQP).

* spatial re-indexing: the horizon starts at the reference point closest to the vehicle
  (not at the current clock time), so a glider is never forced back onto a time schedule
* deviation dynamics linearised along the reference (ZOH, RK4 flow map, central FD):
      dx_{k+1} = A_k dx_k + B_k du_k + c_k
  condensed into  dX = Phi dx_0 + Gamma dU + C
* cost (scaled): cross-track, altitude, energy height, gamma, heading, airspeed (weak along-
  track), control deviations from the reference and control rates; terminal weight x3
* constraints: alpha in [alpha_min, alpha_max(M)], |bank| <= limit (phase dependent),
  0 <= sb <= 98.6, rate limits including the step from the previous command
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
import scipy.sparse as sp

try:
    import osqp
except ImportError:  # pragma: no cover
    osqp = None
clarabel = None
if osqp is None:          # e.g. the browser build (Pyodide ships Clarabel, not OSQP)
    try:
        import clarabel
    except ImportError:   # pragma: no cover
        clarabel = None

from .model import G0, atm
from .scp import DU, DX

TWO_PI = 2.0 * math.pi


@dataclass
class MpcWeights:
    sig_cross: float = 250.0
    sig_along: float = 4000.0
    sig_h: float = 150.0
    sig_e: float = 250.0
    sig_gam: float = math.radians(3.0)
    sig_psi: float = math.radians(10.0)
    sig_V: float = 25.0
    sig_alpha: float = 4.0
    sig_bank: float = math.radians(35.0)
    sig_sb: float = 40.0
    sig_dalpha: float = 1.0
    sig_dbank: float = math.radians(6.0)
    sig_dsb: float = 6.0
    term: float = 3.0


TAEM_W = MpcWeights()
AL_W = MpcWeights(sig_cross=40.0, sig_along=6000.0, sig_h=25.0, sig_e=60.0, sig_gam=math.radians(1.5),
                  sig_psi=math.radians(4.0), sig_V=8.0, sig_alpha=3.0, sig_bank=math.radians(20.0),
                  sig_sb=30.0, sig_dbank=math.radians(5.0), term=2.0)


class Mpc:
    def __init__(self, model, K=28, dt=1.0):
        self.model = model
        self.K = K
        self.dt = dt
        nU = 3 * K
        # constraint matrix: bounds (identity) + rate differences (first row vs previous command)
        I = sp.identity(nU, format="csc")
        D = sp.lil_matrix((nU, nU))
        for k in range(K):
            for j in range(3):
                D[3 * k + j, 3 * k + j] = 1.0
                if k > 0:
                    D[3 * k + j, 3 * (k - 1) + j] = -1.0
        self.D = D.tocsc()
        self.Acon = sp.vstack([I, self.D], format="csc")
        # fixed (dense) upper-triangular sparsity pattern for P so OSQP updates stay cheap
        iu, ju = np.triu_indices(nU)
        Pp = sp.csc_matrix((np.ones(len(iu)), (iu, ju)), shape=(nU, nU))
        Pp.sort_indices()
        self._Prow = Pp.indices.copy()
        self._Pcol = np.repeat(np.arange(nU), np.diff(Pp.indptr))
        self._Pshape = Pp
        self._solver = None
        self.last = None
        self.status = "idle"
        self.ms = 0.0

    def _cost_mats(self, Xr, w: MpcWeights):
        """Per-step state cost matrices (scaled coordinates)."""
        K = self.K
        Q = np.zeros((K + 1, 6, 6))
        for k in range(K + 1):
            psi = Xr[k, 5]
            a = np.array([math.sin(psi), math.cos(psi)])
            c = np.array([math.cos(psi), -math.sin(psi)])
            q = np.zeros((6, 6))
            Pp = np.outer(c, c) / w.sig_cross ** 2 + np.outer(a, a) / w.sig_along ** 2
            q[:2, :2] = Pp
            q[2, 2] += 1.0 / w.sig_h ** 2
            q[3, 3] += 1.0 / w.sig_V ** 2
            q[4, 4] += 1.0 / w.sig_gam ** 2
            q[5, 5] += 1.0 / w.sig_psi ** 2
            ev = np.array([0.0, 0.0, 1.0, Xr[k, 3] / G0, 0.0, 0.0])
            q += np.outer(ev, ev) / w.sig_e ** 2
            if k == K:
                q *= w.term
            # to scaled coordinates: dx = DX * dxs
            Q[k] = q * np.outer(DX, DX)
        return Q

    def solve(self, x, u_prev, ref, fi, gear_now=0.0, w: MpcWeights = TAEM_W, bank_lim=math.radians(60.0),
              sb_lim=98.6, alpha_min=1.0):
        """Returns (u_cmd (3,), U plan (K,3), X pred (K+1,6), ok)."""
        t0 = time.perf_counter()
        m = self.model
        K, dt = self.K, self.dt
        tau0 = ref.time_at_index(fi)
        tq = tau0 + dt * np.arange(K + 1)
        Xr, Ur, gr = ref.at_times(tq)
        gr = np.maximum(gr, gear_now)
        # linearise along the reference (ZOH)
        F, A, B, _, _ = m.flow_jac(Xr[:-1], Ur[:-1], Ur[:-1], np.full(K, dt), nsub=2, gear=gr[:-1], foh=False)
        c = F - Xr[1:]
        c[:, 5] = (c[:, 5] + math.pi) % TWO_PI - math.pi
        # scaled
        As = A * DX[None, None, :] / DX[None, :, None]
        Bs = B * DU[None, None, :] / DX[None, :, None]
        cs = c / DX
        dx0 = x - Xr[0]
        dx0[5] = (dx0[5] + math.pi) % TWO_PI - math.pi
        dx0s = dx0 / DX
        # condensed prediction
        nx, nu = 6, 3
        Phi = np.zeros(((K + 1) * nx, nx))
        Gam = np.zeros(((K + 1) * nx, K * nu))
        Cv = np.zeros((K + 1) * nx)
        Phi[:nx] = np.eye(nx)
        for k in range(K):
            r0, r1 = k * nx, (k + 1) * nx
            Phi[r1:r1 + nx] = As[k] @ Phi[r0:r0 + nx]
            Gam[r1:r1 + nx] = As[k] @ Gam[r0:r0 + nx]
            Gam[r1:r1 + nx, k * nu:(k + 1) * nu] += Bs[k]
            Cv[r1:r1 + nx] = As[k] @ Cv[r0:r0 + nx] + cs[k]
        Q = self._cost_mats(Xr, w)
        Qbig = np.zeros(((K + 1) * nx, (K + 1) * nx))
        for k in range(K + 1):
            Qbig[k * nx:(k + 1) * nx, k * nx:(k + 1) * nx] = Q[k]
        free = Phi @ dx0s + Cv
        Ru = np.tile(DU ** 2 / np.array([w.sig_alpha, w.sig_bank, w.sig_sb]) ** 2, K)
        Rd = np.tile(DU ** 2 / np.array([w.sig_dalpha, w.sig_dbank, w.sig_dsb]) ** 2, K)
        Dd = self.D.toarray()
        GtQ = Gam.T @ Qbig
        H = GtQ @ Gam + np.diag(Ru) + Dd.T @ (Rd[:, None] * Dd)
        # rate term on the first step relative to the previous command
        upr = (u_prev - Ur[0]) / DU
        dfirst = np.zeros(K * nu)
        dfirst[:3] = upr
        g = GtQ @ free - Dd.T @ (Rd * dfirst)
        H = 0.5 * (H + H.T)
        # bounds
        rho, a = atm(Xr[:-1, 2])
        Mach = Xr[:-1, 3] / a
        amax = m.alpha_max(Mach)
        lo = np.column_stack([np.full(K, alpha_min), np.full(K, -bank_lim), np.zeros(K)])
        hi = np.column_stack([np.maximum(amax, alpha_min + 1.0), np.full(K, bank_lim), np.full(K, sb_lim)])
        # keep the box consistent with the rate limits from the previous command
        kk = np.arange(1, K + 1)
        rmax = np.column_stack([np.full(K, 2.0), np.degrees(m.bank_rate(Mach)) * math.pi / 180.0, np.full(K, 5.0)])
        hi = np.maximum(hi, u_prev[None, :] - rmax * kk[:, None] * dt + 1e-3)
        lo = np.minimum(lo, u_prev[None, :] + rmax * kk[:, None] * dt - 1e-3)
        lb = ((lo - Ur[:-1]) / DU).ravel()
        ub = ((hi - Ur[:-1]) / DU).ravel()
        rate = np.column_stack([np.full(K, 2.0 * dt), m.bank_rate(Mach) * dt, np.full(K, 5.0 * dt)]) / DU
        # D dU in [ -rate - dUr, rate - dUr ] (reference control increments), first row vs u_prev
        dUr = np.vstack([Ur[0] - u_prev, np.diff(Ur[:-1], axis=0)]) / DU
        dl = (-rate.ravel() - dUr.ravel())
        du = (rate.ravel() - dUr.ravel())
        l = np.concatenate([lb, dl])
        u = np.concatenate([ub, du])
        # infeasible box (reference outside the limits) -> relax
        bad = l > u
        l[bad], u[bad] = u[bad], l[bad]
        ok = False
        dU = np.zeros(K * nu)
        if osqp is not None:
            Pdata = H[self._Prow, self._Pcol]
            P = sp.csc_matrix((Pdata, self._Pshape.indices, self._Pshape.indptr), shape=H.shape)
            try:
                if self._solver is None:
                    self._solver = osqp.OSQP()
                    self._solver.setup(P, g, self.Acon, l, u, verbose=False, eps_abs=1e-5, eps_rel=1e-5,
                                       max_iter=4000, polish=False, warm_starting=True)
                else:
                    self._solver.update(Px=P.data, q=g, l=l, u=u)
                res = self._solver.solve()
                st = str(res.info.status)
                if res.x is not None and ("solved" in st):
                    dU = np.asarray(res.x)
                    ok = True
                self.status = st
            except Exception as e:  # pragma: no cover
                self.status = f"error {e}"
                self._solver = None
        elif clarabel is not None:
            # same QP for Clarabel: l <= A x <= u  ->  [A; -A] x + s = [u; -l], s >= 0
            try:
                if self._solver is None:
                    self._A2 = sp.vstack([self.Acon, -self.Acon], format="csc")
                    st = clarabel.DefaultSettings()
                    st.verbose = False
                    st.max_iter = 60
                    self._solver = st
                Pu = sp.triu(sp.csc_matrix(H), format="csc")
                solver = clarabel.DefaultSolver(Pu, g, self._A2, np.concatenate([u, -l]),
                                                [clarabel.NonnegativeConeT(self._A2.shape[0])], self._solver)
                res = solver.solve()
                st = str(res.status)
                if st in ("Solved", "AlmostSolved"):
                    dU = np.asarray(res.x)
                    ok = True
                self.status = st.lower()
            except Exception as e:  # pragma: no cover
                self.status = f"error {e}"
                self._solver = None
        if not ok:
            # unconstrained solution clipped (fallback)
            try:
                dU = np.linalg.solve(H + 1e-9 * np.eye(len(g)), -g)
                dU = np.clip(dU, l[:K * nu], u[:K * nu])
                ok = True
                self.status = "fallback-lsq"
            except np.linalg.LinAlgError:
                dU = np.zeros(K * nu)
        dUm = dU.reshape(K, nu) * DU
        Uplan = Ur[:-1] + dUm
        Xpred = Xr + (free + Gam @ dU).reshape(K + 1, nx) * DX
        self.ms = (time.perf_counter() - t0) * 1e3
        self.last = dict(t0=tau0, U=Uplan, X=Xpred, Xr=Xr, Ur=Ur)
        return Uplan[0].copy(), Uplan, Xpred, ok
