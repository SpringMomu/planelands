"""Free-final-time successive convexification (penalised trust region, PTR) for the
TAEM glide from the current state to the approach-and-landing interface (ALI).

Discretisation: time dilation t = s * tau, uniform tau grid, first-order-hold controls,
exact RK4 flow map with central-difference Jacobians (vectorised over all intervals):
    x_{k+1} = A_k x_k + Bm_k u_k + Bp_k u_{k+1} + S_k s + z_k + nu_k
Convex subproblem (cvxpy, DPP: compiled once per node count, solved with CLARABEL,
ECOS fallback), all quantities scaled to O(1):
    min  w_tf s + w_du ||Delta u||^2 + w_bank ||bank||^2 + w_sb sum(sb)
         + w_nu ||nu||_1 + w_f |terminal slack|_1 + w_soft (qbar, nz, terrain slacks)
         + w_tr (||x - xbar||^2 + ||u - ubar||^2 + (s - sbar)^2)
    s.t. dynamics, x_0 = x(t0), alpha in [alpha_min, alpha_max(M)] (trimmable range),
         |bank| <= bank_max(M), 0 <= sb <= 98.6, control rates |Delta u| <= rate * s dtau,
         4 <= qbar <= 15 kPa (soft, linearised), n_z <= 2.5 g (soft, linearised),
         h >= terrain_max + 600 m (soft, linearised), gamma / V box limits,
         terminal ALI state with penalised slack (heading on the unwrapped branch).
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import numpy as np

try:
    import cvxpy as cp
except ImportError:  # pragma: no cover
    cp = None

from .model import G0, atm

DX = np.array([1.0e4, 1.0e4, 3.0e3, 100.0, 0.3, 1.0])
DU = np.array([10.0, 0.5, 50.0])
DS = 100.0
SB_MAX = 98.6


class TerrainField:
    """Coarse, max-filtered and smoothed terrain for clearance constraints."""

    def __init__(self, terrain, cell=800.0, radius_cells=2):
        self.ok = bool(getattr(terrain, "loaded", False))
        if not self.ok:
            return
        h = terrain.h
        s = terrain.size
        k = max(int(round(cell / (terrain.extent / s))), 1)
        n = s // k
        hm = h[: n * k, : n * k].reshape(n, k, n, k).max(axis=(1, 3))
        hm = np.maximum(hm, 0.0)
        # max filter (square) + light smoothing
        r = radius_cells
        pad = np.pad(hm, r, mode="edge")
        mx = np.full_like(hm, -1e9)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                mx = np.maximum(mx, pad[r + di: r + di + n, r + dj: r + dj + n])
        pad = np.pad(mx, 1, mode="edge")
        sm = sum(pad[1 + di: 1 + di + n, 1 + dj: 1 + dj + n] for di in (-1, 0, 1) for dj in (-1, 0, 1)) / 9.0
        self.g = np.maximum(sm, mx * 0.9)
        self.n = n
        self.cell = terrain.extent / n
        self.half = terrain.extent / 2.0

    def lin(self, E, N):
        """Height and gradient (vectorised, bilinear)."""
        E = np.asarray(E, float)
        N = np.asarray(N, float)
        if not self.ok:
            z = np.zeros_like(E)
            return z, z, z
        fx = np.clip((E + self.half) / self.cell - 0.5, 0, self.n - 1.001)
        fy = np.clip((N + self.half) / self.cell - 0.5, 0, self.n - 1.001)
        j = fx.astype(int)
        i = fy.astype(int)
        tx, ty = fx - j, fy - i
        g = self.g
        h00, h01, h10, h11 = g[i, j], g[i, j + 1], g[i + 1, j], g[i + 1, j + 1]
        h = (h00 * (1 - tx) + h01 * tx) * (1 - ty) + (h10 * (1 - tx) + h11 * tx) * ty
        dE = ((h01 - h00) * (1 - ty) + (h11 - h10) * ty) / self.cell
        dN = ((h10 - h00) * (1 - tx) + (h11 - h01) * tx) / self.cell
        return h, dE, dN


@dataclass
class ScpConfig:
    w_tf: float = 1.0
    w_du: float = 3.0
    w_bank: float = 0.05
    w_sb: float = 0.002
    w_nu: float = 1.0e3
    w_soft: float = 50.0
    w_tr: float = 1.0
    rad0: float = 1.0                # initial L-inf trust radius (scaled)
    tol_cost: float = 0.01
    rate_alpha: float = 2.0          # deg/s
    rate_bank: float = math.radians(8.0)
    rate_sb: float = 5.0             # deg/s
    bank_max_sub: float = math.radians(60.0)
    bank_max_sup: float = math.radians(70.0)
    alpha_min: float = 1.0
    q_lo: float = 4.0                # kPa
    q_hi: float = 15.0
    n_max: float = 2.5
    clearance: float = 600.0
    gamma_min: float = math.radians(-35.0)
    gamma_max: float = math.radians(12.0)
    v_min: float = 70.0
    max_iter: int = 12
    tol_step: float = 2e-3
    tol_defect: float = 5e-3
    nsub: int = 3
    wf: tuple = (300.0, 300.0, 300.0, 60.0, 60.0, 60.0)


class _Problem:
    """Compiled DPP subproblem for a fixed node count."""

    def __init__(self, N, cfg: ScpConfig):
        K = N - 1
        self.N, self.K = N, K
        X = cp.Variable((6, N))
        U = cp.Variable((3, N))
        s = cp.Variable(nonneg=True)
        nu = cp.Variable((6, K))
        sf = cp.Variable(6)
        sq = cp.Variable(N, nonneg=True)
        sql = cp.Variable(N, nonneg=True)
        sn = cp.Variable(N, nonneg=True)
        st = cp.Variable(N, nonneg=True)
        P = {}
        P["A"] = cp.Parameter((36, K))
        P["Bm"] = cp.Parameter((18, K))
        P["Bp"] = cp.Parameter((18, K))
        P["S"] = cp.Parameter((6, K))
        P["z"] = cp.Parameter((6, K))
        P["x0"] = cp.Parameter(6)
        P["u0"] = cp.Parameter(3)
        P["du0"] = cp.Parameter(3, nonneg=True)
        P["xf"] = cp.Parameter(6)
        P["wf"] = cp.Parameter(6, nonneg=True)
        P["alo"] = cp.Parameter(N)
        P["ahi"] = cp.Parameter(N)
        P["bmax"] = cp.Parameter(N, nonneg=True)
        P["rate"] = cp.Parameter((3, K), nonneg=True)
        for k in ("qc", "qh", "qV", "nc", "nh", "nV", "na", "tc", "tE", "tN"):
            P[k] = cp.Parameter(N)
        P["Xr"] = cp.Parameter((6, N))
        P["Ur"] = cp.Parameter((3, N))
        P["sr"] = cp.Parameter()
        P["rad"] = cp.Parameter(nonneg=True)     # L-inf trust radius (scaled units)
        P["smin"] = cp.Parameter(nonneg=True)
        cons = [X[:, 0] == P["x0"]]
        for r in range(6):
            e = P["z"][r] + P["S"][r] * s + nu[r]
            for c in range(6):
                e = e + cp.multiply(P["A"][6 * r + c], X[c, :-1])
            for j in range(3):
                e = e + cp.multiply(P["Bm"][3 * r + j], U[j, :-1]) + cp.multiply(P["Bp"][3 * r + j], U[j, 1:])
            cons.append(X[r, 1:] == e)
        cons += [X[:, N - 1] - P["xf"] == sf]
        cons += [U[0] >= P["alo"], U[0] <= P["ahi"], cp.abs(U[1]) <= P["bmax"],
                 U[2] >= 0.0, U[2] <= SB_MAX / DU[2]]
        cons += [cp.abs(U[:, 0] - P["u0"]) <= P["du0"]]
        for j in range(3):
            dU = U[j, 1:] - U[j, :-1]
            cons += [dU <= P["rate"][j] * s, -dU <= P["rate"][j] * s]
        cons += [X[4] >= cfg.gamma_min / DX[4], X[4] <= cfg.gamma_max / DX[4], X[3] >= cfg.v_min / DX[3]]
        qlin = P["qc"] + cp.multiply(P["qh"], X[2]) + cp.multiply(P["qV"], X[3])
        cons += [qlin <= cfg.q_hi + sq, qlin >= cfg.q_lo - sql]
        nlin = P["nc"] + cp.multiply(P["nh"], X[2]) + cp.multiply(P["nV"], X[3]) + cp.multiply(P["na"], U[0])
        cons += [nlin <= cfg.n_max + sn]
        cons += [X[2] >= P["tc"] + cp.multiply(P["tE"], X[0]) + cp.multiply(P["tN"], X[1]) - st]
        cons += [s >= P["smin"], s <= 20.0]
        cons += [cp.abs(X - P["Xr"]) <= P["rad"], cp.abs(U - P["Ur"]) <= 2.0 * P["rad"], cp.abs(s - P["sr"]) <= P["rad"]]
        dUall = U[:, 1:] - U[:, :-1]
        obj = (cfg.w_tf * s + cfg.w_du * cp.sum_squares(dUall) * (N / 40.0)
               + cfg.w_bank * cp.sum_squares(U[1]) * (40.0 / N) + cfg.w_sb * cp.sum(U[2]) * (40.0 / N)
               + cfg.w_nu * cp.sum(cp.abs(nu)) + cp.sum(cp.multiply(P["wf"], cp.abs(sf)))
               + cfg.w_soft * (cp.sum(sq) + cp.sum(sql) + cp.sum(sn) + 0.01 * cp.sum(st))
               + cfg.w_tr * (cp.sum_squares(X - P["Xr"]) + cp.sum_squares(U - P["Ur"]) + cp.square(s - P["sr"])))
        self.prob = cp.Problem(cp.Minimize(obj), cons)
        self.X, self.U, self.s, self.nu, self.sf = X, U, s, nu, sf
        self.slacks = (sq, sql, sn, st)
        self.P = P

    def solve(self):
        for solver in ("CLARABEL", "ECOS"):
            try:
                self.prob.solve(solver=solver, warm_start=True, **_CANON)
                if self.prob.status in ("optimal", "optimal_inaccurate"):
                    return self.prob.status
            except Exception:  # solver failure -> fallback
                continue
        return self.prob.status or "failed"


# Browser build (Pyodide, wasm32): cvxpy's default COO canonicalisation backend fails there
# (int64 -> int32 index cast) and the C++ one is not built; the SciPy backend gives identical problems.
_CANON = {"canon_backend": "SCIPY"} if sys.platform == "emscripten" else {}


@dataclass
class ScpResult:
    X: np.ndarray
    U: np.ndarray
    tf: float
    t: np.ndarray
    converged: bool
    iterations: int
    ms: float
    cost: float
    defect: float
    status: str
    nu_norm: float
    term_err: np.ndarray
    history: list


class ScpSolver:
    def __init__(self, model, terrain_field: TerrainField | None = None, cfg: ScpConfig | None = None):
        if cp is None:
            raise RuntimeError("cvxpy is required for the SCP planner")
        self.model = model
        self.tf_field = terrain_field
        self.cfg = cfg or ScpConfig()
        self._probs = {}

    def problem(self, N):
        if N not in self._probs:
            self._probs[N] = _Problem(N, self.cfg)
        return self._probs[N]

    # ------------------------------------------------------------------ scaling helpers
    @staticmethod
    def _xs(X, xc):
        return (X - xc) / DX

    @staticmethod
    def _xu(Xs, xc):
        return Xs * DX + xc

    def _params(self, pb, Xb, Ub, tfb, x0, u0, xf, xc, wtr):
        """Linearise along the reference (Xb, Ub, tfb) and set all parameters."""
        m = self.model
        cfg = self.cfg
        N, K = pb.N, pb.K
        dtau = 1.0 / K
        dt = np.full(K, tfb * dtau)
        F, A, Bm, Bp, Sdt = m.flow_jac(Xb[:-1], Ub[:-1], Ub[1:], dt, nsub=cfg.nsub)
        Dx, Du = DX, DU
        As = A * Dx[None, None, :] / Dx[None, :, None]
        Bms = Bm * Du[None, None, :] / Dx[None, :, None]
        Bps = Bp * Du[None, None, :] / Dx[None, :, None]
        Ss = Sdt * (dtau * DS) / Dx[None, :]
        Xbs = self._xs(Xb, xc)
        Ubs = Ub / Du
        sbs = tfb / DS
        Fs = (F - xc) / Dx
        z = (Fs - np.einsum("kij,kj->ki", As, Xbs[:-1]) - np.einsum("kij,kj->ki", Bms, Ubs[:-1])
             - np.einsum("kij,kj->ki", Bps, Ubs[1:]) - Ss * sbs)
        P = pb.P
        P["A"].value = As.reshape(K, 36).T
        P["Bm"].value = Bms.reshape(K, 18).T
        P["Bp"].value = Bps.reshape(K, 18).T
        P["S"].value = Ss.T
        P["z"].value = z.T
        P["x0"].value = self._xs(x0[None], xc)[0]
        P["u0"].value = u0 / Du
        P["du0"].value = np.array([3.0, math.radians(12.0), 8.0]) / Du
        P["xf"].value = self._xs(xf[None], xc)[0]
        P["wf"].value = np.array(cfg.wf)
        # control bounds along the reference Mach
        rho, a = atm(Xb[:, 2])
        M = Xb[:, 3] / a
        amax = m.alpha_max(M)
        # alpha bounds; always contain what is reachable from the current command at the rate
        # limit (an initial alpha above the control-margin ceiling must not make it infeasible)
        tk = np.arange(N) * tfb * dtau
        ahi = np.maximum(np.maximum(amax, cfg.alpha_min + 2.0), u0[0] - cfg.rate_alpha * tk + 0.5)
        alo = np.minimum(np.full(N, cfg.alpha_min), u0[0] + cfg.rate_alpha * tk - 0.5)
        P["alo"].value = alo / Du[0]
        P["ahi"].value = ahi / Du[0]
        bmax = np.where(M > 1.05, cfg.bank_max_sup, cfg.bank_max_sub)
        bmax[-1] = math.radians(3.0)          # wings (nearly) level at the A/L interface
        P["bmax"].value = bmax / Du[1]
        Mi = 0.5 * (M[:-1] + M[1:])
        rates = np.vstack([np.full(K, cfg.rate_alpha), np.minimum(m.bank_rate(Mi), cfg.rate_bank),
                           np.full(K, cfg.rate_sb)])
        P["rate"].value = rates * dtau * DS / Du[:, None]
        # qbar linearisation (kPa)
        h, V = Xb[:, 2], Xb[:, 3]
        q = 0.5 * rho * V * V / 1000.0
        rho2, _ = atm(h + 10.0)
        qh = 0.5 * (rho2 - rho) / 10.0 * V * V / 1000.0
        qV = rho * V / 1000.0
        P["qc"].value = q - qh * Dx[2] * Xbs[:, 2] - qV * Dx[3] * Xbs[:, 3]
        P["qh"].value = qh * Dx[2]
        P["qV"].value = qV * Dx[3]
        # normal load
        n0 = m.nz(Xb, Ub)
        e = np.zeros_like(Xb)
        e[:, 2] = 10.0
        nh = (m.nz(Xb + e, Ub) - n0) / 10.0
        e[:, 2] = 0.0
        e[:, 3] = 0.5
        nV = (m.nz(Xb + e, Ub) - n0) / 0.5
        eu = np.zeros_like(Ub)
        eu[:, 0] = 0.2
        na = (m.nz(Xb, Ub + eu) - n0) / 0.2
        P["nc"].value = n0 - nh * Dx[2] * Xbs[:, 2] - nV * Dx[3] * Xbs[:, 3] - na * Du[0] * Ubs[:, 0]
        P["nh"].value = nh * Dx[2]
        P["nV"].value = nV * Dx[3]
        P["na"].value = na * Du[0]
        # terrain clearance (h >= T + clearance): in scaled units
        if self.tf_field is not None and self.tf_field.ok:
            T, TE, TN = self.tf_field.lin(Xb[:, 0], Xb[:, 1])
        else:
            T = TE = TN = np.zeros(N)
        T = T + cfg.clearance
        # h_s*Dh + xc_h >= T + TE*(E - Eb) + TN*(N - Nb)
        P["tc"].value = (T - TE * Xb[:, 0] - TN * Xb[:, 1] + TE * xc[0] + TN * xc[1] - xc[2]) / Dx[2]
        P["tE"].value = TE * Dx[0] / Dx[2]
        P["tN"].value = TN * Dx[1] / Dx[2]
        P["Xr"].value = Xbs.T
        P["Ur"].value = Ubs.T
        P["sr"].value = sbs
        P["rad"].value = wtr
        P["smin"].value = max(0.25 * tfb, 20.0) / DS

    def defects(self, X, U, tf):
        K = X.shape[0] - 1
        dt = np.full(K, tf / K)
        Y = self.model.flow(X[:-1], U[:-1], U[1:], dt, nsub=self.cfg.nsub)
        return (Y - X[1:]) / DX

    def solve(self, x0, u0, xf, Xg, Ug, tfg, max_iter=None, time_budget=None, verbose=False) -> ScpResult:
        """x0 current state, u0 current command, xf ALI target (heading unwrapped
        consistently with the guess), (Xg, Ug, tfg) initial guess with N nodes."""
        cfg = self.cfg
        N = Xg.shape[0]
        pb = self.problem(N)
        Xb, Ub, tfb = Xg.copy(), Ug.copy(), float(tfg)
        Xb[0] = x0
        xc = np.array([0.5 * (x0[0] + xf[0]), 0.5 * (x0[1] + xf[1]), 0.0, 0.0, 0.0, 0.0])
        t0 = time.perf_counter()
        hist = []
        conv = False
        status = "init"
        wtr = cfg.rad0          # here: trust radius
        it = 0
        prev_cost = None
        cost = float("nan")
        max_iter = max_iter or cfg.max_iter
        best = None
        for it in range(1, max_iter + 1):
            self._params(pb, Xb, Ub, tfb, x0, u0, xf, xc, wtr)
            status = pb.solve()
            if status not in ("optimal", "optimal_inaccurate") or pb.X.value is None:
                wtr *= 0.5
                hist.append(dict(it=it, status=status))
                if wtr < 1e-3:
                    break
                continue
            Xn = self._xu(pb.X.value.T, xc)
            Un = pb.U.value.T * DU
            tfn = float(pb.s.value) * DS
            step = max(np.abs((Xn - Xb) / DX).max(), np.abs((Un - Ub) / DU).max(), abs(tfn - tfb) / DS)
            nu = float(np.abs(pb.nu.value).sum())
            d = self.defects(Xn, Un, tfn)
            dmax = float(np.abs(d).max())
            cost = float(pb.prob.value)
            hist.append(dict(it=it, status=status, step=step, nu=nu, defect=dmax, tf=tfn, cost=cost,
                             ms=(time.perf_counter() - t0) * 1e3))
            if verbose:
                print(f"    scp it {it}: {status} step {step:.4f} nu {nu:.4f} defect {dmax:.4f} tf {tfn:.1f} cost {cost:.3f}")
            Xb, Ub, tfb = Xn, Un, tfn
            best = (Xb.copy(), Ub.copy(), tfb)
            dc = abs(cost - prev_cost) / max(abs(cost), 1e-3) if prev_cost is not None else 1.0
            prev_cost = cost
            feasible = dmax < cfg.tol_defect and nu < 1e-3
            if feasible and (step < cfg.tol_step or dc < cfg.tol_cost):
                conv = True
                break
            # trust-radius adaptation from the linearisation (defect) quality
            if dmax > 0.05:
                wtr = max(wtr * 0.5, 0.02)
            elif dmax < 0.01:
                wtr = min(wtr * 1.5, 2.0)
            if time_budget is not None and time.perf_counter() - t0 > time_budget:
                break
        ms = (time.perf_counter() - t0) * 1e3
        if best is None:
            return ScpResult(X=Xg, U=Ug, tf=tfg, t=np.linspace(0, tfg, N), converged=False, iterations=it,
                             ms=ms, cost=float("nan"), defect=float("nan"), status=status, nu_norm=float("nan"),
                             term_err=np.full(6, np.nan), history=hist)
        Xb, Ub, tfb = best
        d = self.defects(Xb, Ub, tfb)
        term = Xb[-1] - xf
        return ScpResult(X=Xb, U=Ub, tf=tfb, t=np.linspace(0, tfb, N), converged=conv, iterations=it, ms=ms,
                         cost=cost, defect=float(np.abs(d).max()), status=status,
                         nu_norm=float(hist[-1].get("nu", np.nan)), term_err=term, history=hist)
