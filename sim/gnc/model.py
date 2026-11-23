"""Vectorised 3-DOF point-mass planning model.

State   x = [E, N, h, V, gamma, psi]   ENU [m], true airspeed [m/s], air-relative flight-path
                                       angle and heading [rad] (psi from north, clockwise)
Control u = [alpha_deg, bank_rad, sb_deg]

    E'   = V cos(g) sin(psi) + wE(h)
    N'   = V cos(g) cos(psi) + wN(h)
    h'   = V sin(g)
    V'   = -D/m - g0 sin(g)
    g'   = (L cos(bank) - m g0 cos(g)) / (m V)
    psi' =  L sin(bank) / (m V cos(g))

L, D from the TRIMMED coefficient tables (alpha x Mach x speedbrake x gear) generated from
the full component-build-up aero model (sim/trim.py), so elevon/body-flap trim drag is
included.  Everything is vectorised over arbitrary leading dimensions: an SCP
discretisation (RK4 flow map + central finite-difference Jacobians for all intervals at
once) is a handful of numpy calls.
"""
from __future__ import annotations

import math

import numpy as np

G0 = 9.80665
R_AIR = 287.05287
RHO0 = 1.225
RE = 6371000.0
R_GEOP = 6356766.0
KT = 0.514444

_HB = np.array([0.0, 11000.0, 20000.0, 32000.0, 47000.0, 51000.0, 71000.0])
_TB = np.array([288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65])
_LB = np.array([-0.0065, 0.0, 0.0010, 0.0028, 0.0, -0.0028, -0.0020])


def _pbase():
    p = [101325.0]
    for i in range(len(_HB) - 1):
        dh = _HB[i + 1] - _HB[i]
        if _LB[i] == 0.0:
            p.append(p[-1] * math.exp(-G0 * dh / (R_AIR * _TB[i])))
        else:
            p.append(p[-1] * ((_TB[i] + _LB[i] * dh) / _TB[i]) ** (-G0 / (R_AIR * _LB[i])))
    return np.array(p)


_PB = _pbase()
_LB_SAFE = np.where(_LB == 0.0, 1.0, _LB)


def atm(h):
    """Vectorised US76 (0..86 km): returns (rho, a)."""
    h = np.asarray(h, dtype=float)
    H = R_GEOP * np.clip(h, -500.0, 85000.0) / (R_GEOP + np.clip(h, -500.0, 85000.0))
    i = np.clip(np.searchsorted(_HB, H, side="right") - 1, 0, len(_HB) - 1)
    tb, lb, pb, hb = _TB[i], _LB[i], _PB[i], _HB[i]
    dh = H - hb
    T = tb + lb * dh
    iso = lb == 0.0
    p = np.where(iso, pb * np.exp(-G0 * dh / (R_AIR * tb)),
                 pb * (np.maximum(T, 1.0) / tb) ** (-G0 / (R_AIR * _LB_SAFE[i])))
    rho = p / (R_AIR * T)
    a = np.sqrt(1.4 * R_AIR * T)
    return rho, a


class PlanningModel:
    nx = 6
    nu = 3

    def __init__(self, tables, mass, S, wind_speed=0.0, wind_dir_from=0.0, ground_elev=3.0):
        self.tab = tables
        self.m = float(mass)
        self.S = float(S)
        self.ag = np.asarray(tables.alpha, float)
        self.mg = np.asarray(tables.mach, float)
        self.sg = np.asarray(tables.sb, float)
        self.gg = np.asarray(tables.gear, float)
        nA, nM, nS, nG = len(self.ag), len(self.mg), len(self.sg), len(self.gg)
        self.shape = (nA, nM, nS, nG)
        T = np.stack([tables.CL, tables.CD], axis=-1)       # (nA,nM,nS,nG,2)
        self.T = T.reshape(-1, 2)
        self.strides = np.array([nM * nS * nG, nS * nG, nG, 1])
        self.a0 = self.ag[0]
        self.da = self.ag[1] - self.ag[0]
        self.ground_elev = ground_elev
        # trimmable alpha ceiling per Mach (gear up, clean), with a 1 deg margin
        trim = tables.trimmable
        amax = []
        for j in range(nM):
            ok = trim[:, j, 0, 0]
            a = self.ag[ok]
            amax.append(a.max() if len(a) else 10.0)
        self.amax_M = np.minimum(np.array(amax) - 1.0, np.where(self.mg < 1.0, 20.0, 18.0))
        # control-margin ceiling: keep the trimmed symmetric elevon above -12 deg (>= 21 deg of
        # up-travel left for roll / yaw); around Mach 1.2-1.6 the body flap is saturated and the
        # elevons carry the trim, so the usable alpha drops to ~9-12 deg
        DE = getattr(tables, "DE", None)
        if DE is not None:
            ctrl = []
            for j in range(nM):
                de = DE[:, j, 0, 0]
                am = None
                for i in range(nA):
                    if self.ag[i] < 2.0:
                        continue
                    if np.isfinite(de[i]) and de[i] >= -12.0:
                        am = self.ag[i]
                    else:
                        break
                ctrl.append((am if am is not None else 6.0) - 0.5)
            self.amax_M = np.minimum(self.amax_M, np.maximum(np.array(ctrl), 6.0))
        # best / worst L/D per Mach (clean, trimmable) for reachability
        ld_best, a_best, ld_min = [], [], []
        for j in range(nM):
            ok = trim[:, j, 0, 0] & (self.ag > 0.5) & (self.ag <= self.amax_M[j])
            ld = np.where(ok, tables.CL[:, j, 0, 0] / np.maximum(tables.CD[:, j, 0, 0], 1e-6), -1)
            k = int(np.argmax(ld))
            ld_best.append(ld[k])
            a_best.append(self.ag[k])
            ka = int(np.clip(np.searchsorted(self.ag, self.amax_M[j]) - 1, 0, nA - 1))
            ld_min.append(tables.CL[ka, j, -1, 0] / tables.CD[ka, j, -1, 0])
        self.ld_best_M = np.array(ld_best)
        self.alpha_best_M = np.array(a_best)
        self.ld_min_M = np.array(ld_min)
        self.set_wind(wind_speed, wind_dir_from)

    # ------------------------------------------------------------------ wind
    def set_wind(self, speed, dir_from):
        self.w_speed = float(speed)
        self.w_dir = float(dir_from)
        d = math.radians(dir_from)
        self.w_unit = np.array([-math.sin(d), -math.cos(d)])     # direction the wind blows TO

    def wind(self, h):
        """Mean wind (wE, wN) at altitude h (MSL): 1/7 power law below 500 m AGL."""
        if self.w_speed <= 0.0:
            z = np.zeros_like(np.asarray(h, float))
            return z, z
        agl = np.clip(np.asarray(h, float) - self.ground_elev, 0.5, 500.0)
        s = self.w_speed * (agl / 10.0) ** (1.0 / 7.0)
        return s * self.w_unit[0], s * self.w_unit[1]

    # ------------------------------------------------------------------ aero tables
    def coeffs(self, alpha, mach, sb, gear=0.0):
        """Vectorised 4-D multilinear lookup of trimmed CL, CD."""
        alpha = np.asarray(alpha, float)
        shp = np.broadcast_shapes(alpha.shape, np.shape(mach), np.shape(sb), np.shape(gear))
        a = np.broadcast_to(alpha, shp).ravel()
        mm = np.broadcast_to(np.asarray(mach, float), shp).ravel()
        s = np.broadcast_to(np.asarray(sb, float), shp).ravel()
        g = np.broadcast_to(np.asarray(gear, float), shp).ravel()
        nA, nM, nS, nG = self.shape
        fa = np.clip((a - self.a0) / self.da, 0.0, nA - 1.000001)
        ia = fa.astype(np.int64)
        wa = fa - ia
        mc = np.clip(mm, self.mg[0], self.mg[-1])
        im = np.clip(np.searchsorted(self.mg, mc, side="right") - 1, 0, nM - 2)
        wm = (mc - self.mg[im]) / (self.mg[im + 1] - self.mg[im])
        sc = np.clip(s, self.sg[0], self.sg[-1])
        is_ = np.clip(np.searchsorted(self.sg, sc, side="right") - 1, 0, nS - 2)
        ws = (sc - self.sg[is_]) / (self.sg[is_ + 1] - self.sg[is_])
        gc = np.clip(g, 0.0, 1.0)
        ig = np.zeros_like(ia)
        wg = gc
        st = self.strides
        base = ia * st[0] + im * st[1] + is_ * st[2] + ig * st[3]
        out = np.zeros((len(a), 2))
        for da_, wa_ in ((0, 1.0 - wa), (1, wa)):
            for dm, wm_ in ((0, 1.0 - wm), (1, wm)):
                for ds, ws_ in ((0, 1.0 - ws), (1, ws)):
                    w3 = wa_ * wm_ * ws_
                    idx = base + da_ * st[0] + dm * st[1] + ds * st[2]
                    out += (w3 * (1.0 - wg))[:, None] * self.T[idx]
                    out += (w3 * wg)[:, None] * self.T[idx + st[3]]
        return out[:, 0].reshape(shp), out[:, 1].reshape(shp)

    @staticmethod
    def bank_rate(mach):
        """Achievable bank rate [rad/s]: yaw authority collapses supersonically."""
        return np.radians(np.interp(mach, [0.9, 1.3, 2.0, 3.0], [8.0, 5.0, 2.5, 2.0]))

    def alpha_max(self, mach):
        return np.interp(mach, self.mg, self.amax_M)

    def ld_best(self, mach):
        return np.interp(mach, self.mg, self.ld_best_M)

    def ld_min(self, mach):
        return np.interp(mach, self.mg, self.ld_min_M)

    def alpha_best(self, mach):
        return np.interp(mach, self.mg, self.alpha_best_M)

    def alpha_for_cl(self, cl, mach, sb=0.0, gear=0.0):
        """Vectorised inverse of CL(alpha) (monotone part) by bisection on the table."""
        cl = np.asarray(cl, float)
        lo = np.full(cl.shape, -4.0)
        hi = np.minimum(self.alpha_max(mach) + 1.0, 30.0) * np.ones_like(cl)
        for _ in range(18):
            mid = 0.5 * (lo + hi)
            c, _d = self.coeffs(mid, mach, sb, gear)
            up = c < cl
            lo = np.where(up, mid, lo)
            hi = np.where(up, hi, mid)
        return 0.5 * (lo + hi)

    # ------------------------------------------------------------------ dynamics
    def gravity(self, h):
        return G0 * (RE / (RE + np.asarray(h, float))) ** 2

    def aero_forces(self, X, U, gear=0.0):
        h, V = X[..., 2], np.maximum(X[..., 3], 1.0)
        rho, a = atm(h)
        M = V / a
        CL, CD = self.coeffs(U[..., 0], M, U[..., 2], gear)
        q = 0.5 * rho * V * V
        return q * self.S * CL, q * self.S * CD, q, M

    def f(self, X, U, gear=0.0):
        E, N, h, V, gam, psi = (X[..., i] for i in range(6))
        V = np.maximum(V, 1.0)
        L, D, q, M = self.aero_forces(X, U, gear)
        g = self.gravity(h)
        bank = U[..., 1]
        cg, sg = np.cos(gam), np.sin(gam)
        wE, wN = self.wind(h)
        m = self.m
        out = np.empty(X.shape)
        out[..., 0] = V * cg * np.sin(psi) + wE
        out[..., 1] = V * cg * np.cos(psi) + wN
        out[..., 2] = V * sg
        out[..., 3] = -D / m - g * sg
        out[..., 4] = (L * np.cos(bank) - m * g * cg) / (m * V)
        out[..., 5] = L * np.sin(bank) / (m * V * np.maximum(cg, 0.05))
        return out

    def flow(self, X0, U0, U1, dt, nsub=3, gear=0.0):
        """RK4 flow map over dt with first-order-hold controls u(t)=U0+(U1-U0) t/dt."""
        x = np.array(X0, float)
        dt = np.asarray(dt, float)[..., None]
        h = dt / nsub
        dU = U1 - U0
        for i in range(nsub):
            t0 = i / nsub
            tm = (i + 0.5) / nsub
            t1 = (i + 1) / nsub
            u0 = U0 + dU * t0
            um = U0 + dU * tm
            u1 = U0 + dU * t1
            k1 = self.f(x, u0, gear)
            k2 = self.f(x + 0.5 * h * k1, um, gear)
            k3 = self.f(x + 0.5 * h * k2, um, gear)
            k4 = self.f(x + h * k3, u1, gear)
            x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x

    # perturbation sizes for central differences: x(6), u0(3), u1(3), dt
    EPS_X = np.array([2.0, 2.0, 2.0, 0.05, 1e-4, 1e-4])
    EPS_U = np.array([0.02, 1e-3, 0.1])

    def flow_jac(self, X0, U0, U1, dt, nsub=3, gear=0.0, foh=True):
        """Flow map and its Jacobians for K intervals at once (central differences).

        Returns F (K,6), A (K,6,6), Bm (K,6,3), Bp (K,6,3), Sdt (K,6)  [dF/d dt].
        With foh=False the controls are held (U1 ignored) and Bp = 0.
        """
        K = X0.shape[0]
        nz = 13 if foh else 10
        npert = 1 + 2 * nz
        Xs = np.repeat(X0[None], npert, axis=0)
        U0s = np.repeat(U0[None], npert, axis=0)
        U1s = np.repeat((U1 if foh else U0)[None], npert, axis=0)
        dts = np.repeat(np.asarray(dt, float)[None], npert, axis=0)
        eps_dt = np.maximum(1e-3 * np.asarray(dt, float), 1e-3)
        j = 1
        for i in range(6):
            Xs[j, :, i] += self.EPS_X[i]
            Xs[j + 1, :, i] -= self.EPS_X[i]
            j += 2
        for i in range(3):
            U0s[j, :, i] += self.EPS_U[i]
            U0s[j + 1, :, i] -= self.EPS_U[i]
            if not foh:
                U1s[j, :, i] += self.EPS_U[i]
                U1s[j + 1, :, i] -= self.EPS_U[i]
            j += 2
        if foh:
            for i in range(3):
                U1s[j, :, i] += self.EPS_U[i]
                U1s[j + 1, :, i] -= self.EPS_U[i]
                j += 2
        dts[j] += eps_dt
        dts[j + 1] -= eps_dt
        Y = self.flow(Xs.reshape(-1, 6), U0s.reshape(-1, 3), U1s.reshape(-1, 3), dts.reshape(-1),
                      nsub, gear if np.ndim(gear) == 0 else np.tile(gear, npert))
        Y = Y.reshape(npert, K, 6)
        F = Y[0]
        A = np.empty((K, 6, 6))
        j = 1
        for i in range(6):
            A[:, :, i] = (Y[j] - Y[j + 1]) / (2 * self.EPS_X[i])
            j += 2
        Bm = np.empty((K, 6, 3))
        for i in range(3):
            Bm[:, :, i] = (Y[j] - Y[j + 1]) / (2 * self.EPS_U[i])
            j += 2
        Bp = np.zeros((K, 6, 3))
        if foh:
            for i in range(3):
                Bp[:, :, i] = (Y[j] - Y[j + 1]) / (2 * self.EPS_U[i])
                j += 2
        Sdt = (Y[j] - Y[j + 1]) / (2 * eps_dt)[:, None]
        return F, A, Bm, Bp, Sdt

    # ------------------------------------------------------------------ helpers
    def qbar(self, h, V):
        rho, a = atm(h)
        return 0.5 * rho * np.asarray(V) ** 2

    def eas(self, h, V):
        rho, a = atm(h)
        return np.asarray(V) * np.sqrt(rho / RHO0)

    def tas_from_eas(self, h, eas):
        rho, a = atm(h)
        return np.asarray(eas) / np.sqrt(rho / RHO0)

    def mach(self, h, V):
        rho, a = atm(h)
        return np.asarray(V) / a

    def nz(self, X, U, gear=0.0):
        L, D, q, M = self.aero_forces(X, U, gear)
        return L / (self.m * G0)

    @staticmethod
    def energy_height(h, V):
        return np.asarray(h) + np.asarray(V) ** 2 / (2 * G0)
