"""Dense reference trajectory (TAEM plan + A/L reference) with spatial projection."""
from __future__ import annotations

import math

import numpy as np

from .approach import PH_NAMES
from .geometry import TWO_PI

SUB_ACQ, SUB_HAC, SUB_PRF = 0, 1, 2
SUB_NAMES = ["ACQ", "HAC", "PRF"]


class RefPath:
    """Time-indexed dense reference (dt <= 1 s).

    seg: 0 = TAEM, 1 = A/L.  sub: TAEM sub-phase (ACQ/HAC/PRF) or A/L phase index."""

    def __init__(self, model, t0, taem_X, taem_U, taem_tf, al, hac=None, dt=1.0):
        self.model = model
        K = taem_X.shape[0] - 1
        Dt = taem_tf / K
        m = max(int(math.ceil(Dt / dt)), 1)
        fr = np.arange(m) / m
        X0 = np.repeat(taem_X[:-1], m, axis=0)
        U0 = np.repeat(taem_U[:-1], m, axis=0)
        U1 = np.repeat(taem_U[1:], m, axis=0)
        f = np.tile(fr, K)
        Ue = U0 + (U1 - U0) * f[:, None]
        Xd = model.flow(X0, U0, Ue, np.maximum(f * Dt, 1e-6), nsub=2)
        Xd[f == 0] = taem_X[:-1]
        td = np.repeat(np.arange(K) * Dt, m) + f * Dt
        Xt = np.vstack([Xd, taem_X[-1:]])
        Ut = np.vstack([Ue, taem_U[-1:]])
        tt = np.concatenate([td, [taem_tf]])
        # A/L part (downsample to ~dt spacing, unwrap the heading onto the TAEM branch)
        ta = al.t
        sel = [0]
        for i in range(1, len(ta)):
            if ta[i] - ta[sel[-1]] >= dt * 0.999 or i == len(ta) - 1:
                sel.append(i)
        sel = np.array(sel)
        Xa = al.X[sel].copy()
        Ua = al.U[sel].copy()
        ga = al.gear[sel]
        pha = al.ph[sel]
        Xa[:, 5] += round((Xt[-1, 5] - Xa[0, 5]) / TWO_PI) * TWO_PI
        self.t = t0 + np.concatenate([tt, taem_tf + ta[sel][1:]])
        self.X = np.vstack([Xt, Xa[1:]])
        self.U = np.vstack([Ut, Ua[1:]])
        self.gear = np.concatenate([np.zeros(len(tt)), ga[1:]])
        self.seg = np.concatenate([np.zeros(len(tt), int), np.ones(len(sel) - 1, int)])
        self.i_ali = len(tt) - 1
        self.al = al
        self.hac = hac
        self.t0 = t0
        # TAEM sub-phases from the geometry: HAC = last continuous stretch within the HAC
        # annulus turning onto the runway, PRF = aligned with the runway before the ALI
        sub = np.zeros(len(self.t), int)
        rwy = al.rwy
        E, N = self.X[:, 0], self.X[:, 1]
        xr, yr = rwy.to_rwy(E, N)
        herr = np.abs((self.X[:, 5] - rwy.heading + math.pi) % TWO_PI - math.pi)
        prf = (np.abs(yr) < 600.0) & (herr < math.radians(12.0)) & (np.arange(len(sub)) <= self.i_ali)
        if hac is not None:
            c = np.array(hac["center"])
            d = np.hypot(E - c[0], N - c[1])
            on = (np.abs(d - hac["radius"]) < 0.35 * hac["radius"]) & (np.arange(len(sub)) <= self.i_ali)
            # keep only the final contiguous stretch before PRF
            idx = np.where(on & ~prf)[0]
            if len(idx):
                last = idx[-1]
                first = last
                while first - 1 >= 0 and on[first - 1] and not prf[first - 1]:
                    first -= 1
                sub[first:last + 1] = SUB_HAC
        sub[prf] = SUB_PRF
        al_sub = np.concatenate([np.zeros(len(tt), int), pha[1:]])
        self.sub = np.where(self.seg == 1, al_sub, sub)
        # cumulative horizontal arc length
        ds = np.hypot(np.diff(E), np.diff(N))
        self.s = np.concatenate([[0.0], np.cumsum(ds)])
        self._last = 0

    # ------------------------------------------------------------------ queries
    def sub_name(self, i):
        if self.seg[i] == 1:
            return PH_NAMES[int(self.sub[i])]
        return SUB_NAMES[int(self.sub[i])]

    def project(self, x, hint=None, window=(-30, 400), w_h=3.0):
        """Closest reference point (3-D, altitude weighted) near the previous index.
        Returns (fractional index, reference time)."""
        n = len(self.t)
        i0 = self._last if hint is None else int(hint)
        a = max(i0 + window[0], 0)
        b = min(i0 + window[1], n - 1)
        W = np.array([1.0, 1.0, w_h])
        xp = np.asarray(x[:3], float)
        P = self.X[a:b + 1, :3]
        d2 = np.sum(((P - xp) * W) ** 2, axis=1)
        k = a + int(np.argmin(d2))
        best_f, best_d = float(k), float(d2[k - a])
        for j in (k - 1, k):
            if 0 <= j < n - 1:
                p0, p1 = self.X[j, :3], self.X[j + 1, :3]
                v = (p1 - p0) * W
                w = (xp - p0) * W
                L2 = float(v @ v)
                if L2 > 1e-9:
                    f = min(max(float(w @ v) / L2, 0.0), 1.0)
                    d = float(np.sum(((p0 + f * (p1 - p0) - xp) * W) ** 2))
                    if d < best_d:
                        best_f, best_d = j + f, d
        self._last = int(best_f)
        return best_f, self.time_at_index(best_f)

    def time_at_index(self, fi):
        i = int(min(max(math.floor(fi), 0), len(self.t) - 2))
        f = fi - i
        return float(self.t[i] + f * (self.t[i + 1] - self.t[i]))

    def interp_index(self, fi):
        i = int(min(max(math.floor(fi), 0), len(self.t) - 2))
        f = min(max(fi - i, 0.0), 1.0)
        return (self.X[i] + f * (self.X[i + 1] - self.X[i]), self.U[i] + f * (self.U[i + 1] - self.U[i]),
                self.gear[i] if f < 0.5 else self.gear[i + 1])

    def at_times(self, tq):
        """Interpolated (X, U, gear) at absolute times tq (clamped to the reference)."""
        tq = np.clip(np.asarray(tq, float), self.t[0], self.t[-1])
        X = np.column_stack([np.interp(tq, self.t, self.X[:, j]) for j in range(6)])
        U = np.column_stack([np.interp(tq, self.t, self.U[:, j]) for j in range(3)])
        g = np.interp(tq, self.t, self.gear)
        return X, U, g

    @property
    def t_ali(self):
        return float(self.t[self.i_ali])

    @property
    def t_end(self):
        return float(self.t[-1])

    def range_to_go(self, fi):
        i = int(min(max(fi, 0), len(self.s) - 1))
        return float(self.s[-1] - self.s[i])
