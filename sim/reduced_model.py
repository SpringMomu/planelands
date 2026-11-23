"""Reduced 3-DOF point-mass model for guidance (SCP / MPC in a later phase).

State  x = [E, N, h, V, gamma, psi]
    E, N, h : position ENU [m] (h above mean sea level)
    V       : true airspeed [m/s]
    gamma   : air-relative flight-path angle [rad]
    psi     : air-relative heading [rad] (from north, clockwise)
Control u = [alpha_deg, bank_deg, speedbrake_deg]   (bank = wind-axis bank angle mu)

Dynamics (flat earth, constant horizontal wind w = (wE, wN)):
    E'   = V cos(gamma) sin(psi) + wE
    N'   = V cos(gamma) cos(psi) + wN
    h'   = V sin(gamma)
    V'   = -D/m - g sin(gamma)
    gam' = (L cos(mu) - m g cos(gamma)) / (m V)
    psi' =  L sin(mu) / (m V cos(gamma))
with L, D from TRIMMED coefficients CL(alpha, M, sb, gear), CD(alpha, M, sb, gear)
tabulated from the full 6-DOF component aero model (sim/trim.py; the elevon/body-flap
trim drag is therefore included).
"""
from __future__ import annotations

import math

import numpy as np

from .atmosphere import atmosphere, gravity


class TrimTables:
    def __init__(self, t: dict):
        self.alpha = np.asarray(t["alpha"], float)
        self.mach = np.asarray(t["mach"], float)
        self.sb = np.asarray(t["sb"], float)
        self.gear = np.asarray(t["gear"], float)
        self.CL = np.asarray(t["CL"], float)
        self.CD = np.asarray(t["CD"], float)
        self.DE = np.asarray(t["DE"], float)
        self.BF = np.asarray(t["BF"], float)
        self.trimmable = np.isfinite(self.DE)
        self.DEf = np.where(self.trimmable, self.DE, 0.0)
        self.BFf = np.where(self.trimmable, self.BF, 0.0)

    @staticmethod
    def _axis(grid, v):
        v = min(max(v, grid[0]), grid[-1])
        i = int(np.searchsorted(grid, v) - 1)
        i = min(max(i, 0), len(grid) - 2)
        w = (v - grid[i]) / (grid[i + 1] - grid[i])
        return i, w

    def _interp(self, arr, alpha, mach, sb, gear):
        ia, wa = self._axis(self.alpha, alpha)
        im, wm = self._axis(self.mach, mach)
        isb, ws = self._axis(self.sb, sb)
        ig, wg = self._axis(self.gear, gear)
        sub = arr[ia:ia + 2, im:im + 2, isb:isb + 2, ig:ig + 2]
        wA = np.array([1 - wa, wa])[:, None, None, None]
        wM = np.array([1 - wm, wm])[None, :, None, None]
        wS = np.array([1 - ws, ws])[None, None, :, None]
        wG = np.array([1 - wg, wg])[None, None, None, :]
        return float((sub * wA * wM * wS * wG).sum())

    def cl_cd(self, alpha_deg, mach, sb_deg=0.0, gear=0.0):
        return (self._interp(self.CL, alpha_deg, mach, sb_deg, gear),
                self._interp(self.CD, alpha_deg, mach, sb_deg, gear))

    def trim_surfaces(self, alpha_deg, mach, sb_deg=0.0, gear=0.0):
        """Trimmed (symmetric elevon, body flap) in degrees."""
        return (self._interp(self.DEf, alpha_deg, mach, sb_deg, gear),
                self._interp(self.BFf, alpha_deg, mach, sb_deg, gear))

    def _column(self, arr, mach, sb, gear):
        im, wm = self._axis(self.mach, mach)
        isb, ws = self._axis(self.sb, sb)
        ig, wg = self._axis(self.gear, gear)
        sub = arr[:, im:im + 2, isb:isb + 2, ig:ig + 2]
        w = (np.array([1 - wm, wm])[:, None, None] * np.array([1 - ws, ws])[None, :, None]
             * np.array([1 - wg, wg])[None, None, :])
        return (sub * w[None]).sum(axis=(1, 2, 3))

    def trim_alpha_max(self, mach, sb=0.0, gear=0.0):
        im, _ = self._axis(self.mach, mach)
        ok = self.trimmable[:, im:im + 2, :, 0].all(axis=(1, 2))
        a = self.alpha[ok]
        return float(a.max()) if len(a) else 10.0

    def alpha_for_cl(self, cl, mach, sb_deg=0.0, gear=0.0, amin=-4.0, amax=30.0):
        col = self._column(self.CL, mach, sb_deg, gear)
        m = (self.alpha >= amin) & (self.alpha <= amax)
        a, c = self.alpha[m], col[m]
        # CL is monotonic over the usable range
        k = int(np.argmax(c))
        a, c = a[:k + 1], c[:k + 1]
        return float(np.interp(cl, c, a))

    def best_ld_alpha(self, mach, sb_deg=0.0, gear=0.0):
        cl = self._column(self.CL, mach, sb_deg, gear)
        cd = self._column(self.CD, mach, sb_deg, gear)
        im, _ = self._axis(self.mach, mach)
        ok = self.trimmable[:, im:im + 2, 0, 0].all(axis=1) & (self.alpha > 0.5) & (self.alpha <= 25.0)
        ld = np.where(ok, cl / np.maximum(cd, 1e-6), -1e9)
        k = int(np.argmax(ld))
        if 0 < k < len(ld) - 1 and ld[k - 1] > -1e8 and ld[k + 1] > -1e8:
            y0, y1, y2 = ld[k - 1], ld[k], ld[k + 1]
            den = y0 - 2 * y1 + y2
            off = 0.5 * (y0 - y2) / den if abs(den) > 1e-12 else 0.0
            return float(self.alpha[k] + np.clip(off, -0.5, 0.5) * (self.alpha[1] - self.alpha[0]))
        return float(self.alpha[k])


class ReducedModel:
    def __init__(self, tables: TrimTables, mass: float, S: float):
        self.tab = tables
        self.m = mass
        self.S = S

    def aero(self, x, u, gear=0.0):
        h, V = x[2], x[3]
        T, p, rho, a, mu = atmosphere(h)
        M = V / a
        CL, CD = self.tab.cl_cd(u[0], M, u[2], gear)
        q = 0.5 * rho * V * V
        return q * self.S * CL, q * self.S * CD, M, q

    def f(self, x, u, wind=(0.0, 0.0), gear=0.0):
        E, N, h, V, gam, psi = x
        V = max(V, 1.0)
        L, D, M, q = self.aero(x, u, gear)
        g = gravity(h)
        mu = math.radians(u[1])
        cg = math.cos(gam)
        return np.array([
            V * cg * math.sin(psi) + wind[0],
            V * cg * math.cos(psi) + wind[1],
            V * math.sin(gam),
            -D / self.m - g * math.sin(gam),
            (L * math.cos(mu) - self.m * g * cg) / (self.m * V),
            L * math.sin(mu) / (self.m * V * max(cg, 0.05)),
        ])

    def step(self, x, u, dt, wind=(0.0, 0.0), gear=0.0):
        k1 = self.f(x, u, wind, gear)
        k2 = self.f(x + 0.5 * dt * k1, u, wind, gear)
        k3 = self.f(x + 0.5 * dt * k2, u, wind, gear)
        k4 = self.f(x + dt * k3, u, wind, gear)
        return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def linearize(self, x, u, wind=(0.0, 0.0), gear=0.0, eps_x=None, eps_u=None):
        """Finite-difference Jacobians A = df/dx (6x6), B = df/du (6x3) (u in degrees)."""
        x = np.asarray(x, float)
        u = np.asarray(u, float)
        eps_x = eps_x if eps_x is not None else np.array([1.0, 1.0, 1.0, 0.1, 1e-4, 1e-4])
        eps_u = eps_u if eps_u is not None else np.array([0.05, 0.1, 0.5])
        A = np.zeros((6, 6))
        B = np.zeros((6, 3))
        for i in range(6):
            dx = np.zeros(6)
            dx[i] = eps_x[i]
            A[:, i] = (self.f(x + dx, u, wind, gear) - self.f(x - dx, u, wind, gear)) / (2 * eps_x[i])
        for j in range(3):
            du = np.zeros(3)
            du[j] = eps_u[j]
            B[:, j] = (self.f(x, u + du, wind, gear) - self.f(x, u - du, wind, gear)) / (2 * eps_u[j])
        return A, B
