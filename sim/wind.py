"""Wind model: steady wind with a boundary-layer profile + Dryden turbulence.

* Steady wind: ``speed`` is the 10 m reference wind, ``dir_from`` the meteorological
  direction the wind blows FROM (deg, clockwise from north). Below 500 m AGL a 1/7
  power law is used, above that the wind is constant (gradient wind).
* Turbulence: Dryden forms of MIL-F-8785C (low altitude < 1000 ft and medium/high
  altitude >= 2000 ft, linearly blended in between). Linear velocity components only
  (u along the flight path ~ body x, v lateral, w vertical), generated in body axes.
  ``intensity`` 0..1 maps to light (~0.3) .. severe (1.0).
* Automatic weather (``auto``): the user values are means; speed, direction and turbulence wander
  slowly (Ornstein-Uhlenbeck), and a storm forcing sent by the client (turbulence level, gust
  front / outflow strength, up- or downdraft at the vehicle, derived from the cloud field) makes
  everything rougher inside and under cumulonimbus. Vertical drafts fade out near the ground.
"""
from __future__ import annotations

import math

import numpy as np

FT = 0.3048


class WindModel:
    def __init__(self, seed: int | None = None):
        self.speed = 0.0          # m/s at 10 m
        self.dir_from = 0.0       # deg
        self.intensity = 0.0      # 0..1
        self.rng = np.random.default_rng(seed)
        # Dryden filter states: u (1st order), v and w (2 states each)
        self._u = 0.0
        self._v1 = self._v2 = 0.0
        self._w1 = self._w2 = 0.0
        self.gust_body = np.zeros(3)
        # automatic weather: user means + slow random deviations + storm forcing
        self.auto = False         # the web client switches this on (tests stay deterministic)
        self.base_speed = 0.0
        self.base_dir = 0.0
        self.base_int = 0.0
        self._ds = self._dd = self._di = self._dn = 0.0
        self.storm_turb = self.storm_gust = self.draft_target = 0.0
        self._st = self._sg = self._sd = 0.0
        self.draft = 0.0          # vertical wind (m/s, + up) above ~150 m AGL

    def set(self, speed=None, dir_from=None, intensity=None, auto=None):
        if speed is not None:
            self.base_speed = float(np.clip(speed, 0.0, 60.0))
        if dir_from is not None:
            self.base_dir = float(dir_from) % 360.0
        if intensity is not None:
            self.base_int = float(np.clip(intensity, 0.0, 1.0))
        if auto is not None:
            self.auto = bool(auto)
        if not self.auto:
            self.speed, self.dir_from, self.intensity, self.draft = self.base_speed, self.base_dir, self.base_int, 0.0

    def set_storm(self, turb=0.0, gust=0.0, draft=0.0):
        """storm forcing at the vehicle: turbulence level 0..1, extra wind (m/s), vertical draft target (m/s)"""
        self.storm_turb = float(np.clip(turb, 0.0, 1.0))
        self.storm_gust = float(np.clip(gust, 0.0, 20.0))
        self.draft_target = float(np.clip(draft, -12.0, 20.0))

    def _evolve(self, dt: float):
        if not self.auto:
            self.speed, self.dir_from, self.intensity, self.draft = self.base_speed, self.base_dir, self.base_int, 0.0
            return
        n = self.rng.standard_normal(4)

        def ou(x, tau, sigma, k):
            e = math.exp(-dt / tau)
            return x * e + sigma * math.sqrt(1.0 - e * e) * n[k]

        self._ds = ou(self._ds, 90.0, 1.0, 0)        # speed deviation (normalised)
        self._dd = ou(self._dd, 150.0, 14.0, 1)      # direction deviation (deg)
        self._di = ou(self._di, 45.0, 0.08, 2)       # turbulence deviation
        self._dn = ou(self._dn, 5.0, 1.0, 3)         # fast storm gustiness
        k = 1.0 - math.exp(-dt / 2.0)
        self._st += (self.storm_turb - self._st) * k
        self._sg += (self.storm_gust - self._sg) * k
        self._sd += (self.draft_target - self._sd) * (1.0 - math.exp(-dt / 4.0))
        spd = self.base_speed + (0.25 * self.base_speed + 1.5) * self._ds + self._sg * (1.0 + 0.35 * self._dn)
        self.speed = float(np.clip(spd, 0.0, 60.0))
        self.dir_from = (self.base_dir + self._dd + 30.0 * self._st * self._dn) % 360.0
        self.intensity = float(np.clip(max(self.base_int + self._di, self._st * (0.85 + 0.15 * self._dn)), 0.0, 1.0))
        self.draft = self._sd * (1.0 + 0.35 * self._dn)

    def vertical(self, agl: float) -> float:
        """up/downdraft (m/s, + up) at height agl: fades out in the lowest 150 m (outflow takes over)"""
        return self.draft * min(1.0, max(agl, 0.0) / 150.0)

    def reset_turbulence(self):
        self._u = self._v1 = self._v2 = self._w1 = self._w2 = 0.0
        self.gust_body[:] = 0.0

    # ------------------------------------------------------------------ steady
    def steady(self, agl: float) -> np.ndarray:
        """Steady wind vector (ENU, m/s) at height ``agl`` above ground."""
        if self.speed <= 0.0:
            return np.zeros(3)
        h = min(max(agl, 0.5), 500.0)
        s = self.speed * (h / 10.0) ** (1.0 / 7.0)
        d = math.radians(self.dir_from)
        # blowing FROM dir -> vector points the opposite way
        return np.array([-s * math.sin(d), -s * math.cos(d), 0.0])

    # -------------------------------------------------------------- turbulence
    def _scales(self, agl: float):
        """Return (sigma_u, sigma_v, sigma_w, L_u, L_v, L_w) in SI units."""
        hft = max(agl / FT, 10.0)
        # low-altitude model (h < 1000 ft)
        w20 = 45.0 * 0.5144 * self.intensity       # wind at 20 ft: severe = 45 kt
        sw_lo = 0.1 * w20
        k = (0.177 + 0.000823 * min(hft, 1000.0))
        su_lo = sw_lo / k ** 0.4
        Lw_lo = min(hft, 1000.0) * FT
        Lu_lo = min(hft, 1000.0) / k ** 1.2 * FT
        # medium / high altitude (>= 2000 ft): isotropic, sigma from intensity
        s_hi = 3.6 * self.intensity * (1.0 if agl < 15000.0 else max(0.2, 1.0 - (agl - 15000.0) / 15000.0))
        L_hi = 1750.0 * FT
        if hft <= 1000.0:
            return su_lo, su_lo, sw_lo, Lu_lo, Lu_lo, Lw_lo
        if hft >= 2000.0:
            return s_hi, s_hi, s_hi, L_hi, L_hi, L_hi / 2.0
        f = (hft - 1000.0) / 1000.0
        lerp = lambda a, b: a + (b - a) * f
        return (lerp(su_lo, s_hi), lerp(su_lo, s_hi), lerp(sw_lo, s_hi),
                lerp(Lu_lo, L_hi), lerp(Lu_lo, L_hi), lerp(Lw_lo, L_hi / 2.0))

    def step(self, dt: float, airspeed: float, agl: float) -> np.ndarray:
        """Advance the weather and the Dryden filters by ``dt``; returns gust in BODY axes (m/s)."""
        self._evolve(dt)
        if self.intensity <= 0.0:
            self.gust_body[:] = 0.0
            return self.gust_body
        V = max(airspeed, 20.0)
        su, sv, sw, Lu, Lv, Lw = self._scales(agl)
        n = self.rng.standard_normal(3)
        # u: first-order Gauss-Markov with exact discretisation
        tau = Lu / V
        e = math.exp(-dt / tau)
        self._u = e * self._u + su * math.sqrt(1.0 - e * e) * n[0]

        # v, w: (1 + sqrt3 tau s)/(1 + tau s)^2 driven by white noise of intensity
        # S = sigma^2 tau  (variance of output = S / tau = sigma^2)
        def second(z1, z2, sigma, L, nn):
            tau = L / V
            S = sigma * sigma * tau
            white = math.sqrt(S / dt) * nn
            a = 1.0 - math.exp(-dt / tau)
            z1 = z1 + a * (white - z1)
            z2 = z2 + a * (z1 - z2)
            y = math.sqrt(3.0) * z1 + (1.0 - math.sqrt(3.0)) * z2
            return z1, z2, y

        self._v1, self._v2, v = second(self._v1, self._v2, sv, Lv, n[1])
        self._w1, self._w2, w = second(self._w1, self._w2, sw, Lw, n[2])
        self.gust_body[0] = self._u
        self.gust_body[1] = v
        self.gust_body[2] = w
        return self.gust_body
