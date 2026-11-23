"""Landing gear and structural ground contact.

Per wheel: oleo-pneumatic strut along body z (progressive spring + asymmetric damper,
bottoming), tyre normal force, rolling resistance, brakes with simple anti-skid
(friction limited to the peak value), slip-angle cornering force (tanh saturation),
friction circle, nose-wheel steering.  Structural points (tail scrape, belly, wing
tips, nozzles) get stiff penalty contacts used for crash / scrape detection.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Wheel:
    name: str
    r: np.ndarray            # FRD position of the unloaded tyre contact point (gear extended)
    stroke: float
    k0: float                # spring constant at zero compression [N/m]
    c_comp: float            # damping in compression [N s/m]
    c_ext: float             # damping in extension
    mu_lat: float = 0.75
    mu_roll: float = 0.015
    mu_brake: float = 0.0    # 0 for the nose wheel
    steer: bool = False


class LandingGear:
    def __init__(self, geo):
        g = geo.raw["gear"]
        self.deploy_time = float(g["deploy_time_s"])
        mass = geo.mass
        W = mass * 9.80665
        wheels = []
        # static loads: nose ~20 %, mains ~40 % each (CG 3.53 m ahead of the mains)
        for key, share, brake, steer in (("nose", 0.20, 0.0, True), ("main_l", 0.40, 0.42, False),
                                         ("main_r", 0.40, 0.42, False)):
            p = g[key]
            stroke = p["stroke"]
            s_static = 0.4 * stroke
            k0 = share * W / (s_static * (1.0 + 3.0 * (s_static / stroke) ** 2))
            m_eff = share * mass
            k_lin = k0 * (1.0 + 9.0 * (s_static / stroke) ** 2)
            c = 0.55 * 2.0 * math.sqrt(k_lin * m_eff)
            wheels.append(Wheel(key, np.array([p["x"], p["y"], p["z"]], dtype=float), stroke, k0,
                                c, 1.6 * c, mu_brake=brake, steer=steer))
        self.wheels = wheels
        self.pos = 0.0            # 0 = up, 1 = down & locked
        self.cmd_down = False
        self.wow = [False, False, False]
        self.compress = [0.0, 0.0, 0.0]
        s = geo.raw["tail_scrape_point"]
        self.struct_pts = np.array([
            s,                                   # tail / body-flap trailing edge
            [-14.6, -1.37, 1.45], [-14.6, 1.37, 1.45],   # lower SSME nozzles
            [-12.2, -11.9, 1.05], [-12.2, 11.9, 1.05],   # wing tips (TE)
            [-3.5, 0.0, 2.32], [8.0, 0.0, 2.32], [18.5, 0.0, 1.95],   # belly
        ], dtype=float)
        self.struct_names = ["tail", "nozzle_l", "nozzle_r", "wingtip_l", "wingtip_r",
                             "belly_aft", "belly_mid", "nose"]
        self.struct_contact = np.zeros(len(self.struct_pts), dtype=bool)
        self.struct_sink = 0.0
        self._struct_list = [tuple(map(float, p)) for p in self.struct_pts]
        for w in self.wheels:
            w.rb = tuple(map(float, w.r))

    def reset(self, down=False):
        self.pos = 1.0 if down else 0.0
        self.cmd_down = down
        self.wow = [False, False, False]
        self.compress = [0.0, 0.0, 0.0]
        self.struct_contact[:] = False

    def animate(self, dt):
        target = 1.0 if self.cmd_down else 0.0
        step = dt / self.deploy_time
        if self.pos < target:
            self.pos = min(target, self.pos + step)
        elif self.pos > target:
            self.pos = max(target, self.pos - step)

    @property
    def locked(self):
        return self.pos >= 0.999

    # ------------------------------------------------------------------ forces
    def forces(self, pos, vel, R, omega, ground_fn, brakes=0.0, steer=0.0, record=False):
        """Return (F_body, M_body). pos/vel ENU, R body->ENU, omega body rates.

        ground_fn(e, n) -> ground height.  ``steer`` = nose-wheel angle [rad] (+ = right).
        Pure-float implementation (called at every RK4 stage at 400 Hz near the ground).
        """
        (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = R.tolist()
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        vx, vy, vz = float(vel[0]), float(vel[1]), float(vel[2])
        wp, wq, wr = float(omega[0]), float(omega[1]), float(omega[2])
        Fx = Fy = Fz = Mx = My = Mz = 0.0
        cos_strut = max(-r22, 0.2)
        nf = math.hypot(r00, r10)
        fe, fn = (r00 / nf, r10 / nf) if nf > 1e-6 else (0.0, 1.0)
        wow = [False, False, False]
        comp = [0.0, 0.0, 0.0]
        if self.locked:
            for k, w in enumerate(self.wheels):
                bx, by, bz = w.rb
                ex = px + r00 * bx + r01 * by + r02 * bz
                ey = py + r10 * bx + r11 * by + r12 * bz
                ez = pz + r20 * bx + r21 * by + r22 * bz
                d = ground_fn(ex, ey) - ez
                if d <= 0.0:
                    continue
                s = d / cos_strut
                # point velocity: v + R (w x r)
                cx = wq * bz - wr * by
                cy = wr * bx - wp * bz
                cz = wp * by - wq * bx
                vpx = vx + r00 * cx + r01 * cy + r02 * cz
                vpy = vy + r10 * cx + r11 * cy + r12 * cz
                vpz = vz + r20 * cx + r21 * cy + r22 * cz
                sdot = -vpz / cos_strut
                sc = min(s, w.stroke)
                fs = w.k0 * sc * (1.0 + 3.0 * (sc / w.stroke) ** 2)
                fs += (w.c_comp if sdot > 0 else w.c_ext) * sdot
                if s > w.stroke:                    # bottoming
                    fs += 2.0e7 * (s - w.stroke) + 2.0e5 * max(sdot, 0.0)
                N = fs if fs > 0.0 else 0.0
                if N <= 0.0:
                    continue
                wow[k] = True
                comp[k] = sc
                if w.steer and steer != 0.0:
                    cs, sn = math.cos(steer), math.sin(steer)
                    re, rn = fe * cs + fn * sn, -fe * sn + fn * cs
                else:
                    re, rn = fe, fn
                le, ln = rn, -re                    # right of the rolling direction
                vl = vpx * re + vpy * rn
                vt = vpx * le + vpy * ln
                slip = math.atan2(vt, abs(vl) + 0.5)
                f_lat = -N * w.mu_lat * math.tanh(slip / 0.08727)
                f_lon = -N * (w.mu_roll + brakes * w.mu_brake) * math.tanh(vl / 0.3)
                mag = math.hypot(f_lat, f_lon)
                lim = 0.85 * N
                if mag > lim:
                    f_lat *= lim / mag
                    f_lon *= lim / mag
                fwx = f_lon * re + f_lat * le
                fwy = f_lon * rn + f_lat * ln
                fwz = N
                # world -> body (R^T)
                fbx = r00 * fwx + r10 * fwy + r20 * fwz
                fby = r01 * fwx + r11 * fwy + r21 * fwz
                fbz = r02 * fwx + r12 * fwy + r22 * fwz
                rcz = bz - sc
                Fx += fbx
                Fy += fby
                Fz += fbz
                Mx += by * fbz - rcz * fby
                My += rcz * fbx - bx * fbz
                Mz += bx * fby - by * fbx
        # structural contacts (vectorised pre-check, exact per-point force)
        sp = self.struct_pts
        Uz = pz + sp[:, 0] * r20 + sp[:, 1] * r21 + sp[:, 2] * r22
        hg0 = ground_fn(px, py)
        sc_flags = np.zeros(len(sp), dtype=bool)
        sink = 0.0
        cand = np.where(Uz < hg0 + 8.0)[0]
        for i in cand:
            bx, by, bz = self._struct_list[i]
            ex = px + r00 * bx + r01 * by + r02 * bz
            ey = py + r10 * bx + r11 * by + r12 * bz
            d = ground_fn(ex, ey) - Uz[i]
            if d <= 0.0:
                continue
            cx = wq * bz - wr * by
            cy = wr * bx - wp * bz
            cz = wp * by - wq * bx
            vpx = vx + r00 * cx + r01 * cy + r02 * cz
            vpy = vy + r10 * cx + r11 * cy + r12 * cz
            vpz = vz + r20 * cx + r21 * cy + r22 * cz
            sink = max(sink, -vpz)
            N = max(4.0e6 * d - 3.0e5 * vpz, 0.0)
            spd = math.hypot(vpx, vpy)
            if spd > 1e-6:
                k = -0.45 * N * math.tanh(spd / 0.5) / spd
                fwx, fwy = k * vpx, k * vpy
            else:
                fwx = fwy = 0.0
            fwz = N
            fbx = r00 * fwx + r10 * fwy + r20 * fwz
            fby = r01 * fwx + r11 * fwy + r21 * fwz
            fbz = r02 * fwx + r12 * fwy + r22 * fwz
            Fx += fbx
            Fy += fby
            Fz += fbz
            Mx += by * fbz - bz * fby
            My += bz * fbx - bx * fbz
            Mz += bx * fby - by * fbx
            sc_flags[i] = True
        if record:
            self.wow = wow
            self.compress = comp
            self.struct_contact = sc_flags
            self.struct_sink = sink
        return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])

    def any_near_ground(self, pos, R, ground_fn, margin=4.0):
        """True if any wheel or structural point is within ``margin`` metres of the ground."""
        for w in self.wheels:
            p = pos + R @ w.r
            if p[2] - ground_fn(p[0], p[1]) < margin:
                return True
        for r in self.struct_pts[[0, 5, 7]]:
            p = pos + R @ r
            if p[2] - ground_fn(p[0], p[1]) < margin:
                return True
        return False
