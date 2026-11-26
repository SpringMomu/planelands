"""Landing gear and structural ground contact.

Per wheel: oleo-pneumatic strut along body z (progressive spring + asymmetric damper,
bottoming), tyre normal force, rolling resistance, brakes with simple anti-skid
(friction limited to the peak value), slip-angle cornering force (tanh saturation),
friction circle, nose-wheel steering.  Structural points (tail scrape, belly, wing
tips, nozzles) get penalty contacts with Coulomb sliding friction.

Wing tips are not rigid points: the outer wing bends (its tip stiffness is in series with the
contact), and above the crush strength of the tip structure it deforms plastically, so the
contact force saturates and the contact point moves into the wing (permanent crush, a plastic
return mapping on the penalty spring).  A sliding contact also grinds material away (Archard
wear: worn volume = K N s / H).  Crush + wear are the damage the simulator judges; the friction
work, load and sliding velocity of every contact are accumulated for the sparks / dust the
display draws (``drain_scrape``).

Wet runway (``water`` = depth of the water film, set by the display from the ground wetness):
* tyre-to-runway friction falls with speed on a wet surface: the maximum braking coefficient of
  14 CFR 25.109(c)(1) for a smooth wet runway, interpolated in tyre pressure, blended in from dry
  as a damp film builds up (viscous hydroplaning is what that curve describes)
* dynamic hydroplaning in standing water deeper than the tread and the runway texture can drain:
  the water wedge under the footprint carries a share ~(V / V_p)^2 of the load, V_p = 9 sqrt(p) kt
  (NASA TN D-2056), and that share has no friction - braking and cornering are lost together
* braking = min(brake torque, anti-skid efficiency x available friction); cornering and the friction
  circle scale with the available friction
* displacement drag of the water the tyres push aside: 1/2 rho V^2 C_D h b per tyre, C_D 0.75,
  falling off above V_p when the tyre rides on the water (EASA CS-25 AMC 25.1591)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

import numpy as np

MU_DRY = 0.85            # peak tyre-runway friction, dry concrete
ANTISKID = 0.8           # anti-skid efficiency (fully modulating system)
TEXTURE_H = 0.4e-3       # water held in the runway macrotexture: not displaced by the tyre [m]
# 14 CFR 25.109(c)(1): wet-runway maximum braking coefficient, mu = sum c_i (V_kt / 100)^i, by tyre pressure
_CFR = ((50.0, (0.883, -0.851, 0.306, -0.0350)), (100.0, (0.804, -0.805, 0.320, -0.0437)),
        (200.0, (0.692, -0.658, 0.252, -0.0331)), (300.0, (0.614, -0.611, 0.263, -0.0401)))


def wet_mu_max(v_kt, psi):
    """Maximum tyre-to-ground braking coefficient on a smooth wet runway (14 CFR 25.109(c)(1))."""
    x = min(max(v_kt, 0.0), 220.0) / 100.0
    psi = min(max(psi, 50.0), 300.0)
    for (p0, c0), (p1, c1) in pairwise(_CFR):
        if psi <= p1:
            u = (psi - p0) / (p1 - p0)
            break
    m0 = c0[0] + x * (c0[1] + x * (c0[2] + x * c0[3]))
    m1 = c1[0] + x * (c1[1] + x * (c1[2] + x * c1[3]))
    return m0 + (m1 - m0) * u


def _ss(a, b, x):
    t = min(max((x - a) / (b - a), 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def tyre_friction(v, h, psi):
    """Available tyre-runway friction coefficient at ground speed ``v`` [m/s] on a water film ``h``
    [m] for a tyre at ``psi``, and the share of the load carried by the water (hydroplaning, 0..1)."""
    if h <= 0.0:
        return MU_DRY, 0.0
    mu = MU_DRY + (wet_mu_max(v / 0.514444, psi) - MU_DRY) * _ss(0.0, 0.2e-3, h)
    vp = 9.0 * math.sqrt(psi) * 0.514444
    lift = min(0.95 * _ss(1.0e-3, 3.0e-3, h) * (v / vp) ** 2, 0.95)
    return mu * (1.0 - lift), lift


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
    psi: float = 315.0       # tyre pressure (hydroplaning speed)
    width: float = 0.38      # section width of each of the two tyres [m]


class LandingGear:
    def __init__(self, geo):
        g = geo.raw["gear"]
        self.deploy_time = float(g["deploy_time_s"])
        mass = geo.mass
        W = mass * 9.80665
        wheels = []
        # static loads: nose ~20 %, mains ~40 % each (CG 3.53 m ahead of the mains)
        for key, share, brake, steer, psi, width in (("nose", 0.20, 0.0, True, 300.0, 0.22),
                                                     ("main_l", 0.40, 0.42, False, 315.0, 0.38),
                                                     ("main_r", 0.40, 0.42, False, 315.0, 0.38)):
            p = g[key]
            stroke = p["stroke"]
            s_static = 0.4 * stroke
            k0 = share * W / (s_static * (1.0 + 3.0 * (s_static / stroke) ** 2))
            m_eff = share * mass
            k_lin = k0 * (1.0 + 9.0 * (s_static / stroke) ** 2)
            c = 0.55 * 2.0 * math.sqrt(k_lin * m_eff)
            wheels.append(Wheel(key, np.array([p["x"], p["y"], p["z"]], dtype=float), stroke, k0,
                                c, 1.6 * c, mu_brake=brake, steer=steer, psi=psi, width=width))
        self.wheels = wheels
        self.water = 0.0          # water film on the runway [m]
        self.mu = [MU_DRY] * 3    # available friction per gear (last recorded)
        self.hydro = [0.0] * 3    # share of the load carried by the water (hydroplaning)
        self.pos = 0.0            # 0 = up, 1 = down & locked
        self.cmd_down = False
        self.wow = [False, False, False]
        self.compress = [0.0, 0.0, 0.0]
        s = geo.raw["tail_scrape_point"]
        wg = geo.raw["wing"]
        tip_y = float(wg["tip_y"])
        # wing tip lower surface: trailing edge, and ~25 % chord (thickest, lowest at small pitch)
        z_tip = float(wg["z_root"]) - (tip_y - float(wg["y_root"])) * math.tan(math.radians(float(wg["dihedral_deg"])))
        x_te, x_le = float(wg["te"][-1][0]), float(wg["le"][-1][0])
        c_tip = x_le - x_te
        x_q = x_le - 0.25 * c_tip
        z_q = z_tip + 0.5 * float(wg["t_over_c"][-1][1]) * c_tip
        self.struct_pts = np.array([
            s,                                   # tail / body-flap trailing edge
            [-14.6, -1.37, 1.45], [-14.6, 1.37, 1.45],   # lower SSME nozzles
            [x_te, -tip_y, z_tip + 0.02], [x_te, tip_y, z_tip + 0.02],   # wing tips (TE)
            [-3.5, 0.0, 2.32], [8.0, 0.0, 2.32], [18.5, 0.0, 1.95],   # belly
            [x_q, -tip_y + 0.15, z_q], [x_q, tip_y - 0.15, z_q],       # wing tips (lower surface, 25 % chord)
            # upper / side structure: only reached by a wreck rolling over after a crash (crew cabin,
            # payload bay doors, OMS pods, fuselage sides, vertical-tail tip and root)
            [13.4, 0.0, -3.25], [5.0, 0.0, -2.92], [-7.6, 0.0, -2.92],
            [-2.0, -2.25, -4.4], [-2.0, 2.25, -4.4], [-11.0, -2.25, -4.6], [-11.0, 2.25, -4.6],
            [12.0, -2.6, 0.0], [12.0, 2.6, 0.0], [-9.0, -2.7, 0.0], [-9.0, 2.7, 0.0],
            [-15.2, 0.0, -12.1], [-9.0, 0.0, -4.3],
        ], dtype=float)
        self.struct_names = ["tail", "nozzle_l", "nozzle_r", "wingtip_l", "wingtip_r",
                             "belly_aft", "belly_mid", "nose", "wingtip_fwd_l", "wingtip_fwd_r",
                             "cabin_top", "bay_fwd", "bay_aft", "oms_fwd_l", "oms_fwd_r", "oms_aft_l", "oms_aft_r",
                             "side_fwd_l", "side_fwd_r", "side_aft_l", "side_aft_r", "fin_tip", "fin_root"]
        # region of each point: what the simulator judges, what the display grinds (material mix)
        self.struct_region = ["tail", "nozzle", "nozzle", "wing_l", "wing_r",
                              "belly", "belly", "nose", "wing_l", "wing_r",
                              "upper", "upper", "upper", "upper", "upper", "upper", "upper",
                              "upper", "upper", "upper", "upper", "fin", "fin"]
        n = len(self.struct_pts)
        # the upper / side points are only checked when the simulator enables them (crashed, or a
        # steep bank / pitch): in normal flight and rollout they are metres above the ground
        self.n_lower = 10
        self.upper_on = False
        # contact law per point: k [N/m], c [N s/m], mu (sliding, on concrete), crush force [N],
        # Archard wear K / H [1/Pa] over the apparent contact area [m^2]
        #   rigid structure: stiff penalty.  Wing tip: tip bending stiffness of the outer wing
        #   (~0.15 m tip deflection at the crush load) in series with the contact, light structural
        #   damping (zeta ~0.15 on the ~8.5 t effective tip mass), aluminium skin-stringer box
        #   crushing at ~250 kN; tiles / aluminium / titanium fittings on concrete mu ~0.4.
        #   Wear: two-body abrasion by the concrete aggregate, K ~1e-2 over the hardness of the
        #   aluminium structure (~1.2 GPa; the silica tiles in front of it go within metres)
        rigid = (4.0e6, 3.0e5, 0.45, math.inf, 1.0e-11, 0.05)
        wing = (1.6e6, 3.5e4, 0.40, 2.5e5, 2.0e-11, 0.04)
        par = {"tail": rigid, "nozzle": (4.0e6, 3.0e5, 0.50, math.inf, 5.0e-12, 0.03), "belly": rigid,
               "nose": rigid, "wing_l": wing, "wing_r": wing, "upper": rigid,
               # fin: a thin box that bends and crushes like the outer wing
               "fin": (1.6e6, 3.5e4, 0.40, 2.5e5, 2.0e-11, 0.04)}
        self._sp_par = [par[r] for r in self.struct_region]
        self.struct_contact = np.zeros(n, dtype=bool)
        self.struct_sink = 0.0
        # last recorded contact state per point (normal force, sliding velocity, contact point,
        # penetration of the elastic contact, world-up in body axes)
        self._c_N = np.zeros(n)
        self._c_v = np.zeros((n, 3))
        self._c_p = np.zeros((n, 3))
        self._c_d = np.zeros(n)
        self._c_up = np.zeros((n, 3))
        self.damage = np.zeros((n, 3))            # permanent crush + wear offset of each point (body FRD)
        self._acc = [None] * n                    # per point accumulation since the last drain
        self._apply_damage()
        for w in self.wheels:
            w.rb = tuple(map(float, w.r))
        self.load = [0.0, 0.0, 0.0]

    def _apply_damage(self):
        eff = self.struct_pts + self.damage
        self.struct_eff = eff
        self._struct_list = [tuple(map(float, p)) for p in eff]

    def reset(self, down=False):
        self.pos = 1.0 if down else 0.0
        self.cmd_down = down
        self.wow = [False, False, False]
        self.compress = [0.0, 0.0, 0.0]
        self.load = [0.0, 0.0, 0.0]
        self.struct_contact[:] = False
        self.damage[:] = 0.0
        self._acc = [None] * len(self.struct_pts)
        self._apply_damage()

    def damage_depth(self, region):
        """Largest crush + wear depth [m] of the points in ``region``."""
        return max((float(np.linalg.norm(self.damage[i])) for i, r in enumerate(self.struct_region) if r == region),
                   default=0.0)

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
        load = [0.0, 0.0, 0.0]
        mus = [MU_DRY] * 3
        hyd = [0.0] * 3
        water = self.water
        h_disp = max(water - TEXTURE_H, 0.0)
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
                load[k] = N
                if w.steer and steer != 0.0:
                    cs, sn = math.cos(steer), math.sin(steer)
                    re, rn = fe * cs + fn * sn, -fe * sn + fn * cs
                else:
                    re, rn = fe, fn
                le, ln = rn, -re                    # right of the rolling direction
                vl = vpx * re + vpy * rn
                vt = vpx * le + vpy * ln
                slip = math.atan2(vt, abs(vl) + 0.5)
                # available friction on the (wet) runway; braking limited by the anti-skid
                mu, hp = tyre_friction(abs(vl), water, w.psi)
                mus[k], hyd[k] = mu, hp
                f_lat = -N * w.mu_lat * (mu / MU_DRY) * math.tanh(slip / 0.08727)
                f_lon = -N * (w.mu_roll + min(brakes * w.mu_brake, ANTISKID * mu)) * math.tanh(vl / 0.3)
                mag = math.hypot(f_lat, f_lon)
                lim = mu * N
                if mag > lim:
                    f_lat *= lim / mag
                    f_lon *= lim / mag
                if h_disp > 0.0:
                    # water pushed aside by the two tyres (less once they ride on it)
                    v2 = vl * vl
                    vp = 9.0 * math.sqrt(w.psi) * 0.514444
                    kd = 1.0 - 0.6 * _ss(vp, 1.3 * vp, abs(vl)) * _ss(1.0e-3, 3.0e-3, water)
                    f_lon -= 0.5 * 1000.0 * v2 * 0.75 * h_disp * 0.85 * w.width * 2.0 * kd * math.tanh(vl / 0.3)
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
        sp = self.struct_eff
        Uz = pz + sp[:, 0] * r20 + sp[:, 1] * r21 + sp[:, 2] * r22
        hg0 = ground_fn(px, py)
        sc_flags = np.zeros(len(sp), dtype=bool)
        sink = 0.0
        cand = np.where(Uz < hg0 + 8.0)[0]
        if not self.upper_on:
            cand = cand[cand < self.n_lower]
        for i in cand:
            bx, by, bz = self._struct_list[i]
            ex = px + r00 * bx + r01 * by + r02 * bz
            ey = py + r10 * bx + r11 * by + r12 * bz
            hg = ground_fn(ex, ey)
            d = hg - Uz[i]
            if d <= 0.0:
                continue
            cx = wq * bz - wr * by
            cy = wr * bx - wp * bz
            cz = wp * by - wq * bx
            vpx = vx + r00 * cx + r01 * cy + r02 * cz
            vpy = vy + r10 * cx + r11 * cy + r12 * cz
            vpz = vz + r20 * cx + r21 * cy + r22 * cz
            sink = max(sink, -vpz)
            kc, cc, mu, fmax, _, _ = self._sp_par[i]
            N = kc * d - cc * vpz
            N = min(max(N, 0.0), fmax)
            spd = math.hypot(vpx, vpy)
            if spd > 1e-6:
                k = -mu * N * math.tanh(spd / 0.5) / spd
                fwx, fwy = k * vpx, k * vpy
            else:
                fwx = fwy = 0.0
            if record:
                self._c_N[i] = N
                self._c_v[i] = (vpx, vpy, vpz)
                self._c_p[i] = (ex, ey, hg)
                self._c_d[i] = d
                self._c_up[i] = (r20, r21, r22)
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
            self.load = load
            self.mu = mus
            self.hydro = hyd
            self.struct_contact = sc_flags
            self.struct_sink = sink
        return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])

    # ------------------------------------------------------------------ damage + scrape record
    def accumulate(self, h):
        """After an integration step of length ``h`` whose first stage was recorded: plastic crush and
        wear of the structural contacts, and the friction work etc. for the display."""
        changed = False
        for i in np.flatnonzero(self.struct_contact):
            kc, _, mu, fmax, kw, area = self._sp_par[i]
            N = float(self._c_N[i])
            vx, vy, vz = (float(v) for v in self._c_v[i])
            spd = math.hypot(vx, vy)
            up = self._c_up[i]
            # crush: the elastic penetration beyond the crush load flows plastically (return mapping)
            dpl = float(self._c_d[i]) - fmax / kc if fmax < math.inf else 0.0
            # wear: worn depth = (K/H) N s / A
            dw = kw * N * spd * h / area
            if dpl > 0.0 or dw > 1e-7:
                self.damage[i] += (max(dpl, 0.0) + dw) * up
                changed = True
            a = self._acc[i]
            if a is None:
                a = self._acc[i] = {"W": 0.0, "J": 0.0, "dt": 0.0, "Nmax": 0.0, "sink": 0.0}
            a["W"] += mu * N * math.tanh(spd / 0.5) * spd * h       # friction work [J]
            a["J"] += N * h                                          # normal impulse [N s]
            a["dt"] += h
            a["Nmax"] = max(a["Nmax"], N)
            a["sink"] = max(a["sink"], -vz)
            a["p"] = self._c_p[i].tolist()
            a["v"] = [vx, vy, vz]
        if changed:
            self._apply_damage()

    def drain_scrape(self):
        """Structural contacts since the last call (each point at most once): body point (FRD, with
        its damage), world contact point (ENU), sliding velocity (ENU, m/s), mean / peak normal force
        (N), friction work (J), contact time (s), damage depth (m) and region."""
        out = []
        for i, a in enumerate(self._acc):
            if a is None:
                continue
            out.append({"id": self.struct_names[i], "region": self.struct_region[i],
                        "b": self.struct_eff[i].tolist(), "p": a["p"], "v": a["v"],
                        "N": a["J"] / max(a["dt"], 1e-9), "Nmax": a["Nmax"], "W": a["W"], "dt": a["dt"],
                        "sink": a["sink"], "dmg": float(np.linalg.norm(self.damage[i]))})
            self._acc[i] = None
        return out

    def any_near_ground(self, pos, R, ground_fn, margin=4.0):
        """True if any wheel or structural point is within ``margin`` metres of the ground."""
        for w in self.wheels:
            p = pos + R @ w.r
            if p[2] - ground_fn(p[0], p[1]) < margin:
                return True
        # every structural point: a banked wing tip reaches the ground long before the wheels
        z = pos[2] + self.struct_eff @ R[2]
        hg = ground_fn(pos[0], pos[1])
        idx = np.flatnonzero(z < hg + margin + 30.0)
        if not self.upper_on:
            idx = idx[idx < self.n_lower]
        for i in idx:
            p = pos + R @ self.struct_eff[i]
            if p[2] - ground_fn(p[0], p[1]) < margin:
                return True
        return False
