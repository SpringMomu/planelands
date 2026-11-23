"""TAEM geometry: runway ends, approach-and-landing interface (ALI), heading alignment
cylinder (HAC), Dubins CSC ground tracks with energy-based path stretching, reachability
estimate and runway / HAC-side selection.

Runway frame for a landing end: x along the landing direction (threshold at x = 0),
y to the right of the landing direction.  Heights are MSL.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from .model import G0, KT, RHO0, atm

TWO_PI = 2.0 * math.pi


def wrap_pi(a):
    return (a + math.pi) % TWO_PI - math.pi


def heading_vec(psi):
    return np.array([math.sin(psi), math.cos(psi)])


def right_of(t):
    return np.array([t[1], -t[0]])


@dataclass
class RunwayEnd:
    name: str
    thr: np.ndarray        # threshold (E, N)
    d: np.ndarray          # unit landing direction (E, N)
    heading: float         # rad
    elev: float
    length: float
    width: float

    @property
    def r(self):
        return right_of(self.d)

    def to_rwy(self, E, N):
        dE, dN = np.asarray(E) - self.thr[0], np.asarray(N) - self.thr[1]
        return dE * self.d[0] + dN * self.d[1], dE * self.d[1] - dN * self.d[0]

    def from_rwy(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        return (self.thr[0] + x * self.d[0] + y * self.d[1],
                self.thr[1] + x * self.d[1] - y * self.d[0])


def runway_ends(world):
    rw = world["runway"]
    h = math.radians(rw["heading_deg"])
    c = np.array(rw["center_en"], float)
    d33 = heading_vec(h)
    L = rw["length"]
    e33 = RunwayEnd(rw["ends"][1], c - d33 * L / 2, d33, h, rw["elevation"], L, rw["width"])
    e15 = RunwayEnd(rw["ends"][0], c + d33 * L / 2, -d33, wrap_pi(h + math.pi) % TWO_PI,
                    rw["elevation"], L, rw["width"])
    return {e33.name: e33, e15.name: e15}


@dataclass
class ApproachConfig:
    gamma_ogs_deg: float = -18.0     # outer glide slope
    x_aim_ogs: float = -2300.0       # OGS aim point (before the threshold)
    h_ali: float = 3000.0            # ALI height above the runway
    eas_ogs: float = 290.0 * KT      # equivalent airspeed on the OGS
    d_prefinal: float = 2500.0       # straight leg between HAC exit (WP2) and the ALI
    r_hac: float = 5500.0            # HAC radius
    gamma_igs_deg: float = -1.5
    x_aim_igs: float = 450.0
    x_td: float = 760.0              # touchdown point past the threshold (Orbiter: 2500 ft)
    h_td_cg: float = 4.4             # CG height above the runway at touchdown
    r_preflare: float = 9000.0
    v_td: float = 102.0              # target touchdown true airspeed (~198 KTAS)

    @property
    def x_ali(self):
        return self.x_aim_ogs - self.h_ali / math.tan(math.radians(-self.gamma_ogs_deg))

    @property
    def x_wp2(self):
        return self.x_ali - self.d_prefinal


def shortened(cfg: ApproachConfig, level: int) -> ApproachConfig:
    """Low-energy fallbacks: lower ALI, shorter prefinal, tighter HAC."""
    if level <= 0:
        return cfg
    if level == 1:
        return replace(cfg, h_ali=2400.0, d_prefinal=1500.0, r_hac=4800.0)
    return replace(cfg, h_ali=1800.0, d_prefinal=800.0, r_hac=4200.0, gamma_ogs_deg=-16.0)


# ----------------------------------------------------------------------------- Dubins
@dataclass
class DubinsPath:
    p0: np.ndarray
    psi0: float
    r1: float
    s1: int                  # +1 right (clockwise), -1 left
    turn1: float             # rad (>= 0)
    t1: np.ndarray           # tangent departure point
    ell: float
    theta: float             # straight-leg heading
    t2: np.ndarray
    r2: float
    s2: int
    turn2: float
    c1: np.ndarray
    c2: np.ndarray
    p2: np.ndarray
    psi2: float
    loops: int = 0

    @property
    def length(self):
        return self.r1 * self.turn1 + self.ell + self.r2 * (self.turn2 + TWO_PI * self.loops)

    def sample(self, ds=200.0):
        """Return (E, N, psi_unwrapped, curvature_signed, label) samples along the path."""
        pts, psis, kap, lab = [], [], [], []

        def arc(c, r, s, psi_a, ang, label):
            n = max(int(math.ceil(r * ang / ds)), 1)
            for i in range(n):
                f = i / n
                ps = psi_a + s * ang * f
                t = heading_vec(ps)
                p = c - s * r * right_of(t)
                pts.append(p)
                psis.append(ps)
                kap.append(s / r)
                lab.append(label)

        psi_u = self.psi0
        arc(self.c1, self.r1, self.s1, psi_u, self.turn1, 0)
        psi_u = psi_u + self.s1 * self.turn1
        # straight
        n = max(int(math.ceil(self.ell / ds)), 1)
        th = psi_u
        for i in range(n):
            pts.append(self.t1 + heading_vec(th) * self.ell * i / n)
            psis.append(th)
            kap.append(0.0)
            lab.append(1)
        arc(self.c2, self.r2, self.s2, psi_u, self.turn2 + TWO_PI * self.loops, 2)
        psi_u = psi_u + self.s2 * (self.turn2 + TWO_PI * self.loops)
        pts.append(self.p2.copy())
        psis.append(psi_u)
        kap.append(0.0)
        lab.append(3)
        P = np.array(pts)
        return P[:, 0], P[:, 1], np.array(psis), np.array(kap), np.array(lab)


def dubins_csc(p0, psi0, r1, s1, p2, psi2, r2, s2):
    """CSC path from (p0, psi0) on a circle of radius r1 (turn s1) to (p2, psi2) on a
    circle of radius r2 (turn s2).  Returns None if the tangent does not exist."""
    t0 = heading_vec(psi0)
    tf = heading_vec(psi2)
    c1 = p0 + s1 * r1 * right_of(t0)
    c2 = p2 + s2 * r2 * right_of(tf)
    rho1, rho2 = s1 * r1, s2 * r2
    D = c2 - c1
    dist = math.hypot(D[0], D[1])
    k = rho2 - rho1
    if dist < abs(k) + 1e-6:
        return None
    ell = math.sqrt(dist * dist - k * k)
    th_d = math.atan2(D[0], D[1])
    th = th_d - math.atan2(k, ell)
    if s1 > 0:
        a1 = (th - psi0) % TWO_PI
    else:
        a1 = (psi0 - th) % TWO_PI
    if s2 > 0:
        a2 = (psi2 - th) % TWO_PI
    else:
        a2 = (th - psi2) % TWO_PI
    t = heading_vec(th)
    T1 = c1 - rho1 * right_of(t)
    T2 = c2 - rho2 * right_of(t)
    return DubinsPath(p0=np.asarray(p0, float), psi0=psi0, r1=r1, s1=s1, turn1=a1, t1=T1, ell=ell,
                      theta=th, t2=T2, r2=r2, s2=s2, turn2=a2, c1=c1, c2=c2,
                      p2=np.asarray(p2, float), psi2=psi2)


def turn_radius(V, bank_deg=45.0):
    return max(V, 60.0) ** 2 / (G0 * math.tan(math.radians(bank_deg)))


# ----------------------------------------------------------------------------- profiles
def h_from_energy(Eh, q, iters=30):
    """Altitude for energy height Eh at dynamic pressure q (vectorised bisection)."""
    Eh = np.asarray(Eh, float)
    q = np.broadcast_to(np.asarray(q, float), Eh.shape)
    lo = np.full(Eh.shape, -200.0)
    hi = np.maximum(Eh, 0.0) + 10.0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        rho, a = atm(mid)
        val = mid + q / (rho * G0)
        big = val > Eh
        hi = np.where(big, mid, hi)
        lo = np.where(big, lo, mid)
    return 0.5 * (lo + hi)


@dataclass
class Candidate:
    rwy: RunwayEnd
    cfg: ApproachConfig
    path: DubinsPath
    side: int                 # HAC side: +1 centre right of the landing direction (right turn)
    L_taem: float             # ground length to the ALI
    dE_avail: float           # energy height available down to the ALI
    dE_min: float             # energy needed flying at best L/D
    dE_max: float             # energy that can be dissipated at worst L/D
    tail: float               # tailwind component on final (m/s)
    level: int = 0

    @property
    def feasible(self):
        return self.dE_min * 1.04 <= self.dE_avail <= self.dE_max * 0.97

    @property
    def margin(self):
        """>0: inside the band.  Normalised position: 0 at the max-range edge."""
        return (self.dE_avail - self.dE_min) / max(self.dE_avail, 1.0)

    @property
    def band_pos(self):
        """0 = needs best L/D, 1 = needs max dissipation."""
        return (self.dE_avail - self.dE_min) / max(self.dE_max - self.dE_min, 1.0)

    def hac(self):
        return {"center": [float(self.path.c2[0]), float(self.path.c2[1])],
                "radius": float(self.path.r2), "dir": int(self.path.s2)}


def ali_state(model, rwy: RunwayEnd, cfg: ApproachConfig):
    x = cfg.x_ali
    E, N = rwy.from_rwy(x, 0.0)
    h = rwy.elev + cfg.h_ali
    V = float(model.tas_from_eas(h, cfg.eas_ogs))
    return np.array([E, N, h, V, math.radians(cfg.gamma_ogs_deg), rwy.heading])


def path_energy_budget(model, path: DubinsPath, x0, x_ali, ds=400.0):
    """Energy needed (best L/D) and dissipable (worst L/D) to fly ``path`` from x0 to x_ali,
    using the Mach profile of a linear-energy / blended-qbar glide."""
    E, N, psi, kap, lab = path.sample(ds)
    seg = np.hypot(np.diff(E), np.diff(N))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    Ltot = s[-1] if s[-1] > 0 else 1.0
    Eh0 = model.energy_height(x0[2], x0[3])
    Ehf = model.energy_height(x_ali[2], x_ali[3])
    f = s / Ltot
    Eh = Eh0 + (Ehf - Eh0) * f
    q0 = float(model.qbar(x0[2], x0[3]))
    qf = float(model.qbar(x_ali[2], x_ali[3]))
    q = q0 + (qf - q0) * np.clip(f * 1.5, 0, 1)
    h = h_from_energy(Eh, q)
    rho, a = atm(h)
    V = np.sqrt(2 * q / rho)
    M = V / a
    # turn penalty (bank needed for the curvature) and along-path wind
    bank = np.arctan(V * V * np.abs(kap) / G0)
    turn_eff = np.sqrt(np.clip(np.cos(np.minimum(bank, 1.2)), 0.3, 1.0))
    wE, wN = model.wind(h)
    wpar = wE * np.sin(psi) + wN * np.cos(psi)
    gs = np.clip((V + wpar) / V, 0.3, 2.0)
    ldb = model.ld_best(M) * turn_eff * gs
    ldm = model.ld_min(M) * gs * np.where(np.abs(kap) > 0, 0.85, 1.0)
    mid = lambda v: 0.5 * (v[1:] + v[:-1])
    dE_min = float(np.sum(seg / mid(ldb)))
    dE_max = float(np.sum(seg / mid(ldm)))
    return dE_min, dE_max, Eh0 - Ehf, (E, N, psi, kap, lab, s, h, V, M, q)


def candidates(model, x0, ends, cfg: ApproachConfig, level=0):
    out = []
    V0 = float(x0[3])
    r1 = min(max(turn_radius(V0, 45.0), 3000.0), 40000.0)
    psi0 = float(x0[5])
    p0 = np.array([x0[0], x0[1]])
    for rwy in ends.values():
        c = shortened(cfg, level)
        x_ali = ali_state(model, rwy, c)
        wp2 = np.array(rwy.from_rwy(c.x_wp2, 0.0))
        wE, wN = model.wind(rwy.elev + 300.0)
        tail = float(wE * rwy.d[0] + wN * rwy.d[1])
        for side in (+1, -1):
            for s1 in (+1, -1):
                p = dubins_csc(p0, psi0, r1, s1, wp2, rwy.heading, c.r_hac, side)
                if p is None:
                    continue
                for loops in (0, 1, 2):
                    p.loops = loops
                    dmin, dmax, dav, _ = path_energy_budget(model, p, x0, x_ali, ds=700.0)
                    L = p.length + c.d_prefinal
                    out.append(Candidate(rwy=rwy, cfg=c, path=replace(p), side=side, L_taem=L,
                                         dE_avail=dav, dE_min=dmin, dE_max=dmax, tail=tail, level=level))
                    if dav <= dmax * 0.97:
                        break            # extra HAC loops only when there is too much energy
    return out


def select_plan(model, x0, ends, cfg: ApproachConfig, prefer=None):
    """Pick runway end / HAC side / loops.  Returns (candidate, reachable, note)."""
    for level in (0, 1, 2):
        cands = candidates(model, x0, ends, cfg, level)
        feas = [c for c in cands if c.feasible]
        if feas:
            def score(c):
                # aim for the middle-low part of the energy band, prefer into-wind, fewer loops
                s = abs(c.band_pos - 0.35)
                s += 0.04 * max(c.tail - 2.0, 0.0)
                s += 0.05 * c.path.loops
                s += 1.5e-6 * c.L_taem
                if prefer is not None and c.rwy.name == prefer:
                    s -= 0.08
                return s
            best = min(feas, key=score)
            note = "" if level == 0 else f"low energy: shortened approach (level {level})"
            return best, True, note
    # nothing feasible: report the one with the best margin
    cands = candidates(model, x0, ends, cfg, 2)
    low = [c for c in cands if c.dE_avail < c.dE_min * 1.04]
    if low:
        best = max(low, key=lambda c: c.dE_avail / c.dE_min)
        return best, False, (f"LOW ENERGY: runway {best.rwy.name} needs {best.dE_min/1000:.1f} km energy "
                             f"height, {best.dE_avail/1000:.1f} km available")
    best = min(cands, key=lambda c: c.dE_avail / max(c.dE_max, 1.0))
    return best, False, "HIGH ENERGY: cannot dissipate enough energy before the ALI"


# ----------------------------------------------------------------------------- initial guess
def initial_guess(model, x0, cand: Candidate, n_nodes=40):
    """Geometric TAEM guess (current state -> ALI) resampled to n_nodes in time.

    Returns X (n,6), U (n,3) [alpha_deg, bank_rad, sb_deg], tf, labels (n,)."""
    rwy, c = cand.rwy, cand.cfg
    x_ali = ali_state(model, rwy, c)
    path = cand.path
    dmin, dmax, dav, prof = path_energy_budget(model, path, x0, x_ali, ds=250.0)
    E, N, psi, kap, lab, s, h, V, M, q = prof
    # append the prefinal straight WP2 -> ALI
    nP = max(int(c.d_prefinal / 250.0), 2)
    xs = np.linspace(c.x_wp2, c.x_ali, nP + 1)[1:]
    Ep, Np = rwy.from_rwy(xs, 0.0)
    E = np.concatenate([E, Ep])
    N = np.concatenate([N, Np])
    psi = np.concatenate([psi, np.full(nP, psi[-1])])
    kap = np.concatenate([kap, np.zeros(nP)])
    lab = np.concatenate([lab, np.full(nP, 3)])
    seg = np.hypot(np.diff(E), np.diff(N))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    f = s / max(s[-1], 1.0)
    Eh0 = model.energy_height(x0[2], x0[3])
    Ehf = model.energy_height(x_ali[2], x_ali[3])
    Eh = Eh0 + (Ehf - Eh0) * f
    q0 = float(model.qbar(x0[2], x0[3]))
    qf = float(model.qbar(x_ali[2], x_ali[3]))
    q = q0 + (qf - q0) * np.clip(f * 1.5, 0.0, 1.0)
    # keep the very first samples on the actual state, blend into the schedule
    h = h_from_energy(Eh, q)
    h[0] = x0[2]
    rho, a = atm(h)
    V = np.sqrt(2.0 * q / rho)
    V[0] = x0[3]
    # final node: exactly the ALI
    h[-1], V[-1] = x_ali[2], x_ali[3]
    # flight path angle from the altitude profile vs ground distance
    dh = np.gradient(h, s, edge_order=1) if len(s) > 2 else np.zeros_like(h)
    gam = np.arctan(dh)
    gam[0] = x0[4]
    gam[-1] = x_ali[4]
    gam = np.clip(gam, math.radians(-30), math.radians(8))
    # unwrapped heading: start from the actual heading
    psi = psi - psi[0] + x0[5]
    bank = np.clip(np.arctan(V * V * kap / G0), -math.radians(55), math.radians(55))
    n = np.cos(gam) / np.cos(bank)
    rho, a = atm(h)
    CL = n * model.m * G0 / (0.5 * rho * V * V * model.S)
    Mach = V / a
    ld_need = (s[-1] / max(Eh0 - Ehf, 100.0))
    ldb = float(np.mean(model.ld_best(Mach)))
    sb0 = float(np.clip((1.0 - ld_need / ldb) * 160.0, 0.0, 70.0)) if ld_need < ldb else 0.0
    sb = np.full(len(s), sb0)
    alpha = model.alpha_for_cl(CL, Mach, sb, 0.0)
    alpha = np.clip(alpha, 1.0, model.alpha_max(Mach))
    # time along the path
    wE, wN = model.wind(h)
    gs = np.maximum(V * np.cos(gam) + wE * np.sin(psi) + wN * np.cos(psi), 30.0)
    dt = seg / (0.5 * (gs[1:] + gs[:-1]))
    t = np.concatenate([[0.0], np.cumsum(dt)])
    tf = float(t[-1])
    # resample uniformly in time
    tn = np.linspace(0.0, tf, n_nodes)
    X = np.column_stack([np.interp(tn, t, v) for v in (E, N, h, V, gam, psi)])
    U = np.column_stack([np.interp(tn, t, v) for v in (alpha, bank, sb)])
    labels = np.interp(tn, t, lab).round().astype(int)
    X[0] = x0
    X[-1] = x_ali
    X[-1, 5] = x_ali[5] + round((psi[-1] - x_ali[5]) / TWO_PI) * TWO_PI
    U[-1, 1] = 0.0
    return X, U, tf, labels
