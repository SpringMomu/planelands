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
    gamma_ogs_deg: float = -20.0     # outer glide slope (Orbiter lightweight OGS; matches the OGS PAPI)
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
            if r * ang < 1.0:
                return              # no turn (e.g. straight-in): no duplicate zero-length sample
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


@dataclass
class LeadPath:
    """S-turn ("lead-out") in front of a Dubins route, for straight-in-like starts with too much
    energy where a larger HAC or another loop changes nothing or far too much: turn ``beta`` rad
    toward ``s0`` (+1 right) on a circle of radius ``r0``, fly ``a`` metres straight, then the CSC
    route ``tail`` to WP2.  ``a`` lengthens the track continuously (about 2 a (1 - cos beta) for the
    out-and-back).  Has the ``DubinsPath`` interface used by the candidates / initial guess."""
    p0: np.ndarray
    psi0: float
    r0: float
    s0: int
    beta: float
    a: float
    tail: DubinsPath

    @property
    def length(self):
        return self.r0 * self.beta + self.a + self.tail.length

    loops = property(lambda self: self.tail.loops)
    c2 = property(lambda self: self.tail.c2)
    r2 = property(lambda self: self.tail.r2)
    s2 = property(lambda self: self.tail.s2)
    p2 = property(lambda self: self.tail.p2)
    psi2 = property(lambda self: self.tail.psi2)
    turn1 = property(lambda self: self.beta)

    def sample(self, ds=200.0):
        t0 = heading_vec(self.psi0)
        c0 = self.p0 + self.s0 * self.r0 * right_of(t0)
        pts, psis, kap, lab = [], [], [], []
        n = max(int(math.ceil(self.r0 * self.beta / ds)), 1)
        for i in range(n):
            ps = self.psi0 + self.s0 * self.beta * i / n
            pts.append(c0 - self.s0 * self.r0 * right_of(heading_vec(ps)))
            psis.append(ps)
            kap.append(self.s0 / self.r0)
            lab.append(0)
        psl = self.psi0 + self.s0 * self.beta
        pl = c0 - self.s0 * self.r0 * right_of(heading_vec(psl))
        if self.a >= 1.0:                  # (orbit: no straight - no duplicate sample at the tail start)
            n = max(int(math.ceil(self.a / ds)), 1)
            for i in range(n):
                pts.append(pl + heading_vec(psl) * self.a * i / n)
                psis.append(psl)
                kap.append(0.0)
                lab.append(1)
        P = np.array(pts)
        E, N, psi, k, lb = self.tail.sample(ds)
        psi = psi - self.tail.psi0 + psl          # continue the unwrapped heading
        return (np.concatenate([P[:, 0], E]), np.concatenate([P[:, 1], N]), np.concatenate([psis, psi]),
                np.concatenate([kap, k]), np.concatenate([lab, lb]))


def lead_path(p0, psi0, r0, s0, beta, a, r1, s1, p2, psi2, r2, s2):
    """LeadPath from (p0, psi0) to (p2, psi2): lead-out turn / straight, then a CSC route."""
    t0 = heading_vec(psi0)
    c0 = p0 + s0 * r0 * right_of(t0)
    psl = psi0 + s0 * beta
    pl = c0 - s0 * r0 * right_of(heading_vec(psl)) + heading_vec(psl) * a
    tail = dubins_csc(pl, psl, r1, s1, p2, psi2, r2, s2)
    if tail is None:
        return None
    return LeadPath(p0=np.asarray(p0, float), psi0=psi0, r0=r0, s0=s0, beta=beta, a=a, tail=tail)


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
    relaxed: bool = False     # stretched path (enlarged HAC / extra loops) for a high-energy start
    head: float = 0.0         # mean headwind along the path (m/s)
    dE_min_rob: float = 0.0   # energy needed with the headwind 30 % stronger (relaxed paths)
    energy: str = "ok"        # energy direction found by select_plan: ok / high / low

    @property
    def feasible(self):
        if self.relaxed:
            # stretched paths must still make it if the headwind is stronger than forecast
            return max(self.dE_min, self.dE_min_rob) * 1.04 <= self.dE_avail <= self.dE_max * 0.97
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


def path_energy_budget(model, path: DubinsPath, x0, x_ali, ds=400.0, head_k=1.0):
    """Energy needed (best L/D) and dissipable (worst L/D) to fly ``path`` from x0 to x_ali,
    using the Mach profile of a linear-energy / blended-qbar glide.  ``head_k`` scales the
    headwind components (robustness check against a stronger headwind than forecast)."""
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
    if head_k != 1.0:
        wpar = np.where(wpar < 0.0, wpar * head_k, wpar)
    gs = np.clip((V + wpar) / V, 0.3, 2.0)
    ldb = model.ld_best(M) * turn_eff * gs
    ldm = model.ld_min(M) * gs * np.where(np.abs(kap) > 0, 0.85, 1.0)
    mid = lambda v: 0.5 * (v[1:] + v[:-1])
    dE_min = float(np.sum(seg / mid(ldb)))
    dE_max = float(np.sum(seg / mid(ldm)))
    return dE_min, dE_max, Eh0 - Ehf, (E, N, psi, kap, lab, s, h, V, M, q)


# ----------------------------------------------------------------------------- first turn
R1_MIN, R1_MAX = 3000.0, 40000.0
TURN_BANK_MAX = math.radians(55.0)   # planning bank for the initial turn (the SCP allows 60 / 70 deg)
TURN_NZ_MAX = 2.0                    # ... and load factor (SCP soft limit 2.5 g)
TIGHT_R1_FROM = 6000.0               # offer the decelerating-turn radius once the 45 deg circle exceeds this


def effective_turn_radius(model, x0, turn=math.pi, n_steps=12):
    """Mean radius of an initial turn of ``turn`` rad flown at the lift-limited bank.

    A fast, high start does not keep its speed through a long turn: it glides down while turning,
    so at roughly constant dynamic pressure its true airspeed - and the radius, ~V^2 - shrinks.
    A constant-speed 45 deg circle at the initial speed (Mach 1.6 at 20 km: 22.9 km radius, a
    285 deg turn-back ~115 km long) wildly overestimates the ground track and made such starts look
    hopelessly short of energy.  The turn is integrated in ``n_steps`` arcs: bank limited by
    min(TURN_BANK_MAX, the load factor CL_max q S / (m g) allows, TURN_NZ_MAX); energy lost at the
    best L/D reduced by the turn (sqrt(cos bank)); altitude / speed from the energy height at the
    (clipped) current dynamic pressure.  Returns the arc length / ``turn``."""
    h, V = float(x0[2]), float(x0[3])
    q = float(np.clip(model.qbar(h, V), 6.0e3, 12.0e3))
    Eh = h + V * V / (2.0 * G0)
    W = model.m * G0
    L = 0.0
    dth = turn / n_steps
    for _ in range(n_steps):
        rho, a = atm(h)
        M = V / float(a)
        cl_max = float(model.coeffs(model.alpha_max(M), M, 0.0)[0])
        n_max = min(cl_max * q * model.S / W, TURN_NZ_MAX)
        bank = math.acos(1.0 / n_max) if n_max > 1.02 else math.radians(11.0)
        bank = min(bank, TURN_BANK_MAX)
        r = V * V / (G0 * math.tan(bank))
        ds = r * dth
        L += ds
        Eh -= ds / (float(model.ld_best(M)) * math.sqrt(math.cos(bank)))
        h = float(h_from_energy(np.array([Eh]), q)[0])
        rho, a = atm(h)
        V = math.sqrt(2.0 * q / float(rho))
    return L / turn


def first_turn_radii(model, x0):
    """Initial-turn radii for the Dubins candidates: the 45 deg circle at the current speed and, for
    fast starts (where that circle is large), the decelerating lift-limited turn."""
    r_nom = min(max(turn_radius(float(x0[3]), 45.0), R1_MIN), R1_MAX)
    if r_nom <= TIGHT_R1_FROM:
        return (r_nom,)
    r_eff = min(max(effective_turn_radius(model, x0), R1_MIN), R1_MAX)
    return (r_nom, r_eff) if r_eff < 0.85 * r_nom else (r_nom,)


def candidates(model, x0, ends, cfg: ApproachConfig, level=0, radii=None):
    out = []
    radii = radii or first_turn_radii(model, x0)
    psi0 = float(x0[5])
    p0 = np.array([x0[0], x0[1]])
    for rwy in ends.values():
        c = shortened(cfg, level)
        x_ali = ali_state(model, rwy, c)
        wp2 = np.array(rwy.from_rwy(c.x_wp2, 0.0))
        wE, wN = model.wind(rwy.elev + 300.0)
        tail = float(wE * rwy.d[0] + wN * rwy.d[1])
        for r1 in radii:
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


def energy_direction(cands):
    """Which way the energy is off for a set of (nominal) candidates: "ok" if one is inside its
    band; "high" if at least one route still leaves energy to spare (the field is in reach, the
    routes are just too short or fall in the gap between two loop counts); "low" if every route
    needs more energy than is available."""
    if any(c.feasible for c in cands):
        return "ok"
    if any(c.dE_avail > c.dE_max * 0.97 for c in cands):
        return "high"
    return "low"


BAND_TARGET = 0.35                         # aimed-for position in the energy band (plan_score)
K_GRID = (0.76, 1.0, 1.3, 1.7, 2.2)        # stretched: HAC radius scale, searched continuously ...
K_BISECT = 6                               # ... by bisection between these grid points
MAX_LOOPS_RELAXED = 4                      # ... with up to this many extra HAC loops
HEAD_K_ROBUST = 1.3                        # stretched paths must still reach with 30 % more headwind


def headwind_mean(model, prof):
    """Mean headwind (m/s, >= 0 parts only) along a sampled path profile."""
    E, N, psi, kap, lab, s, h, V, M, q = prof
    wE, wN = model.wind(h)
    head = np.maximum(-(wE * np.sin(psi) + wN * np.cos(psi)), 0.0)
    seg = np.diff(s)
    return float(np.sum(seg * 0.5 * (head[1:] + head[:-1])) / max(s[-1], 1.0))


def _with_prefinal(path: DubinsPath, rwy: RunwayEnd, c: ApproachConfig, ds=350.0):
    """Ground track of ``path`` plus the prefinal straight WP2 -> ALI, for an energy budget that
    covers the whole TAEM (the nominal candidates leave the prefinal out)."""
    E, N, psi, _, _ = path.sample(ds)
    n = max(int(c.d_prefinal / ds), 2)
    xs = np.linspace(c.x_wp2, c.x_ali, n + 1)[1:]
    Ep, Np = rwy.from_rwy(xs, 0.0)
    return TrackPath(np.concatenate([E, Ep]), np.concatenate([N, Np]),
                     np.concatenate([psi, np.full(n, psi[-1])]))


def _stretch_eval(model, x0, p0, psi0, r1, s1, rwy, cfg, k, side, loops, tail):
    """Stretched candidate with the HAC radius scaled by ``k`` (budget includes the prefinal)."""
    c = replace(cfg, r_hac=cfg.r_hac * k)
    x_ali = ali_state(model, rwy, c)
    wp2 = np.array(rwy.from_rwy(c.x_wp2, 0.0))
    p = dubins_csc(p0, psi0, r1, s1, wp2, rwy.heading, c.r_hac, side)
    if p is None:
        return None
    p.loops = loops
    tp = _with_prefinal(p, rwy, c)
    dmin, dmax, dav, prof = path_energy_budget(model, tp, x0, x_ali, ds=700.0)
    cand = Candidate(rwy=rwy, cfg=c, path=replace(p), side=side, L_taem=p.length + c.d_prefinal,
                     dE_avail=dav, dE_min=dmin, dE_max=dmax, tail=tail, level=0, relaxed=True,
                     head=headwind_mean(model, prof))
    cand._tp, cand._x_ali = tp, x_ali
    return cand


def _family_search(ev, grid, target, x0, model):
    """Continuous 1-D search over a path family: evaluate ``ev(v)`` on ``grid`` (values ordered so
    the track gets longer), bracket the energy-band position ``target`` and bisect it; otherwise the
    in-band grid point closest to it.  Returns the candidate (with ``dE_min_rob`` set) or None."""
    pts = [(v, ev(v)) for v in grid]
    pts = [(v, c) for v, c in pts if c is not None]
    if not pts:
        return None
    best = None
    for (va, ca), (vb, cb) in zip(pts[:-1], pts[1:]):
        if (ca.band_pos - target) * (cb.band_pos - target) <= 0.0:
            lo, hi, clo = va, vb, ca
            for _ in range(K_BISECT):
                vm = 0.5 * (lo + hi)
                cm = ev(vm)
                if cm is None:
                    break
                if (clo.band_pos - target) * (cm.band_pos - target) <= 0.0:
                    hi = vm
                else:
                    lo, clo = vm, cm
                best = cm
            if best is None:
                best = ca if abs(ca.band_pos - target) < abs(cb.band_pos - target) else cb
            break
    if best is None:
        ok = [c for _, c in pts if c.dE_min * 1.04 <= c.dE_avail <= c.dE_max * 0.97]
        if not ok:
            return None
        best = min(ok, key=lambda c: abs(c.band_pos - target))
    if best.dE_min * 1.04 <= best.dE_avail <= best.dE_max * 0.97:
        best.dE_min_rob = path_energy_budget(model, best._tp, x0, best._x_ali, ds=700.0, head_k=HEAD_K_ROBUST)[0]
    else:
        best.dE_min_rob = best.dE_min
    best._tp = best._x_ali = None
    return best


def stretched_candidates(model, x0, ends, cfg: ApproachConfig, target=BAND_TARGET, names=None, radii=None):
    """Too much energy for every nominal route (too close / too high or fast, or between two loop
    counts): lengthen the ground track *continuously*.  For every route topology (runway end, first
    turn radius and direction, HAC side, number of extra HAC loops) the HAC radius scale k in
    [K_GRID[0], K_GRID[-1]] is searched (grid, then bisection) for the energy-band position
    ``target``; the loop count only moves the search to the next stretch interval, so there are no
    gaps between "one loop too much energy" and "two loops too little".  Every candidate is rated
    for headwind exposure and checked against a 30 % stronger headwind (``dE_min_rob``)."""
    out = []
    radii = radii or first_turn_radii(model, x0)
    psi0 = float(x0[5])
    p0 = np.array([x0[0], x0[1]])
    for rwy in ends.values():
        if names is not None and rwy.name not in names:
            continue
        wE, wN = model.wind(rwy.elev + 300.0)
        tail = float(wE * rwy.d[0] + wN * rwy.d[1])
        for r1 in radii:
            for side in (+1, -1):
                for s1 in (+1, -1):
                    for loops in range(MAX_LOOPS_RELAXED + 1):
                        ev = (lambda k, loops=loops, r1=r1, side=side, s1=s1:
                              _stretch_eval(model, x0, p0, psi0, r1, s1, rwy, cfg, k, side, loops, tail))
                        c0 = ev(K_GRID[0])
                        if c0 is None:
                            break
                        if c0.dE_avail < c0.dE_min * 1.04:
                            break        # even the tightest HAC is too long: more loops only longer
                        best = _family_search(ev, K_GRID, target, x0, model)
                        if best is not None:
                            out.append(best)
    return out


STURN_BETA = (math.radians(60.0), math.radians(90.0))   # S-turn: lead-out angles ...
STURN_GRID = (0.0, 4000.0, 9000.0, 16000.0, 26000.0)      # ... and lead-out lengths (m), bisected
STURN_R0 = 4000.0                                          # lead-out turn radius (subsonic, ~32 deg)
ORBIT_R = (2800.0,)                                          # S-turn alternative: initial orbit, min radius (m) ...
ORBIT_R_SCALE = (1.0, 1.25, 1.6, 2.1, 2.8)                   # ... and radius grid (x the minimum), bisected


def _sturn_eval(model, x0, p0, psi0, r0, s0, beta, a, r1, s1, rwy, cfg, side, tail_w):
    x_ali = ali_state(model, rwy, cfg)
    wp2 = np.array(rwy.from_rwy(cfg.x_wp2, 0.0))
    p = lead_path(p0, psi0, r0, s0, beta, a, r1, s1, wp2, rwy.heading, cfg.r_hac, side)
    if p is None:
        return None
    tp = _with_prefinal(p, rwy, cfg)
    dmin, dmax, dav, prof = path_energy_budget(model, tp, x0, x_ali, ds=700.0)
    cand = Candidate(rwy=rwy, cfg=cfg, path=p, side=side, L_taem=p.length + cfg.d_prefinal,
                     dE_avail=dav, dE_min=dmin, dE_max=dmax, tail=tail_w, level=0, relaxed=True,
                     head=headwind_mean(model, prof))
    cand._tp, cand._x_ali = tp, x_ali
    return cand


def sturn_candidates(model, x0, ends, cfg: ApproachConfig, target=BAND_TARGET, names=None, radii=None):
    """Too much energy where neither a larger HAC nor another loop fits (typically straight-in or
    nearly so: the HAC turn is ~0, so its radius hardly matters, and one more loop is far too long):
    an S-turn - turn away by beta, fly the lead-out length ``a`` (searched continuously for the
    band position ``target``), then the nominal CSC route to the HAC."""
    out = []
    radii = radii or first_turn_radii(model, x0)
    psi0 = float(x0[5])
    p0 = np.array([x0[0], x0[1]])
    r0 = max(min(radii), STURN_R0)
    r1 = r0
    for rwy in ends.values():
        if names is not None and rwy.name not in names:
            continue
        wE, wN = model.wind(rwy.elev + 300.0)
        tail_w = float(wE * rwy.d[0] + wN * rwy.d[1])
        for beta in STURN_BETA:
            for s0 in (+1, -1):
                for side in (+1, -1):
                    for s1 in (+1, -1):
                        ev = (lambda a, beta=beta, s0=s0, side=side, s1=s1:
                              _sturn_eval(model, x0, p0, psi0, r0, s0, beta, a, r1, s1, rwy, cfg, side, tail_w))
                        best = _family_search(ev, STURN_GRID, target, x0, model)
                        if best is not None:
                            out.append(best)
        # close to the field (e.g. straight in, ~10 km to WP2): no room for an S-turn - a full
        # orbit first, its radius searched continuously, then the nominal route
        # orbit radius: from what the vehicle can fly now (the decelerating turn when fast) up
        r_lo = min(radii) if max(radii) > TIGHT_R1_FROM else ORBIT_R[0]
        orbit_grid = tuple(r_lo * f for f in ORBIT_R_SCALE)
        for r1n in radii:
            for side in (+1, -1):
                for s1 in (+1, -1):
                    ev = (lambda r, side=side, s1=s1, r1n=r1n:
                          _sturn_eval(model, x0, p0, psi0, r, s1, TWO_PI, 0.0, r1n, s1, rwy, cfg, side, tail_w))
                    best = _family_search(ev, orbit_grid, target, x0, model)
                    if best is not None:
                        out.append(best)
    return out


def plan_score(c: Candidate, prefer=None, target=BAND_TARGET):
    """Lower is better: aim for the middle-low part of the energy band, prefer into-wind,
    fewer HAC loops, shorter TAEM; stretched (relaxed) paths: least headwind exposure, then the
    HAC radius closest to nominal."""
    s = abs(c.band_pos - target)
    s += 0.04 * max(c.tail - 2.0, 0.0)
    s += 0.05 * c.path.loops
    s += 1.5e-6 * c.L_taem
    if c.relaxed:
        s += 0.06 * c.head + 0.08 * abs(c.cfg.r_hac / ApproachConfig.r_hac - 1.0)
    if isinstance(c.path, LeadPath):
        s += 0.05 + 4e-6 * c.path.a
    if prefer is not None and c.rwy.name == prefer:
        s -= 0.08
    return s


def _stretch_note(best):
    if isinstance(best.path, LeadPath) and best.path.beta > 6.0:
        return f"high energy: initial orbit ({best.path.r0/1000:.1f} km radius) before the approach"
    if isinstance(best.path, LeadPath):
        return (f"high energy: S-turn ({math.degrees(best.path.beta):.0f} deg "
                f"{'right' if best.path.s0 > 0 else 'left'}, {best.path.a/1000:.1f} km lead-out)")
    return (f"high energy: stretched approach (HAC radius {best.path.r2/1000:.1f} km, "
            f"{best.path.loops} extra loop{'s' if best.path.loops != 1 else ''})")


def select_plan(model, x0, ends, cfg: ApproachConfig, prefer=None):
    """Pick runway end / HAC side / loops / HAC radius.  Returns (candidate, reachable, note);
    ``candidate.energy`` is "ok", "high" or "low" (which way the energy was off for the nominal
    approach), or "turn" (enough energy for a direct glide, but too fast to turn back yet).

    The energy direction is decided first, on the nominal approach: only when *every* nominal route
    needs more energy than is available is a shortened (low-energy) approach used; when some route
    leaves energy to spare the track is stretched continuously (larger HAC, extra loops) instead.
    (Before, the shortened levels were tried first, so high-energy starts often ended up on the
    low-energy approach and were annunciated as LOW ENERGY.)"""
    radii = first_turn_radii(model, x0)
    nom = candidates(model, x0, ends, cfg, 0, radii)
    feas = [c for c in nom if c.feasible]
    if feas:
        best = min(feas, key=lambda c: plan_score(c, prefer))
        best.energy = "ok"
        return best, True, ""
    direction = energy_direction(nom)
    rel = None
    if direction == "high":
        rel = stretched_candidates(model, x0, ends, cfg, radii=radii)
        feas = [c for c in rel if c.feasible]
        if not feas:
            rel = rel + sturn_candidates(model, x0, ends, cfg, radii=radii)
            feas = [c for c in rel if c.feasible]
        if feas:
            best = min(feas, key=lambda c: plan_score(c, prefer))
            best.energy = "high"
            return best, True, _stretch_note(best)
    # low energy (or nothing stretched works): shortened approaches - lower ALI, shorter prefinal,
    # tighter HAC
    for level in (1, 2):
        cands = candidates(model, x0, ends, cfg, level, radii)
        feas = [c for c in cands if c.feasible]
        if feas:
            best = min(feas, key=lambda c: plan_score(c, prefer))
            best.energy = direction
            if direction == "high":
                return best, True, f"high energy: tighter HAC (shortened approach level {level})"
            return best, True, f"low energy: shortened approach (level {level})"
    # nothing feasible.  Low energy only if even the most economical path (shortest approach,
    # best L/D) is out of reach; otherwise there is energy to spare and the vehicle is too high /
    # fast for how close it is.  Long detours (e.g. a ~360 deg first turn) always "need" more than
    # is available, so they must not decide it.
    cands = candidates(model, x0, ends, cfg, 2, radii)
    cheapest = max(cands, key=lambda c: c.dE_avail / max(c.dE_min, 1.0))
    if direction == "low" and cheapest.dE_avail < cheapest.dE_min * 1.04:
        # every Dubins route is out of reach: really low energy, or only too fast to turn back yet?
        # (supersonic the vehicle has little more than 1 g of lift, so its turns are very wide.)
        # Turn limited if the same routes flown with a subsonic-size first turn would be in reach.
        if max(radii) > TIGHT_R1_FROM:
            tight = candidates(model, x0, ends, cfg, 2, (R1_MIN,))
            ct = max(tight, key=lambda c: c.dE_avail / max(c.dE_min, 1.0))
            if ct.dE_avail >= ct.dE_min * 1.04:
                name = ct.rwy.name
                cheapest = max((c for c in cands if c.rwy.name == name),
                               key=lambda c: c.dE_avail / max(c.dE_min, 1.0), default=cheapest)
                cheapest.energy = "turn"
                return cheapest, False, (f"TURN LIMITED: too fast to turn back toward runway {name} yet (the turn "
                                         f"needs {cheapest.dE_min/1000:.1f} km energy height, {cheapest.dE_avail/1000:.1f}"
                                         f" km available) - gliding toward the field")
        cheapest.energy = "low"
        return cheapest, False, (f"LOW ENERGY: runway {cheapest.rwy.name} needs {cheapest.dE_min/1000:.1f} km "
                                 f"energy height, {cheapest.dE_avail/1000:.1f} km available")
    # energy to spare: stretch the path (enlarged HAC, more loops), avoiding strong headwind legs
    if rel is None:
        rel = stretched_candidates(model, x0, ends, cfg, radii=radii)
        feas = [c for c in rel if c.feasible]
        if not feas:
            rel = rel + sturn_candidates(model, x0, ends, cfg, radii=radii)
            feas = [c for c in rel if c.feasible]
        if feas:
            best = min(feas, key=lambda c: plan_score(c, prefer))
            best.energy = "high"
            return best, True, _stretch_note(best)
    windy = [c for c in rel if c.dE_min * 1.04 <= c.dE_avail <= c.dE_max * 0.97]
    if windy:
        # only stretched paths with long legs into a strong headwind would work: not planned as
        # reachable (no margin if the headwind is stronger); flown as the best effort
        best = min(windy, key=lambda c: plan_score(c, prefer))
        best.energy = "high"
        return best, False, (f"HIGH ENERGY: runway {best.rwy.name} only reachable on a stretched approach into a "
                             f"strong headwind ({best.head:.0f} m/s average) - no margin")
    cands = cands + rel
    # closest to being able to dissipate: smallest excess over the dissipation limit
    high = [c for c in cands if c.dE_avail > c.dE_max * 0.97] or cands
    best = min(high, key=lambda c: c.dE_avail / max(c.dE_max, 1.0))
    best.energy = "high"
    return best, False, (f"HIGH ENERGY: runway {best.rwy.name} can dissipate at most {best.dE_max/1000:.1f} km "
                         f"energy height, {best.dE_avail/1000:.1f} km available - too close for this altitude / speed")


def stretched_pick(model, x0, ends, cfg: ApproachConfig, target, prefer=None):
    """Planner recovery when the SCP reached the ALI too high / fast: the stretched route whose
    estimated band position is ``target`` (below the one just flown, i.e. a longer track) and that
    is still reachable at best L/D.  None if there is none."""
    names = [prefer] if prefer in ends else None
    def pool(nm):
        rel = stretched_candidates(model, x0, ends, cfg, target=target, names=nm)
        ok = [c for c in rel if c.dE_min * 1.04 <= c.dE_avail]
        near = [c for c in ok if abs(c.band_pos - target) < 0.15]
        if not near:
            ok += [c for c in sturn_candidates(model, x0, ends, cfg, target=target, names=nm)
                   if c.dE_min * 1.04 <= c.dE_avail]
        return ok

    ok = pool(names)
    if not ok and names is not None:
        ok = pool(None)
    if not ok:
        return None
    best = min(ok, key=lambda c: plan_score(c, prefer, target=target))
    best.energy = "high"
    return best


class TrackPath:
    """An arbitrary ground track (e.g. the remaining TAEM reference) with the ``sample`` /
    ``length`` / HAC interface of ``DubinsPath``, so ``path_energy_budget`` can rate it."""

    def __init__(self, E, N, psi, hac=None):
        E, N, psi = (np.asarray(v, float) for v in (E, N, psi))
        seg = np.hypot(np.diff(E), np.diff(N))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.E, self.N, self.psi = E, N, psi
        self.loops = 0
        hac = hac or {"center": [0.0, 0.0], "radius": 1.0, "dir": 0}
        self.c2 = np.asarray(hac["center"], float)
        self.r2 = float(hac["radius"])
        self.s2 = int(hac["dir"])

    @property
    def length(self):
        return float(self.s[-1])

    def sample(self, ds=200.0):
        n = max(int(math.ceil(self.length / ds)), 1)
        sn = np.linspace(0.0, self.length, n + 1)
        E, N, psi = (np.interp(sn, self.s, v) for v in (self.E, self.N, self.psi))
        dpsi = np.gradient(psi, sn) if n > 1 else np.zeros_like(sn)
        kap = np.where(np.abs(dpsi) > 1.0 / 60000.0, dpsi, 0.0)
        return E, N, psi, kap, np.zeros(len(sn), int)


def rate_track(model, x0, X, rwy: RunwayEnd, cfg: ApproachConfig, hac=None, level=0):
    """Energy-band rating of the remaining TAEM reference ``X`` (nodes from now to the ALI),
    comparable with the Dubins candidates (prefinal straight excluded, like there)."""
    X = np.asarray(X, float)
    full = TrackPath(X[:, 0], X[:, 1], X[:, 5], hac)
    s_wp2 = max(full.length - cfg.d_prefinal, 1.0)
    k = int(np.searchsorted(full.s, s_wp2))
    E = np.concatenate([full.E[:k], [np.interp(s_wp2, full.s, full.E)]])
    N = np.concatenate([full.N[:k], [np.interp(s_wp2, full.s, full.N)]])
    psi = np.concatenate([full.psi[:k], [np.interp(s_wp2, full.s, full.psi)]])
    path = TrackPath(E, N, psi, hac)
    x_ali = ali_state(model, rwy, cfg)
    dmin, dmax, dav, _ = path_energy_budget(model, path, x0, x_ali, ds=700.0)
    wE, wN = model.wind(rwy.elev + 300.0)
    tail = float(wE * rwy.d[0] + wN * rwy.d[1])
    return Candidate(rwy=rwy, cfg=cfg, path=path, side=int(path.s2), L_taem=full.length, dE_avail=dav,
                     dE_min=dmin, dE_max=dmax, tail=tail, level=level)


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
    # drop repeated samples (zero-length segments make the altitude gradient / time NaN)
    keep = np.concatenate([[True], np.hypot(np.diff(E), np.diff(N)) > 1e-3])
    E, N, psi, kap, lab = E[keep], N[keep], psi[keep], kap[keep], lab[keep]
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
