"""Approach & landing (A/L) reference and terminal laws, Orbiter style.

Vertical profile along the runway centreline (x = distance past the threshold):
    OGS       steep outer glide slope (gamma_ogs) aimed at x_aim_ogs before the threshold,
              flown at constant EAS with the speedbrake modulating drag
    PREFLARE  circular pull-up (radius r_preflare) tangent to OGS and IGS
    IGS       shallow inner glide slope (-1.5 deg) aimed x_aim_igs past the threshold
    FLARE     cubic blend from the IGS slope to a ~0.6 m/s sink at touchdown (x_td)
The speed profile is integrated with the trimmed 3-DOF aero along this geometry: EAS hold
on the OGS (speedbrake solves for the required drag), then a constant speedbrake setting
from preflare to touchdown chosen by bisection so the vehicle arrives at ``v_td``; gear
down 90 m above the runway.  Wind: along-track wind changes the air-relative flight-path
angle, crosswind gives the crab angle (air-relative heading).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import ApproachConfig, RunwayEnd, wrap_pi
from .model import G0, atm

PH_OGS, PH_PREFLARE, PH_IGS, PH_FLARE, PH_ROLL = 0, 1, 2, 3, 4
PH_NAMES = ["OGS", "PREFLARE", "IGS", "FLARE", "ROLLOUT"]


@dataclass
class ALRef:
    rwy: RunwayEnd
    cfg: ApproachConfig
    t: np.ndarray
    X: np.ndarray        # (n,6) E, N, h, V, gamma_air, psi_air
    U: np.ndarray        # (n,3) alpha_deg, bank_rad, sb_deg
    gear: np.ndarray     # (n,)
    x: np.ndarray        # along-runway coordinate
    ph: np.ndarray       # phase label
    keys: dict           # x positions of the phase boundaries
    sb_pf: float

    @property
    def x_ali_state(self):
        return self.X[0].copy()

    def polyline(self, step=10):
        P = self.X[::step, :3]
        if len(P) == 0 or not np.allclose(P[-1], self.X[-1, :3]):
            P = np.vstack([P, self.X[-1:, :3]])
        return P


def vertical_profile(cfg: ApproachConfig):
    """Return function H(x) (CG height above runway), slope and key x positions."""
    to = math.tan(math.radians(-cfg.gamma_ogs_deg))
    ti = math.tan(math.radians(-cfg.gamma_igs_deg))
    go, gi = math.radians(cfg.gamma_ogs_deg), math.radians(cfg.gamma_igs_deg)
    x_int = (cfg.x_aim_ogs * to - cfg.x_aim_igs * ti) / (to - ti)
    H_int = (cfg.x_aim_ogs - x_int) * to
    dgam = abs(go - gi)
    R = cfg.r_preflare
    dt_ = R * math.tan(dgam / 2.0)
    x_t1 = x_int - dt_ * math.cos(go)
    H_t1 = H_int + dt_ * math.sin(-go)
    x_t2 = x_int + dt_ * math.cos(gi)
    H_t2 = H_int - dt_ * math.sin(-gi)
    # arc centre: perpendicular (upward) from T1
    nx, nH = -math.sin(go), math.cos(go)
    xc, Hc = x_t1 + R * nx, H_t1 + R * nH
    # flare: cubic Hermite from the IGS slope to s_td at x_td, start chosen for a monotone blend
    s0 = ti
    s1 = 0.006
    x_td = cfg.x_td
    h_td = cfg.h_td_cg
    x_f0 = (s0 * cfg.x_aim_igs - h_td - 0.5 * (s0 + s1) * x_td) / (0.5 * (s0 - s1))
    x_f0 = max(x_f0, x_t2 + 50.0)
    H_f0 = (cfg.x_aim_igs - x_f0) * ti
    keys = dict(x_ali=cfg.x_ali, x_pf=x_t1, H_pf=H_t1, x_igs=x_t2, H_igs=H_t2, x_flare=x_f0,
                H_flare=H_f0, x_td=x_td, x_c=xc, H_c=Hc)

    def H(x):
        x = np.asarray(x, float)
        out = np.where(x <= x_t1, (cfg.x_aim_ogs - x) * to, 0.0)
        arc = (x > x_t1) & (x <= x_t2)
        out = np.where(arc, Hc - np.sqrt(np.maximum(R * R - (x - xc) ** 2, 0.0)), out)
        igs = (x > x_t2) & (x <= x_f0)
        out = np.where(igs, (cfg.x_aim_igs - x) * ti, out)
        fl = (x > x_f0) & (x <= x_td)
        L = x_td - x_f0
        u = np.clip((x - x_f0) / L, 0.0, 1.0)
        h00, h10, h01, h11 = 2 * u**3 - 3 * u**2 + 1, u**3 - 2 * u**2 + u, -2 * u**3 + 3 * u**2, u**3 - u**2
        Hf = h00 * H_f0 + h10 * L * (-s0) + h01 * h_td + h11 * L * (-s1)
        out = np.where(fl, Hf, out)
        out = np.where(x > x_td, h_td, out)
        return out

    def phase(x):
        x = np.asarray(x, float)
        return np.select([x <= x_t1, x <= x_t2, x <= x_f0, x <= x_td], [PH_OGS, PH_PREFLARE, PH_IGS, PH_FLARE],
                         PH_ROLL)

    return H, phase, keys


def _alpha_cl(model, cl, M, sb, gear):
    """Scalar CL -> alpha inversion on the monotone part of the trimmed CL(alpha) column."""
    ag = model.ag
    c, _ = model.coeffs(ag, M, sb, gear)
    amax = float(model.alpha_max(M)) + 2.0
    k = int(np.argmax(np.where(ag <= amax, c, -9.0)))
    return float(np.interp(cl, c[: k + 1], ag[: k + 1]))


def _integrate(model, rwy, cfg, H, phase, keys, sb_pf, dx=25.0, x_end=None):
    """Integrate the speed along the A/L geometry; returns arrays (or V_td only)."""
    x_end = keys["x_td"] if x_end is None else x_end
    xs = np.arange(cfg.x_ali, x_end + 1e-6, dx)
    Hs = H(xs)
    slope = np.gradient(Hs, xs)
    wE, wN = model.wind(rwy.elev + np.maximum(Hs, 0.0))
    wpar = wE * rwy.d[0] + wN * rwy.d[1]
    wcr = wE * rwy.d[1] - wN * rwy.d[0]            # toward the right of the landing direction
    n = len(xs)
    V = np.empty(n)
    gam = np.empty(n)
    alpha = np.empty(n)
    sb = np.empty(n)
    gear = np.empty(n)
    t = np.empty(n)
    h0 = rwy.elev + Hs[0]
    V[0] = float(model.tas_from_eas(h0, cfg.eas_ogs))
    t[0] = 0.0
    ph = phase(xs)
    m = model.m
    for i in range(n):
        h = rwy.elev + Hs[i]
        v = V[i]
        s = slope[i]
        phi = math.atan(s)
        Rn = math.sqrt(1 + s * s)
        ga = phi + math.asin(np.clip(s * wpar[i] / (v * Rn), -0.5, 0.5))
        gam[i] = ga
        vg = max(v * math.cos(ga) + wpar[i], 20.0)
        # curvature of the path in the vertical plane -> gamma rate
        if 0 < i < n - 1:
            dg = (math.atan(slope[i + 1]) - math.atan(slope[i - 1])) / (xs[i + 1] - xs[i - 1])
        else:
            dg = 0.0
        gdot = dg * vg
        rho, a = atm(h)
        rho, a = float(rho), float(a)
        q = 0.5 * rho * v * v
        M = v / a
        geard = 1.0 if (Hs[i] - 4.9 + 0.4) < 90.0 else 0.0
        gear[i] = geard
        L = m * G0 * math.cos(ga) + m * v * gdot
        CL = L / (q * model.S)
        al = _alpha_cl(model, CL, M, 0.0 if ph[i] == PH_OGS else sb_pf, geard)
        if ph[i] == PH_OGS:
            # EAS hold: required dV/dt from the EAS profile, speedbrake solves for the drag
            if i < n - 1:
                Vn = float(model.tas_from_eas(rwy.elev + Hs[i + 1], cfg.eas_ogs))
                dvdt = (Vn - v) / (dx / vg)
            else:
                dvdt = 0.0
            Dreq = -m * dvdt - m * G0 * math.sin(ga)
            CDreq = Dreq / (q * model.S)
            sbg = np.linspace(0.0, 98.6, 21)
            _, cds = model.coeffs(np.full(21, al), np.full(21, M), sbg, geard)
            sbi = float(np.interp(CDreq, cds, sbg)) if cds[-1] > cds[0] else 0.0
            sb[i] = min(max(sbi, 0.0), 98.6)
        else:
            sb[i] = sb_pf
        al = _alpha_cl(model, CL, M, sb[i], geard)
        alpha[i] = al
        _, CD = model.coeffs(np.array([al]), np.array([M]), np.array([sb[i]]), geard)
        D = q * model.S * float(CD[0])
        dVdt = -D / m - G0 * math.sin(ga)
        if i < n - 1:
            dt = dx / vg
            t[i + 1] = t[i] + dt
            V[i + 1] = max(v + dVdt * dt, 40.0)
    return xs, Hs, V, gam, alpha, sb, gear, t, ph, wcr


def _sb_preflare(model, rwy, cfg, H, phase, keys):
    """Bisection on the preflare speedbrake for the touchdown speed.  Returns (sb, status)
    with status -1 = energy short even with sb 0, +1 = too fast even with sb full."""
    lo, hi = 0.0, 98.6
    vlo = _integrate(model, rwy, cfg, H, phase, keys, lo, dx=100.0)[2][-1]
    if vlo <= cfg.v_td:
        return 0.0, -1
    vhi = _integrate(model, rwy, cfg, H, phase, keys, hi, dx=100.0)[2][-1]
    if vhi >= cfg.v_td:
        return hi, +1
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        v = _integrate(model, rwy, cfg, H, phase, keys, mid, dx=100.0)[2][-1]
        if v > cfg.v_td:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi), 0


_CACHE = {}

WIND_AIM_S = 25.0       # IGS aim / touchdown shift per m/s of along-runway wind (s)
WIND_AIM_TAIL = 650.0   # max shift toward the threshold in a tailwind (m)
WIND_AIM_HEAD = 400.0   # max shift down the runway in a headwind (m)
X_TD_MIN = 400.0        # never aim the touchdown closer to the threshold than this


def wind_aim(model, rwy: RunwayEnd, cfg: ApproachConfig) -> ApproachConfig:
    """Wind-dependent IGS aim point and touchdown target (the OGS / ALI stay unchanged).

    Tailwind: higher ground speed and a steeper air-relative IGS (bigger flare, balloon and float
    in the tailwind that weakens near the ground) carry the touchdown far down the runway -> aim
    closer to the threshold; the preflare speedbrake is then also set to bleed the speed earlier.
    Headwind: the opposite (touchdown near the threshold) -> aim further down the runway, like
    the Orbiter's wind-dependent close-in aim point."""
    wE, wN = model.wind(rwy.elev + 10.0)
    tail = float(wE * rwy.d[0] + wN * rwy.d[1])
    dx = float(np.clip(WIND_AIM_S * np.sign(tail) * max(abs(tail) - 0.5, 0.0), -WIND_AIM_HEAD, WIND_AIM_TAIL))
    if abs(dx) < 1.0:
        return cfg
    from dataclasses import replace as _replace
    return _replace(cfg, x_aim_igs=cfg.x_aim_igs - dx, x_td=max(cfg.x_td - dx, X_TD_MIN))


def _adapt_eas(model, rwy, cfg):
    """OGS EAS (280..320 KEAS, up to 340 when short of energy in a strong headwind) for which
    the preflare speedbrake setting is inside its range."""
    from dataclasses import replace as _replace
    H, phase, keys = vertical_profile(cfg)
    kt = 0.514444
    eas = cfg.eas_ogs
    sb_pf, st = _sb_preflare(model, rwy, cfg, H, phase, keys)
    for _ in range(6):
        if st < 0 or sb_pf < 4.0:
            eas2 = eas + 10.0 * kt
        elif st > 0:
            eas2 = eas - 10.0 * kt
        else:
            break
        if not (279.0 * kt <= eas2 <= (341.0 if st < 0 else 321.0) * kt):
            break
        c2 = _replace(cfg, eas_ogs=eas2)
        sb2, st2 = _sb_preflare(model, rwy, c2, H, phase, keys)
        eas, cfg, sb_pf, st = eas2, c2, sb2, st2
    return cfg, H, phase, keys, sb_pf, st


def build_reference(model, rwy: RunwayEnd, cfg: ApproachConfig, dx=25.0, x_rollout=300.0) -> ALRef:
    """A/L reference; the IGS aim point / touchdown target are shifted with the along-runway wind
    and the OGS EAS is adapted so that the preflare speedbrake setting stays inside its range
    (headwind -> faster OGS, tailwind -> slower)."""
    key = (rwy.name, tuple(sorted(cfg.__dict__.items())), round(model.w_speed, 1), round(model.w_dir, 0))
    if key in _CACHE:
        return _CACHE[key]
    cfg, H, phase, keys, sb_pf, st = _adapt_eas(model, rwy, wind_aim(model, rwy, cfg))
    xs, Hs, V, gam, alpha, sb, gear, t, ph, wcr = _integrate(model, rwy, cfg, H, phase, keys, sb_pf, dx=dx,
                                                              x_end=keys["x_td"] + x_rollout)
    E, N = rwy.from_rwy(xs, 0.0)
    crab = -np.arcsin(np.clip(wcr / np.maximum(V, 30.0), -0.3, 0.3))
    psi = rwy.heading + crab
    X = np.column_stack([E, N, rwy.elev + Hs, V, gam, psi])
    U = np.column_stack([alpha, np.zeros_like(alpha), sb])
    ref = ALRef(rwy=rwy, cfg=cfg, t=t, X=X, U=U, gear=gear, x=xs, ph=ph, keys=keys, sb_pf=sb_pf)
    if len(_CACHE) > 32:
        _CACHE.clear()
    _CACHE[key] = ref
    return ref


# ----------------------------------------------------------------------------- terminal laws
def flare_alpha(tables, mass, S, s, gear=1.0, k_flare=0.32, sink_td=0.9):
    """Exponential sink-rate flare (proven in the 6-DOF tests); s = SimState."""
    h = max(s.agl_gear, 0.0)
    hdot_cmd = -float(np.clip(k_flare * h, sink_td, 4.5))
    # inertial sink-rate loop (vertical gusts must not fool the flare)
    g_now = math.radians(s.gamma)
    n_cmd = math.cos(g_now) + 0.9 * (hdot_cmd - s.vs) / 9.81
    cl_req = n_cmd * mass * 9.81 / max(s.qbar * S, 1.0)
    theta_max = 9.7 + float(np.clip((h - 10.0) / 50.0, 0.0, 1.0)) * 5.3
    a_max = theta_max - s.gamma
    return float(np.clip(tables.alpha_for_cl(cl_req, s.mach, 0.0, gear), 0.0, a_max))


def lateral_bank(rwy: RunwayEnd, s, bank_lim_deg=8.0, k_y=0.012, k_ydot=0.08):
    """Centreline capture by bank (final flare: small bank limit)."""
    x, y = rwy.to_rwy(s.pos[0], s.pos[1])
    vy = s.vel[0] * rwy.d[1] - s.vel[1] * rwy.d[0]
    cmd = -k_y * float(y) - k_ydot * float(vy)
    return float(np.clip(math.degrees(cmd), -bank_lim_deg, bank_lim_deg))


def rollout_steer(rwy: RunwayEnd, s):
    """Nose-wheel steering toward the centreline; returns (steer_deg, along, cross)."""
    x, y = rwy.to_rwy(s.pos[0], s.pos[1])
    herr = math.degrees(wrap_pi(math.radians(s.heading) - rwy.heading))
    steer = float(np.clip(-0.08 * float(y) - 0.8 * herr, -8.0, 8.0)) if s.ground_speed > 2.0 else 0.0
    return steer, float(x), float(y)
