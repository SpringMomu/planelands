"""Approach & landing (A/L) reference and terminal laws, Orbiter style.

Vertical profile along the runway centreline (x = distance past the threshold), after the
Orbiter autoland geometry (JSC-23266 Approach, Landing and Rollout Flight Procedures Handbook,
sections 4.3-4.7, 5.3.3 and appendices B.3/B.4):
    OGS       steep outer glide slope (gamma_ogs) aimed at x_aim_ogs before the threshold,
              flown at constant EAS with the speedbrake modulating drag
    PREFLARE  constant-g pull-up circle (radius r_preflare, ~0.33 g) tangent to the OGS and to a
              line 17.5 ft above the IGS, left 35 ft above the IGS for an exponential decay onto it
    IGS       shallow inner glide slope (-1.5 deg) aimed x_aim_igs past the threshold: several
              seconds of stable flight at ~12 fps sink, threshold crossing ~20-30 ft wheel height
    FLARE     final flare: the Orbiter's linear sink-rate-versus-altitude table (13 fps at 52 ft,
              3 fps at 2 ft, constant 3 fps below), entered at 30-80 ft depending on the sink rate,
              so the vehicle keeps descending all the way to touchdown - no level-off, no float
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


FT = 0.3048
# Orbiter final flare (JSC-23266 5.3.3 m / 4.5.4): reference sink rate linear in altitude, 13 fps at
# 52 ft down to 3 fps at 2 ft, constant 3 fps below 2 ft  ->  hdot_ref = -FF_K * (h + FF_HOFF)
FF_K = 0.2                       # 1/s  ((13 - 3) fps / (52 - 2) ft)
FF_HOFF = 13.0 * FT              # m    (3 fps / 0.2 - 2 ft)
FF_HMIN = 2.0 * FT               # below this: constant sink
FF_SINK_TD = FF_K * (FF_HMIN + FF_HOFF)     # 3 fps reference sink at touchdown
FF_H_MAX = 80.0 * FT             # highest automatic final-flare entry ...
FF_H_MIN = 30.0 * FT             # ... and the forced one


def ff_hdot_ref(h_gear, sink_floor=FF_SINK_TD):
    """Final-flare reference altitude rate (m/s, < 0) at main-gear height ``h_gear``."""
    return -max(FF_K * (max(h_gear, FF_HMIN) + FF_HOFF), sink_floor)


def ff_trigger_height(hdot):
    """Main-gear height at which the final flare starts for the current altitude rate: where the
    table sink equals the current one (higher sink -> earlier flare), limited to 30..80 ft."""
    return float(np.clip(-hdot / FF_K - FF_HOFF, FF_H_MIN, FF_H_MAX))


def vertical_profile(cfg: ApproachConfig):
    """Return function H(x) (CG height above runway), phase(x) and key x positions."""
    to = math.tan(math.radians(-cfg.gamma_ogs_deg))
    ti = math.tan(math.radians(-cfg.gamma_igs_deg))
    go, gi = math.radians(cfg.gamma_ogs_deg), math.radians(cfg.gamma_igs_deg)
    d1, d2 = cfg.dh_pf_igs, cfg.dh_pf_exp
    # pull-up circle tangent to the OGS and to the IGS shifted up by d1
    x_int = (cfg.x_aim_ogs * to - cfg.x_aim_igs * ti - d1) / (to - ti)
    H_int = (cfg.x_aim_ogs - x_int) * to
    dgam = abs(go - gi)
    R = cfg.r_preflare
    dt_ = R * math.tan(dgam / 2.0)
    x_t1 = x_int - dt_ * math.cos(go)
    H_t1 = H_int + dt_ * math.sin(-go)
    x_t2 = x_int + dt_ * math.cos(gi)
    nx, nH = -math.sin(go), math.cos(go)
    xc, Hc = x_t1 + R * nx, H_t1 + R * nH

    def h_igs(x):
        return (cfg.x_aim_igs - x) * ti

    def h_arc(x):
        return Hc - np.sqrt(np.maximum(R * R - (x - xc) ** 2, 0.0))

    # leave the circle where it is d2 above the IGS (before its tangent point x_t2), exponential
    # decay onto the IGS with the circle's slope there
    lo, hi = x_t1, x_t2
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if float(h_arc(mid)) - h_igs(mid) > d2:
            lo = mid
        else:
            hi = mid
    x_e = 0.5 * (lo + hi)
    s_e = (x_e - xc) / math.sqrt(max(R * R - (x_e - xc) ** 2, 1e-9))       # dH/dx on the circle (< 0)
    lam = d2 / max(-s_e - ti, 1e-4)
    H_e = float(h_arc(x_e))

    def h_pre(x):                       # circle, then the exponential onto the IGS
        x = np.asarray(x, float)
        return np.where(x <= x_e, h_arc(x), h_igs(x) + d2 * np.exp(-(x - x_e) / lam))

    # final flare: starts where the table sink at the gear height equals the sink of the path
    # (ground speed v_ff), clamped to the 30..80 ft entry band
    hg_off = cfg.h_td_cg                # CG height above the wheels at touchdown attitude
    xs = np.arange(x_e, cfg.x_aim_igs + 2000.0, 2.0)
    Hs = h_pre(xs)
    sink = -np.gradient(Hs, xs) * cfg.v_ff
    hg = Hs - hg_off
    trig = (hg <= np.clip(sink / FF_K - FF_HOFF, FF_H_MIN, FF_H_MAX)) | (hg <= FF_H_MIN)
    k = int(np.argmax(trig)) if trig.any() else len(xs) - 1
    x_f0, H_f0 = float(xs[k]), float(Hs[k])
    hg0 = H_f0 - hg_off
    Lf = cfg.v_ff / FF_K                # e-folding distance of the flare
    x_min = x_f0 + Lf * math.log(max((hg0 + FF_HOFF) / (FF_HMIN + FF_HOFF), 1.0))
    s_min = FF_SINK_TD / cfg.v_ff       # constant-sink slope below 2 ft
    x_td = x_min + min(hg0, FF_HMIN) / s_min if hg0 > FF_HMIN else x_f0 + max(hg0, 0.0) / s_min

    def h_ff(x):
        x = np.asarray(x, float)
        exp_part = (hg0 + FF_HOFF) * np.exp(-(x - x_f0) / Lf) - FF_HOFF
        lin_part = FF_HMIN - (x - x_min) * s_min
        g = np.where(x <= x_min, exp_part, lin_part)
        return hg_off + np.maximum(g, 0.0)

    x_ig = min(x_e + lam, x_f0)        # (mostly) settled onto the IGS, or the flare starts first
    keys = dict(x_ali=cfg.x_ali, x_pf=x_t1, H_pf=H_t1, x_exp=x_e, H_exp=H_e, x_igs=x_ig,
                H_igs=float(h_pre(x_ig)), x_flare=x_f0, H_flare=H_f0, x_td=x_td, x_c=xc, H_c=Hc,
                r_pf=R, lam=lam)

    def H(x):
        x = np.asarray(x, float)
        out = np.where(x <= x_t1, (cfg.x_aim_ogs - x) * to, 0.0)
        out = np.where((x > x_t1) & (x <= x_f0), h_pre(x), out)
        out = np.where((x > x_f0) & (x <= x_td), h_ff(x), out)
        out = np.where(x > x_td, cfg.h_td_cg, out)
        return out

    def phase(x):
        x = np.asarray(x, float)
        return np.select([x <= x_t1, x <= x_ig, x <= x_f0, x <= x_td], [PH_OGS, PH_PREFLARE, PH_IGS, PH_FLARE],
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
    dwpar = np.gradient(wpar, xs)                  # along-track wind shear along the path (1/s per m)
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
        # airspeed change from the shear: a tailwind that weakens toward the ground raises the
        # airspeed during the descent (ground speed is kept by inertia), a headwind lowers it
        dVdt = -D / m - G0 * math.sin(ga) - dwpar[i] * vg * math.cos(ga)
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

WIND_AIM_S = 25.0       # IGS aim / touchdown shift per m/s of along-runway headwind (s) ...
WIND_AIM_S_TAIL = 10.0  # ... and tailwind: the final flare does not float, so only a small shift
WIND_AIM_TAIL = 250.0   # max shift toward the threshold in a tailwind (m)
WIND_AIM_HEAD = 400.0   # max shift down the runway in a headwind (m)
X_TD_MIN = 400.0        # never aim the touchdown closer to the threshold than this ...
X_AIM_MIN = 200.0       # ... i.e. the IGS aim point never closer than this


def wind_aim(model, rwy: RunwayEnd, cfg: ApproachConfig) -> ApproachConfig:
    """Wind-dependent IGS aim point and touchdown target (the OGS / ALI stay unchanged).

    Tailwind: higher ground speed and a steeper air-relative IGS carry the touchdown somewhat
    further down the runway -> aim a little closer to the threshold (a large shift would put the
    touchdown early and fast: the table flare keeps descending, it does not float off the excess);
    the preflare speedbrake is then also set to bleed the speed earlier.
    Headwind: the opposite (touchdown near the threshold) -> aim further down the runway, like
    the Orbiter's wind-dependent close-in aim point."""
    wE, wN = model.wind(rwy.elev + 10.0)
    tail = float(wE * rwy.d[0] + wN * rwy.d[1])
    k = WIND_AIM_S_TAIL if tail > 0.0 else WIND_AIM_S
    dx = float(np.clip(k * np.sign(tail) * max(abs(tail) - 0.5, 0.0), -WIND_AIM_HEAD, WIND_AIM_TAIL))
    if abs(dx) < 1.0:
        return cfg
    from dataclasses import replace as _replace
    # the touchdown point follows the IGS aim point (final-flare geometry); keep it >= X_TD_MIN
    aim = max(cfg.x_aim_igs - dx, X_AIM_MIN)
    return _replace(cfg, x_aim_igs=aim, x_td=max(cfg.x_td - (cfg.x_aim_igs - aim), X_TD_MIN))


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
GE_K, GE_L = 0.131, 11.0     # trimmed lift ratio in ground effect, fit of the 6-DOF strip model


def ground_effect_lift(h_cg):
    """Trimmed lift at a given alpha in ground effect relative to free air.  The untrimmed lift
    rises ~27 % at touchdown height (the Orbiter's L/D rises ~25 %, JSC-23266 B.4), but the
    ground-effect nose-down moment is trimmed out with up-elevon (the body flap is too slow for the
    last seconds), which costs most of it: ~+9 % at touchdown, fit 1 + 0.131 exp(-h_cg / 11 m)."""
    return 1.0 + GE_K * math.exp(-max(h_cg, 0.0) / GE_L)


def _lift_den(s):
    g_air = math.radians(s.gamma)
    phi = math.radians(s.bank)
    ld = float(np.clip(s.ld, 1.5, 8.0)) if s.ld > 0.0 else 4.0
    return max(math.cos(g_air) * math.cos(phi) - math.sin(g_air) / ld, 0.3)


def alpha_for_accel(tables, mass, S, s, az, sb=0.0, gear=1.0):
    """Angle of attack for an inertial vertical acceleration ``az`` (m/s^2, up) at the current
    flight condition: lift from the vertical force balance with the current L/D for the drag; the
    trim tables are free-air, so the ground-effect lift increase is divided out."""
    lift = mass * (G0 + az) / _lift_den(s)
    cl_req = lift / max(s.qbar * S, 1.0) / ground_effect_lift(s.agl)
    return float(tables.alpha_for_cl(cl_req, s.mach, sb, gear))


def profile_derivs(H, x, d=4.0):
    """Height, slope and curvature of the A/L profile at x."""
    h0, hp, hm = float(H(x)), float(H(x + d)), float(H(x - d))
    return h0, (hp - hm) / (2.0 * d), (hp - 2.0 * h0 + hm) / (d * d)


def al_vertical_accel(H, x, vx, h_cg, vs, a_int=0.0, h_gear=None, kh=0.2, kd=0.8, eh_lim=90.0):
    """Orbiter-style autoland pitch law ahead of the final flare: track the altitude for the current
    range (OGS, pull-up circle, exponential, IGS) with altitude and altitude-rate errors plus the
    path-curvature feed-forward; ``a_int`` is the caller's integral term (model-error trim).
    Sink-rate protection: never descend faster than the final-flare table (extended above 80 ft)
    allows at the current gear height - a vehicle high on short final must not dive into a sink the
    flare cannot arrest (it lands long instead).  Returns (vertical acceleration m/s^2, altitude
    error m)."""
    h_ref, dH, ddH = profile_derivs(H, x)
    hdot_ref = dH * vx
    eh = float(np.clip(h_ref - h_cg, -eh_lim, eh_lim))
    acc = ddH * vx * vx + kh * eh + kd * (hdot_ref - vs) + a_int
    if h_gear is not None:
        hd_min = -FF_K * (max(h_gear, 0.0) + FF_HOFF)
        acc = max(acc, kd * (hd_min - vs) - FF_K * vs)
    return float(np.clip(acc, -0.6 * G0, 1.2 * G0)), eh


def tail_scrape_alpha_max(s, h_gear):
    """Pitch limit close to the ground (tail-scrape protection) expressed as an alpha limit."""
    theta_max = 9.7 + float(np.clip((h_gear - 10.0) / 50.0, 0.0, 1.0)) * 5.3
    return theta_max - s.gamma


def flare_alpha(tables, mass, S, s, sb=0.0, gear=1.0, sink_floor=FF_SINK_TD, kd=0.8):
    """Orbiter final flare: Nz from the error to the sink-rate-versus-altitude table (inertial sink
    rate, so vertical gusts do not fool it), with the table's own rate of change as feed-forward.
    The reference sink never goes to zero, so the vehicle keeps settling onto the runway instead of
    levelling off and floating; nose-down authority is small (no pushing the wheels into the runway,
    whose first response is a sink increase anyway)."""
    h = max(s.agl_gear, 0.0)
    hd_ref = ff_hdot_ref(h, sink_floor)
    ff = -FF_K * s.vs if (h > FF_HMIN and -hd_ref > sink_floor + 1e-6) else 0.0
    az = float(np.clip(ff + kd * (hd_ref - s.vs), -0.15 * G0, 0.6 * G0))
    a = alpha_for_accel(tables, mass, S, s, az, sb, gear)
    return float(np.clip(a, 0.0, tail_scrape_alpha_max(s, h)))


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
