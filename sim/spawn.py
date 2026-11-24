"""Random initial condition "anywhere in the sky"."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .atmosphere import atmosphere
from .dynamics import R_to_quat, euler_ned_to_R


@dataclass
class InitialState:
    pos: np.ndarray       # ENU
    vel: np.ndarray       # ENU inertial
    quat: np.ndarray
    omega: np.ndarray
    alpha_deg: float
    de_deg: float
    bf_deg: float
    gear_down: bool = False
    desc: str = ""


def attitude_from_flight(heading_rad, gamma_rad, alpha_rad, beta_rad=0.0, mu_rad=0.0):
    """Body attitude for wings-level (mu=0) flight with given air-relative velocity
    direction (heading, gamma) and angle of attack (beta = 0)."""
    # wings level: theta = gamma + alpha, psi = heading, phi = 0
    R = euler_ned_to_R(heading_rad, gamma_rad + alpha_rad, 0.0)
    return R


def random_spawn(rng: np.random.Generator, tables, terrain, mass, S, wind_fn=None,
                 tries=200, feasible=None) -> InitialState:
    """Random state "anywhere in the sky".  ``feasible(x3)`` (optional) filters candidates
    with a reachability check on the 3-DOF state [E, N, h, V, gamma, psi] (energy band:
    the runway can be reached by the GNC); None keeps the unconstrained behaviour."""
    for _ in range(tries):
        h = rng.uniform(6000.0, 25000.0)
        dist = rng.uniform(15000.0, 70000.0)
        brg = rng.uniform(0.0, 2 * math.pi)
        E, N = dist * math.sin(brg), dist * math.cos(brg)
        if h - terrain.height(E, N) < 3000.0:
            continue
        T, p, rho, a, mu = atmosphere(h)
        qbar = rng.uniform(8000.0, 14000.0)
        V = math.sqrt(2.0 * qbar / rho)
        V = min(V, 2.6 * a)
        V = max(V, 120.0)
        qbar = 0.5 * rho * V * V
        M = V / a
        gam = math.radians(rng.uniform(-14.0, -4.0))
        hdg = rng.uniform(0.0, 2 * math.pi)
        cl_req = mass * 9.80665 * math.cos(gam) / (qbar * S)
        amax = min(tables.trim_alpha_max(M), 25.0)
        alpha = float(np.clip(tables.alpha_for_cl(cl_req, M), 0.0, amax))
        de, bf = tables.trim_surfaces(alpha, M)
        if feasible is not None and not feasible(np.array([E, N, h, V, gam, hdg])):
            continue
        R = attitude_from_flight(hdg, gam, math.radians(alpha))
        v_air = V * np.array([math.cos(gam) * math.sin(hdg), math.cos(gam) * math.cos(hdg), math.sin(gam)])
        wind = wind_fn(h - terrain.height(E, N)) if wind_fn else np.zeros(3)
        desc = (f"spawn: {dist/1000:.1f} km from runway (bearing {math.degrees(brg):.0f} deg), "
                f"alt {h/1000:.1f} km, Mach {M:.2f}, qbar {qbar/1000:.1f} kPa, heading "
                f"{math.degrees(hdg):.0f} deg, gamma {math.degrees(gam):.1f} deg, alpha {alpha:.1f} deg")
        return InitialState(pos=np.array([E, N, h]), vel=v_air + wind, quat=R_to_quat(R),
                            omega=np.zeros(3), alpha_deg=alpha, de_deg=de, bf_deg=bf, desc=desc)
    raise RuntimeError("could not find a valid spawn point")


KT = 1.0 / 0.514444


def custom_spawn(tables, terrain, mass, S, e, n, alt, heading_deg=None, speed=None, speed_unit="auto",
                 gamma_deg=-8.0, wind_fn=None, gear_down=None, rwy_center=(0.0, 0.0)):
    """User-chosen initial state at ENU position (e, n) and altitude ``alt`` (m MSL).

    heading_deg: air-relative heading (deg true); None = toward the runway centre.
    speed / speed_unit: "auto" (dynamic pressure ~10 kPa, like the random spawns), "mach",
    "keas" (equivalent airspeed in knots) or "tas" (m/s).  gamma_deg: flight-path angle.
    The state is clamped to something the vehicle can fly (>= 150 m above the terrain,
    trimmable angle of attack, 0 < Mach < 3); every clamp is reported in ``desc``.
    Returns (InitialState, warnings)."""
    warn = []
    e, n, alt = float(e), float(n), float(alt)
    ext = float(getattr(terrain, "extent", 0.0))
    lim = 0.5 * ext - 2000.0 if ext > 20000.0 else 100000.0     # no terrain map: flat world
    if abs(e) > lim or abs(n) > lim:
        e, n = float(np.clip(e, -lim, lim)), float(np.clip(n, -lim, lim))
        warn.append(f"position clamped to the map ({lim / 1000:.0f} km)")
    hg = float(terrain.height(e, n))
    if alt - hg < 150.0:
        alt = hg + 150.0
        warn.append(f"altitude raised to {alt:.0f} m (150 m above the terrain)")
    alt = min(alt, 40000.0)
    T, p, rho, a, mu = atmosphere(alt)
    if heading_deg is None:
        hdg = math.atan2(rwy_center[0] - e, rwy_center[1] - n)
    else:
        hdg = math.radians(float(heading_deg))
    unit = (speed_unit or "auto").lower()
    if unit == "auto" or speed is None:
        V = math.sqrt(2.0 * 10000.0 / rho)
        V = max(min(V, 2.6 * a), 120.0)
    elif unit == "mach":
        V = float(speed) * a
    elif unit == "keas":
        V = float(speed) / KT * math.sqrt(1.225 / rho)
    else:
        V = float(speed)
    Vmin, Vmax = 75.0, 3.0 * a
    if not Vmin <= V <= Vmax:
        V = float(np.clip(V, Vmin, Vmax))
        warn.append(f"speed clamped to {V:.0f} m/s")
    gam = math.radians(float(np.clip(gamma_deg, -30.0, 10.0)))
    M = V / a
    qbar = 0.5 * rho * V * V
    gear = bool(gear_down) if gear_down is not None else (alt - hg < 300.0)
    cl_req = mass * 9.80665 * math.cos(gam) / (qbar * S)
    amax = min(tables.trim_alpha_max(M), 25.0)
    alpha = float(tables.alpha_for_cl(cl_req, M, 0.0, 1.0 if gear else 0.0))
    if not np.isfinite(alpha) or alpha > amax:
        warn.append(f"too slow for 1 g at this altitude: alpha limited to {amax:.1f} deg (will sink / accelerate)")
        alpha = amax
    alpha = float(max(alpha, 0.0))
    de, bf = tables.trim_surfaces(alpha, M, 0.0, 1.0 if gear else 0.0)
    R = attitude_from_flight(hdg, gam, math.radians(alpha))
    v_air = V * np.array([math.cos(gam) * math.sin(hdg), math.cos(gam) * math.cos(hdg), math.sin(gam)])
    wind = wind_fn(alt - hg) if wind_fn else np.zeros(3)
    dist = math.hypot(e - rwy_center[0], n - rwy_center[1])
    brg = math.degrees(math.atan2(e - rwy_center[0], n - rwy_center[1])) % 360.0
    desc = (f"custom spawn: {dist/1000:.1f} km from runway (bearing {brg:.0f} deg), "
            f"alt {alt/1000:.1f} km, Mach {M:.2f}, qbar {qbar/1000:.1f} kPa, heading "
            f"{math.degrees(hdg) % 360.0:.0f} deg, gamma {math.degrees(gam):.1f} deg, alpha {alpha:.1f} deg")
    st = InitialState(pos=np.array([e, n, alt]), vel=v_air + wind, quat=R_to_quat(R), omega=np.zeros(3),
                      alpha_deg=alpha, de_deg=de, bf_deg=bf, gear_down=gear, desc=desc)
    return st, warn


def approach_state(terrain, tables, height_agl=30.0, V=105.0, gamma_deg=-2.0, offset_along=None,
                   runway_dir_deg=None, gear_down=True) -> InitialState:
    """Short-final state aligned with RWY33 (used by tests)."""
    rd = terrain.rwy_dir
    hdg = math.atan2(rd[0], rd[1]) if runway_dir_deg is None else math.radians(runway_dir_deg)
    # place the aircraft so that the glide path meets the runway ~600 m past the threshold
    gam = math.radians(gamma_deg)
    if offset_along is None:
        offset_along = -terrain.rwy_half_len + 600.0 - height_agl / math.tan(-gam) if gam < 0 else -2000.0
    E, N = terrain.rwy_c + rd * offset_along
    h = terrain.rwy_elev + height_agl + 4.9      # CG above wheels
    T, p, rho, a, mu = atmosphere(h)
    M = V / a
    qbar = 0.5 * rho * V * V
    cl_req = 99800.0 * 9.80665 * math.cos(gam) / (qbar * 249.91)
    alpha = float(np.clip(tables.alpha_for_cl(cl_req, M, 0.0, 1.0 if gear_down else 0.0), 0.0, 14.0))
    de, bf = tables.trim_surfaces(alpha, M, 0.0, 1.0 if gear_down else 0.0)
    R = attitude_from_flight(hdg, gam, math.radians(alpha))
    v = V * np.array([math.cos(gam) * math.sin(hdg), math.cos(gam) * math.cos(hdg), math.sin(gam)])
    return InitialState(pos=np.array([E, N, h]), vel=v, quat=R_to_quat(R), omega=np.zeros(3),
                        alpha_deg=alpha, de_deg=de, bf_deg=bf, gear_down=gear_down,
                        desc=f"approach: {height_agl:.0f} m AGL, {V:.0f} m/s, gamma {gamma_deg:.1f} deg")
