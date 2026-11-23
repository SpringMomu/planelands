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
