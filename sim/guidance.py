"""Guidance interface (outer loop).  SCP + MPC guidance plugs in here in a later phase.

A guidance law receives a :class:`SimState` snapshot (10 Hz by default) and returns a
:class:`GuidanceCommand` that the fly-by-wire FCS tracks (alpha, wind-axis bank,
speedbrake, ...).  Register new laws with ``@register`` and select them by name:

    @register
    class ScpMpcGuidance(GuidanceBase):
        name = "scp_mpc"
        def update(self, t, s):
            ...  # use self.ctx.model (ReducedModel) / self.ctx.tables (TrimTables)
            return GuidanceCommand(alpha_deg=..., bank_deg=..., speedbrake_deg=...)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

D2R = math.pi / 180.0


@dataclass
class GuidanceCommand:
    alpha_deg: float
    bank_deg: float = 0.0
    speedbrake_deg: float = 0.0
    gear_down: bool = False
    chute: bool = False
    brakes: float = 0.0
    steer_deg: float = 0.0           # nose-wheel steering (rollout)
    beta_deg: float = 0.0
    pitch_rate_dps: float | None = None   # derotation override on the main gear
    info: str = ""


@dataclass
class SimState:
    t: float
    pos: np.ndarray        # ENU [m]
    vel: np.ndarray        # ENU inertial [m/s]
    quat: np.ndarray       # body FRD -> ENU
    omega: np.ndarray      # body rates [rad/s]
    alpha: float           # [deg] (all angles in degrees in this snapshot)
    beta: float
    bank: float            # wind-axis bank mu
    gamma: float           # air-relative flight-path angle
    gamma_i: float         # inertial flight-path angle
    track: float
    heading: float         # body heading psi
    phi: float
    theta: float
    mach: float
    tas: float
    eas: float
    qbar: float
    rho: float
    alt: float
    agl: float             # CG height above ground
    agl_gear: float        # main-gear wheel height above ground (gear extended)
    vs: float
    status: str
    wow: list
    gear_pos: float
    ground_speed: float
    rwy_along: float
    rwy_cross: float
    rwy_heading: float     # deg, RWY33 direction
    mass: float
    ld: float
    wind: np.ndarray


@dataclass
class GuidanceContext:
    tables: object          # sim.reduced_model.TrimTables
    model: object           # sim.reduced_model.ReducedModel
    world: dict
    geometry: dict
    mass: float
    S: float


REGISTRY: dict = {}


def register(cls):
    REGISTRY[cls.name] = cls
    return cls


class GuidanceBase:
    name = "base"
    rate_hz = 10.0

    def __init__(self, ctx: GuidanceContext):
        self.ctx = ctx

    def reset(self, s: SimState):
        pass

    def update(self, t: float, s: SimState) -> GuidanceCommand:
        raise NotImplementedError

    def telemetry(self) -> dict:
        """Extra data for the frontend (e.g. planned / predicted trajectories)."""
        return {}


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


@register
class HoldGlideGuidance(GuidanceBase):
    """Placeholder until SCP+MPC exists: wings level at best trimmed L/D, gear down below
    90 m, an exponential sink-rate flare, then a rollout law (derotation by the FCS,
    drag chute, brakes, nose-wheel steering onto the runway centreline if on the runway).
    It does NOT steer toward the airfield."""
    name = "hold_glide"
    k_flare = 0.32
    sink_td = 0.9

    def reset(self, s):
        self.gear = s.gear_pos > 0.5
        self.t_nose = None

    def update(self, t, s):
        tab = self.ctx.tables
        gear = 1.0 if (self.gear or s.gear_pos > 0.5) else 0.0
        if s.status in ("ROLLOUT", "STOPPED"):
            return self._rollout(t, s)
        if s.agl_gear < 90.0:
            self.gear = True
        alpha = tab.best_ld_alpha(s.mach, 0.0, gear)
        info = "glide best L/D"
        if s.agl_gear < 500.0:
            h = max(s.agl_gear, 0.0)
            # shallow glide, then exponential final flare (tau ~3 s) to ~0.9 m/s sink
            hdot_cmd = -float(np.clip(self.k_flare * h, self.sink_td, 0.12 * max(h, 1.0) if h > 40.0 else 4.5))
            V = max(s.tas, 30.0)
            g_cmd = math.asin(max(min(hdot_cmd / V, 0.5), -0.5))
            g_now = math.radians(s.gamma)
            n_cmd = math.cos(g_now) + V * 0.8 * (g_cmd - g_now) / 9.81
            cl_req = n_cmd * self.ctx.mass * 9.81 / max(s.qbar * self.ctx.S, 1.0)
            # tail-scrape protection: limit body pitch (~10 deg at the wheels), relaxed higher up
            theta_max = 9.7 + float(np.clip((h - 10.0) / 50.0, 0.0, 1.0)) * 5.3
            a_max = theta_max - s.gamma
            a_flare = tab.alpha_for_cl(cl_req, s.mach, 0.0, gear)
            if g_cmd > g_now - 0.02 or h < 250:
                alpha = float(np.clip(a_flare, 0.0, a_max))
            info = "flare"
        return GuidanceCommand(alpha_deg=alpha, bank_deg=0.0, speedbrake_deg=0.0,
                               gear_down=self.gear, info=info)

    def _rollout(self, t, s):
        nose = bool(s.wow[0])
        if nose and self.t_nose is None:
            self.t_nose = t
        gs = s.ground_speed
        # steer toward the runway centreline (direction of travel picks 33 or 15)
        rh = s.rwy_heading
        d33 = abs(_wrap180(s.track - rh))
        if d33 <= 90.0:
            hdg_ref, cross = rh, s.rwy_cross
        else:
            hdg_ref, cross = rh + 180.0, -s.rwy_cross
        on_rwy = abs(s.rwy_along) < 2286.0 + 305.0 and abs(s.rwy_cross) < 60.0
        herr = _wrap180(s.heading - hdg_ref) if on_rwy else 0.0
        cross = cross if on_rwy else 0.0
        steer = float(np.clip(-0.08 * cross - 0.8 * herr, -8.0, 8.0)) if gs > 2.0 else 0.0
        brakes = 0.0
        if nose and gs < 85.0:
            brakes = 0.55
        if gs < 15.0:
            brakes = 1.0
        chute = nose and (self.t_nose is not None and t - self.t_nose > 0.5) and gs > 32.0
        return GuidanceCommand(alpha_deg=0.0, bank_deg=0.0, speedbrake_deg=98.6 if nose else 0.0,
                               gear_down=True, chute=chute, brakes=brakes, steer_deg=steer,
                               info="rollout")
