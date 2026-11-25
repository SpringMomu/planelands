"""Simulation orchestrator: plant (6-DOF + aero + gear) + FCS + guidance + telemetry."""
from __future__ import annotations

import math
import time

import numpy as np

from .aero import AeroModel
from .atmosphere import RHO0, atmosphere, gravity
from .dynamics import Context, RigidBody, R_to_euler_ned, quat_to_R
from .fcs import Actuators, FcsInput, FlightControl, wrap
from .gear import LandingGear
from .geometry import CH_BF, CH_LI, CH_LO, CH_RI, CH_RO, CH_RUD, CH_SB, CTRL_NAMES, build_geometry
from .guidance import REGISTRY, GuidanceCommand, GuidanceContext, SimState
from .gnc import guidance_scpmpc as _gnc_register  # noqa: F401  (registers "scp_mpc")
from .gnc.geometry import ApproachConfig, runway_ends, select_plan
from .gnc.model import PlanningModel
from .reduced_model import ReducedModel, TrimTables
from .spawn import InitialState, approach_state, custom_spawn, random_spawn
from .terrain import Terrain
from .trim import load_or_build_trim_tables
from .wind import WindModel

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
DT = 0.01            # physics macro step
SUBSTEPS = 4         # near the ground
FCS_DT = 0.02
KT = 1.0 / 0.514444


class Simulator:
    def __init__(self, seed=None, guidance="scp_mpc", verbose=True, planner="sync", spawn="feasible"):
        """planner: 'sync' (headless, deterministic) or 'async' (worker process, real time).
        spawn: 'feasible' (energy band reachable by the GNC) or 'any' (unconstrained)."""
        self.verbose = verbose
        self.spawn_mode = spawn
        self.geo = build_geometry()
        self.aero = AeroModel(self.geo)
        self.terrain = Terrain(self.geo.world, verbose=verbose)
        self.tables = TrimTables(load_or_build_trim_tables(self.aero, verbose=verbose))
        self.model = ReducedModel(self.tables, self.geo.mass, self.geo.S)
        self.gear = LandingGear(self.geo)
        self.body = RigidBody(self.geo, self.aero, self.gear, self.terrain)
        self.fcs = FlightControl(self.geo, self.body)
        self.act = Actuators(self.geo)
        self.ctx = Context()
        self.wind = WindModel(seed)
        self.gctx = GuidanceContext(tables=self.tables, model=self.model, world=self.geo.world,
                                    geometry=self.geo.raw, mass=self.geo.mass, S=self.geo.S)
        self.gctx.options = {"planner": planner}
        self.gctx.terrain = self.terrain
        self.gctx.wind_info = lambda: (self.wind.speed, self.wind.dir_from)
        self._spawn_pm = PlanningModel(self.tables, self.geo.mass, self.geo.S,
                                       ground_elev=self.geo.world["runway"]["elevation"])
        self._spawn_ends = runway_ends(self.geo.world)
        self.guidance_name = guidance
        self.guidance = REGISTRY[guidance](self.gctx)
        self.rng = np.random.default_rng(seed)
        self.mode = "AUTO"
        self.manual = dict(pitch=0.0, roll=0.0, yaw=0.0, speedbrake=0.0, gear=False, brakes=0.0, chute=False)
        self.events = []
        self.paused = False
        self.time_scale = 1.0
        self.reset(seed=seed)

    # ------------------------------------------------------------------ events
    def event(self, level, text):
        self.events.append({"type": "event", "level": level, "text": text})
        if self.verbose:
            print(f"[sim] {level}: {text}", flush=True)

    def drain_events(self):
        ev, self.events = self.events, []
        return ev

    def drain_messages(self):
        """Events + guidance messages (plan / mpc / guidance events) for the clients."""
        out = []
        pop = getattr(self.guidance, "pop_messages", None)
        if pop is not None:
            for m in pop():
                if m.get("type") == "event" and self.verbose:
                    print(f"[gnc] {m['level']}: {m['text']}", flush=True)
                out.append(m)
        return self.drain_events() + out

    def latest_plan_message(self):
        f = getattr(self.guidance, "latest_plan_message", None)
        return f() if f else None

    def _spawn_feasible(self, x3):
        pm = self._spawn_pm
        pm.set_wind(self.wind.speed, self.wind.dir_from)
        # the trimmed alpha must be flyable (below the control-margin ceiling)
        try:
            from .atmosphere import atmosphere as _atm
            T_, p_, rho_, a_, mu_ = _atm(x3[2])
            M = x3[3] / a_
            q = 0.5 * rho_ * x3[3] ** 2
            cl = self.geo.mass * 9.80665 * math.cos(x3[4]) / (q * self.geo.S)
            if self.tables.alpha_for_cl(cl, M) > float(pm.alpha_max(M)) + 0.5:
                return False
            cand, ok, note = select_plan(pm, x3, self._spawn_ends, ApproachConfig())
        except Exception:
            return False
        return bool(ok) and cand.level == 0 and not cand.relaxed and cand.band_pos < 0.9

    # ------------------------------------------------------------------ reset
    def reset(self, seed=None, init: InitialState | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.episode = getattr(self, "episode", -1) + 1
        if init is None:
            init = random_spawn(self.rng, self.tables, self.terrain, self.geo.mass, self.geo.S,
                                wind_fn=self.wind.steady,
                                feasible=self._spawn_feasible if self.spawn_mode == "feasible" else None)
        self.x = np.concatenate([init.pos, init.vel, init.quat, init.omega]).astype(float)
        self.t = 0.0
        de, bf = math.radians(init.de_deg), math.radians(init.bf_deg)
        self.act.reset([de, de, de, de, bf, 0.0, 0.0])
        self.fcs.reset()
        self.fcs.cmd = self.act.pos.copy()
        self.gear.reset(down=init.gear_down)
        self.wind.reset_turbulence()
        self.status = "FLIGHT"
        self.chute_cmd = False
        self.chute_state = 0.0      # inflated area fraction
        self.chute_t = None
        self.chute_jettisoned = False
        self._air_hold = None
        self.brakes = 0.0
        self.steer = 0.0
        self.touchdown = None
        self.stop_info = None
        self.tail_scrape_reported = False
        self.nose_td_reported = False
        self._fcs_acc = 0.0
        self._gd_acc = 1e9
        self.gcmd = GuidanceCommand(alpha_deg=init.alpha_deg, gear_down=init.gear_down)
        self.man_alpha = init.alpha_deg * D2R
        self.man_mu = 0.0
        self.last_Fb = np.zeros(3)
        self.last_Mg = np.zeros(3)
        self.last_aero = None
        self.last_detail = None
        self.last_hinge = {k: 0.0 for k in CTRL_NAMES}
        self.obs = self.observe()
        self.guidance.reset(self.snapshot())
        self.event("info", init.desc)

    def reset_to(self, init: InitialState):
        self.reset(init=init)

    # ------------------------------------------------------------------ user-placed start
    def _custom_init(self, sp: dict):
        """InitialState from a client spawn spec: e, n (m) or dist_km + bearing_deg (from the runway
        centre), alt (m MSL) or alt_km, heading_deg (None/'runway' = toward the runway), speed +
        speed_unit (auto | mach | keas | tas), gamma_deg, gear_down."""
        if "e" in sp and "n" in sp:
            e, n = float(sp["e"]), float(sp["n"])
        else:
            d = float(sp.get("dist_km", 40.0)) * 1000.0
            b = math.radians(float(sp.get("bearing_deg", 0.0)))
            e, n = d * math.sin(b), d * math.cos(b)
        alt = float(sp["alt"]) if "alt" in sp else float(sp.get("alt_km", 15.0)) * 1000.0
        hd = sp.get("heading_deg")
        hd = None if hd is None or hd == "runway" else float(hd)
        rc = self.geo.world["runway"]["center_en"]
        return custom_spawn(self.tables, self.terrain, self.geo.mass, self.geo.S, e, n, alt, heading_deg=hd,
                            speed=sp.get("speed"), speed_unit=sp.get("speed_unit", "auto"),
                            gamma_deg=float(sp.get("gamma_deg", -8.0)), wind_fn=self.wind.steady,
                            gear_down=sp.get("gear_down"), rwy_center=(float(rc[0]), float(rc[1])))

    def assess_spawn(self, init: InitialState) -> dict:
        """Energy-band reachability of a start state (same estimate the guidance plans with)."""
        agl = init.pos[2] - self.terrain.height(init.pos[0], init.pos[1])
        v_air = init.vel - self.wind.steady(agl)
        V = float(np.linalg.norm(v_air))
        x3 = np.array([init.pos[0], init.pos[1], init.pos[2], V,
                       math.asin(max(-1.0, min(1.0, v_air[2] / max(V, 1e-3)))), math.atan2(v_air[0], v_air[1])])
        pm = self._spawn_pm
        pm.set_wind(self.wind.speed, self.wind.dir_from)
        try:
            cand, ok, note = select_plan(pm, x3, self._spawn_ends, ApproachConfig())
        except Exception as ex:
            return {"reachable": False, "note": f"assessment failed: {ex!r}"}
        out = {"reachable": bool(ok), "rwy": cand.rwy.name, "side": int(cand.side), "loops": int(cand.path.loops),
               "band": float(cand.band_pos), "level": int(cand.level), "note": note,
               "e_avail": float(cand.dE_avail), "e_min": float(cand.dE_min), "e_max": float(cand.dE_max),
               "relaxed": bool(cand.relaxed), "hac_r": float(cand.path.r2), "head": float(cand.head)}
        if ok:
            out["summary"] = (f"reachable: RWY{cand.rwy.name} {'right' if cand.side > 0 else 'left'} HAC, "
                              f"energy band {cand.band_pos:.2f}" + (f" ({note})" if note else ""))
        else:
            out["summary"] = note or "not reachable"
        return out

    # ------------------------------------------------------------------ observation
    def observe(self, F_aero=None):
        x = self.x
        R = quat_to_R(x[6:10])
        pos, vel, w = x[0:3], x[3:6], x[10:13]
        T, p, rho, a, mu = atmosphere(pos[2])
        hg = self.terrain.height(pos[0], pos[1])
        agl = pos[2] - hg
        self.ctx.wind_w = self.wind.steady(agl)
        self.ctx.wind_w[2] += self.wind.vertical(agl)
        wind = self.ctx.wind_w + R @ self.wind.gust_body
        v_air_w = vel - wind
        v_air = R.T @ v_air_w
        V = float(np.linalg.norm(v_air))
        Vs = max(V, 1e-3)
        alpha = math.atan2(v_air[2], v_air[0])
        beta = math.asin(max(-1.0, min(1.0, v_air[1] / Vs)))
        # wind-axis bank
        xw = v_air_w / Vs
        ca, sa = math.cos(alpha), math.sin(alpha)
        zw = R @ np.array([-sa, 0.0, ca])
        d = np.array([0.0, 0.0, -1.0])
        zr = d - (d @ xw) * xw
        nzr = np.linalg.norm(zr)
        if nzr > 1e-6:
            zr /= nzr
            yr = np.cross(zr, xw)
            mu_b = math.atan2(-(zw @ yr), zw @ zr)
        else:
            mu_b = 0.0
        gamma_air = math.asin(max(-1.0, min(1.0, v_air_w[2] / Vs)))
        gs = math.hypot(vel[0], vel[1])
        gamma_i = math.atan2(vel[2], max(gs, 1e-6))
        track = math.degrees(math.atan2(vel[0], vel[1])) % 360.0
        psi, theta, phi = R_to_euler_ned(R)
        qbar = 0.5 * rho * V * V
        rm = self.geo.gear_pts["main_l"]
        pm = pos + R @ rm
        pm2 = pos + R @ self.geo.gear_pts["main_r"]
        agl_gear = min(pm[2] - self.terrain.height(pm[0], pm[1]), pm2[2] - self.terrain.height(pm2[0], pm2[1]))
        along, cross = self.terrain.runway_frame(pos[0], pos[1])
        o = dict(R=R, rho=rho, a=a, mu_visc=mu, T=T, agl=agl, wind=wind, v_air=v_air, V=V, alpha=alpha,
                 beta=beta, mu=mu_b, gamma=gamma_air, gamma_i=gamma_i, track=track, psi=psi, theta=theta,
                 phi=phi, qbar=qbar, mach=V / a, gs=gs, agl_gear=agl_gear, along=along, cross=cross,
                 eas=V * math.sqrt(rho / RHO0), g=gravity(pos[2]))
        return o

    def snapshot(self) -> SimState:
        o = self.obs
        x = self.x
        ld = 0.0
        if self.last_aero is not None:
            ld = self.last_aero.get("ld", 0.0)
        return SimState(
            t=self.t, pos=x[0:3].copy(), vel=x[3:6].copy(), quat=x[6:10].copy(), omega=x[10:13].copy(),
            alpha=o["alpha"] * R2D, beta=o["beta"] * R2D, bank=o["mu"] * R2D, gamma=o["gamma"] * R2D,
            gamma_i=o["gamma_i"] * R2D, track=o["track"], heading=(o["psi"] * R2D) % 360.0,
            phi=o["phi"] * R2D, theta=o["theta"] * R2D, mach=o["mach"], tas=o["V"], eas=o["eas"],
            qbar=o["qbar"], rho=o["rho"], alt=x[2], agl=o["agl"], agl_gear=o["agl_gear"], vs=x[5],
            status=self.status, wow=list(self.gear.wow), gear_pos=self.gear.pos, ground_speed=o["gs"],
            rwy_along=o["along"], rwy_cross=o["cross"], rwy_heading=self.geo.world["runway"]["heading_deg"],
            mass=self.geo.mass, ld=ld, wind=o["wind"].copy())

    # ------------------------------------------------------------------ inputs
    def set_mode(self, mode):
        mode = "MANUAL" if str(mode).upper() == "MANUAL" else "AUTO"
        if mode != self.mode:
            self.mode = mode
            o = self.obs
            self.man_alpha = o["alpha"]
            self.man_mu = o["mu"]
            if mode == "MANUAL":
                self.manual["gear"] = self.gear.cmd_down
                self.manual["speedbrake"] = self.act.pos[CH_SB] / self.geo.limits["speedbrake"][1]
            self.event("info", f"mode -> {mode}")

    def set_control(self, msg):
        m = self.manual
        for k in ("pitch", "roll", "yaw"):
            if k in msg and msg[k] is not None:
                m[k] = float(np.clip(float(msg[k]), -1.0, 1.0))
        if msg.get("speedbrake") is not None:
            m["speedbrake"] = float(np.clip(float(msg["speedbrake"]), 0.0, 1.0))
        if "gear" in msg and msg["gear"] is not None:
            m["gear"] = bool(msg["gear"])
        if "brakes" in msg and msg["brakes"] is not None:
            m["brakes"] = float(np.clip(float(msg["brakes"]), 0.0, 1.0))
        if "chute" in msg and msg["chute"] is not None:
            m["chute"] = bool(msg["chute"])

    # ------------------------------------------------------------------ FCS / guidance
    def _eval_batch(self, ctrls):
        o = self.obs
        r = self.aero.evaluate(o["v_air"], self.x[10:13], o["rho"], o["a"], o["mu_visc"], o["agl"],
                               o["R"][2, :], ctrls, gear_pos=self.gear.pos, chute=self.chute_state)
        return r.F, r.M

    def _commands(self, dt):
        """Assemble FCS commands from guidance (AUTO) or the stick (MANUAL)."""
        o = self.obs
        on_ground = any(self.gear.wow)
        if self.mode == "AUTO":
            g = self.gcmd
            cmd = dict(alpha=g.alpha_deg * D2R, bank=g.bank_deg * D2R, beta=g.beta_deg * D2R,
                       sb=g.speedbrake_deg * D2R, steer=g.steer_deg * D2R,
                       pitch_rate=None if g.pitch_rate_dps is None else g.pitch_rate_dps * D2R)
            self.gear.cmd_down = bool(g.gear_down) or self.gear.cmd_down and on_ground
            self.brakes = g.brakes
            self.chute_cmd = g.chute
        else:
            m = self.manual
            self.man_alpha = float(np.clip(self.man_alpha + m["pitch"] * 5.0 * D2R * dt, -2.0 * D2R, 28.0 * D2R))
            self.man_mu = float(np.clip(wrap(self.man_mu + m["roll"] * 15.0 * D2R * dt), -80 * D2R, 80 * D2R))
            cmd = dict(alpha=self.man_alpha, bank=self.man_mu, beta=m["yaw"] * 4.0 * D2R,
                       sb=m["speedbrake"] * self.geo.limits["speedbrake"][1],
                       steer=m["yaw"] * 10.0 * D2R,
                       pitch_rate=(m["pitch"] * 3.0 - 1.0) * D2R if abs(m["pitch"]) > 0.05 else None)
            if on_ground:
                cmd["beta"] = 0.0
                self.man_alpha = o["alpha"]
                self.man_mu = 0.0
            self.gear.cmd_down = m["gear"] or (self.gear.cmd_down and on_ground)
            self.brakes = m["brakes"]
            self.chute_cmd = m["chute"]
        return cmd

    def _fcs_step(self):
        o = self.obs
        cmd = self._commands(FCS_DT)
        # envelope protection: alpha command limited to the control-margin ceiling (trimmable
        # with roll / yaw authority left), for every guidance law and for manual flight
        if not any(self.gear.wow):
            a_lim = math.radians(float(self._spawn_pm.alpha_max(o["mach"])) + 1.5)
            cmd["alpha"] = min(cmd["alpha"], a_lim)
        self.steer = float(np.clip(cmd.get("steer", 0.0), -10 * D2R, 10 * D2R))
        if o["qbar"] >= 150.0:
            Fa, M0 = self.fcs.precompute(self._eval_batch, self.act.pos.copy())
            Mo = M0[None, :]
        else:
            F, Mo = self._eval_batch(self.act.pos[None, :])
            Fa = F[0]
        ca, sa = math.cos(o["alpha"]), math.sin(o["alpha"])
        cb, sb = math.cos(o["beta"]), math.sin(o["beta"])
        lift = Fa[0] * sa - Fa[2] * ca
        side = -Fa[0] * ca * sb + Fa[1] * cb - Fa[2] * sa * sb
        drag = -(Fa[0] * ca * cb + Fa[1] * sb + Fa[2] * sa * cb)
        self.last_aero = dict(lift=lift, side=side, drag=drag, F=Fa, M=Mo[0], ld=lift / drag if abs(drag) > 1 else 0.0)
        s = FcsInput(alpha=o["alpha"], beta=o["beta"], mu=o["mu"], gamma=o["gamma"], V=o["V"],
                     qbar=o["qbar"], mach=o["mach"], omega=self.x[10:13].copy(), phi=o["phi"],
                     theta=o["theta"], lift=lift, side=side, g=o["g"], u_act=self.act.pos.copy(),
                     M_now=Mo[0], M_gear=self.last_Mg, wow_main=self.gear.wow[1] or self.gear.wow[2],
                     wow_nose=self.gear.wow[0], gear_pos=self.gear.pos)
        self.act.cmd = self.fcs.update(FCS_DT, s, cmd, self._eval_batch)

    # ------------------------------------------------------------------ chute
    def _chute(self, dt):
        o = self.obs
        if self.chute_jettisoned:
            self.chute_state = 0.0
            return
        can = self.touchdown is not None and self.status == "ROLLOUT"
        if self.chute_cmd and can and self.chute_t is None and o["gs"] > 30.0:
            self.chute_t = self.t
            self.event("info", f"drag chute deploy at {o['gs']*KT:.0f} KGS")
        if self.chute_t is not None:
            te = self.t - self.chute_t
            if te < 0.6:
                target = 0.02                       # pilot chute
            elif te < 4.0:
                target = 0.40                       # reefed
            else:
                target = 1.0                        # disreefed
            self.chute_state += (target - self.chute_state) * min(1.0, dt / 0.35)
            if o["gs"] < 30.0 or (self.mode == "MANUAL" and not self.chute_cmd and te > 1.0):
                self.chute_jettisoned = True
                self.chute_state = 0.0
                self.event("info", f"drag chute jettison at {o['gs']*KT:.0f} KGS")

    # ------------------------------------------------------------------ physics step
    def step(self):
        """Advance one macro step DT."""
        if self.status == "CRASHED":
            return
        dt = DT
        # guidance (AUTO)
        self._gd_acc += dt
        if self.mode == "AUTO" and self._gd_acc >= 1.0 / self.guidance.rate_hz - 1e-9:
            self._gd_acc = 0.0
            try:
                self.gcmd = self.guidance.update(self.t, self.snapshot())
            except Exception as e:  # guidance must never crash the plant
                self.event("error", f"guidance error: {e!r}")
        self._fcs_acc += dt
        if self._fcs_acc >= FCS_DT - 1e-9:
            self._fcs_acc = 0.0
            self._fcs_step()
        self.wind.step(dt, self.obs["V"], self.obs["agl"])
        self.ctx.gust_b = self.wind.gust_body
        self.gear.animate(dt)
        self._chute(dt)
        self.ctx.gear_pos = self.gear.pos
        self.ctx.chute = self.chute_state
        self.ctx.brakes = self.brakes
        self.ctx.steer = self.steer
        R = self.obs["R"]
        near = self.gear.any_near_ground(self.x[0:3], R, self.terrain.height, margin=6.0) or any(self.gear.wow)
        if near:
            h = dt / SUBSTEPS
            for _ in range(SUBSTEPS):
                self.ctx.ctrl = self.act.step(h).copy()
                res, _, _, _, _, _ = self.body.aero_eval(self.x, self.ctx)
                self.ctx.frozen_aero = (res.F[0], res.M[0])
                self.ctx.gear_active = True
                self.x, Fb, Mb = self.body.rk4(self.x, h, self.ctx)
                self.ctx.frozen_aero = None
        else:
            self.ctx.ctrl = self.act.step(dt).copy()
            self.ctx.gear_active = False
            self.gear.wow = [False, False, False]
            self.gear.compress = [0.0, 0.0, 0.0]
            self.gear.struct_contact[:] = False
            self.x, Fb, Mb = self.body.rk4(self.x, dt, self.ctx)
            self.ctx.gear_active = True
        self.last_Fb = Fb
        if near:
            _, self.last_Mg = self.gear.forces(self.x[0:3], self.x[3:6], quat_to_R(self.x[6:10]),
                                               self.x[10:13], self.terrain.height, self.brakes, self.steer)
        else:
            self.last_Mg = np.zeros(3)
        if self.status == "STOPPED" and self.brakes >= 0.5:
            # parked on the brakes: no horizontal creep / yaw drift (the gear springs still settle)
            self.x[3:5] = 0.0
            self.x[12] = 0.0
        self.t += dt
        if not np.all(np.isfinite(self.x)):
            self.event("warn", "state became non-finite - resetting")
            self.reset()
            return
        self.obs = self.observe()
        self._status_logic()

    def _status_logic(self):
        o = self.obs
        wow = self.gear.wow
        vz = self.x[5]
        if self.status == "FLIGHT":
            if wow[1] or wow[2]:
                end, d_thr, cross = self._landing_frame(o)
                on_rwy = self.terrain.on_runway(self.x[0], self.x[1])
                self.touchdown = dict(t=self.t, sink=-vz, gs=o["gs"], along=o["along"], cross=o["cross"],
                                      rwy=end, past_thr=d_thr, cross_rwy=cross, on_runway=bool(on_rwy),
                                      pitch=o["theta"] * R2D, tas=o["V"])
                txt = (f"main gear touchdown: sink {-vz:.2f} m/s ({-vz/0.3048:.1f} ft/s), "
                       f"{o['gs']*KT:.0f} KGS, pitch {o['theta']*R2D:.1f} deg, "
                       + (f"{d_thr:.0f} m past RWY{end} threshold, {cross:+.1f} m off centreline"
                          if on_rwy else "OFF RUNWAY"))
                if -vz > 6.0:
                    self._crash(f"gear collapse: sink rate {-vz:.1f} m/s")
                    return
                self.event("warn" if (-vz > 3.0 or not on_rwy) else "info", txt)
                self.status = "ROLLOUT"
            elif wow[0]:
                self._crash("nose gear first contact")
                return
        if self.gear.struct_contact.any():
            names = [n for n, c in zip(self.gear.struct_names, self.gear.struct_contact) if c]
            if self.gear.struct_sink > 3.0 or not self.gear.locked or any(n in names for n in ("wingtip_l", "wingtip_r", "nose")):
                self._crash("ground impact: " + ", ".join(names) + f" (sink {self.gear.struct_sink:.1f} m/s)")
                return
            if not self.tail_scrape_reported:
                self.tail_scrape_reported = True
                self.event("warn", "tail scrape: " + ", ".join(names))
        if self.status == "ROLLOUT":
            if wow[0] and not self.nose_td_reported:
                self.nose_td_reported = True
                self.event("info", f"nose gear touchdown at {o['gs']*KT:.0f} KGS")
            if o["gs"] < 0.3 and all(wow):
                self.status = "STOPPED"
                end, d, cross = self._landing_frame(o, self.touchdown.get("rwy") if self.touchdown else None)
                self.stop_info = dict(rwy=end, past_thr=d, cross_rwy=cross,
                                      on_runway=bool(self.terrain.on_runway(self.x[0], self.x[1])))
                self.event("info", f"wheels stop: {d:.0f} m from RWY{end} threshold, {cross:+.1f} m off centreline")

    def _landing_frame(self, o, end=None):
        """Runway end in use (by ground track unless given), distance past its threshold and
        cross-track offset (positive right of the landing direction)."""
        h33 = self.geo.world["runway"]["heading_deg"]
        if end is None:
            d = abs((o["track"] - h33 + 180.0) % 360.0 - 180.0)
            end = "33" if d <= 90.0 else "15"
        L2 = self.terrain.rwy_half_len
        if end == "33":
            return end, o["along"] + L2, o["cross"]
        return end, L2 - o["along"], -o["cross"]

    def _crash(self, why):
        self.status = "CRASHED"
        self.x[3:6] = 0.0
        self.x[10:13] = 0.0
        self.event("error", "CRASH - " + why)

    def advance(self, sim_dt, max_wall=0.25):
        """Advance by sim_dt seconds (fixed DT steps, remainder carried)."""
        self._acc = getattr(self, "_acc", 0.0) + sim_dt
        t0 = time.perf_counter()
        n = 0
        while self._acc >= DT - 1e-12:
            self.step()
            self._acc -= DT
            n += 1
            if time.perf_counter() - t0 > max_wall:
                self._acc = 0.0           # cannot keep up: drop the backlog
                break
        return n

    # ------------------------------------------------------------------ telemetry
    def _display_air(self, o):
        """Air-data / inertial quantities as the displays show them.  Angles derived from a
        near-zero velocity are meaningless (alpha / bank flip through +-180 deg, the ground track
        follows millimetre-per-second creep), so like a real air-data system the vane angles are
        flagged invalid below ~50 KTAS on the ground and held at the last valid value, airspeeds
        come from the steady wind only, and the track / flight-path angle fall back to the heading /
        0 when the vehicle is (almost) not moving over the ground."""
        on_ground = any(self.gear.wow) or self.status in ("ROLLOUT", "STOPPED")
        la = self.last_aero or {}
        air_ok = not (on_ground and o["V"] < 25.0)
        if air_ok or getattr(self, "_air_hold", None) is None:
            self._air_hold = {"alpha": o["alpha"] * R2D if air_ok else o["theta"] * R2D,
                              "beta": o["beta"] * R2D if air_ok else 0.0,
                              "bank": o["mu"] * R2D if air_ok else o["phi"] * R2D,
                              "gamma_air": o["gamma"] * R2D if air_ok else 0.0,
                              "ld": la.get("ld", 0.0) if air_ok else 0.0}
        d = dict(self._air_hold, air_valid=bool(air_ok))
        x = self.x
        if air_ok:
            V, eas, qbar, mach = o["V"], o["eas"], o["qbar"], o["mach"]
        else:
            V = float(np.linalg.norm(x[3:6] - self.ctx.wind_w))      # steady wind only (no gusts)
            eas, qbar, mach = V * math.sqrt(o["rho"] / RHO0), 0.5 * o["rho"] * V * V, V / o["a"]
        d.update(tas=V, eas=eas, keas=eas * KT, qbar=qbar, mach=mach)
        moving = not on_ground or o["gs"] > 1.5
        d["track"] = o["track"] if moving else (o["psi"] * R2D) % 360.0
        d["gamma"] = o["gamma_i"] * R2D if moving else 0.0
        d["gs"] = o["gs"] if (not on_ground or o["gs"] > 0.3) else 0.0
        d["vs"] = x[5] if not (self.status == "STOPPED") else 0.0
        return d

    def state_message(self, seq=0):
        o = self.obs
        x = self.x
        R = o["R"]
        Fb = self.last_Fb
        n = Fb / (self.geo.mass * 9.80665)
        s = self.act.pos * R2D
        sb = self.act.pos[CH_SB]
        la = self.last_aero or {}
        thr33 = np.array([-self.terrain.rwy_half_len, 0.0])
        thr15 = np.array([self.terrain.rwy_half_len, 0.0])
        rw = np.array([o["along"], o["cross"]])
        cmd_alpha = self.gcmd.alpha_deg if self.mode == "AUTO" else self.man_alpha * R2D
        cmd_bank = self.gcmd.bank_deg if self.mode == "AUTO" else self.man_mu * R2D
        cmd_sb = self.fcs.cmd[CH_SB] * R2D
        da = self._display_air(o)
        return {
            "type": "state", "t": self.t, "seq": seq, "episode": int(self.episode), "paused": self.paused,
            "time_scale": self.time_scale,
            "mode": self.mode, "status": self.status, "guidance": self.guidance.name + (f":{self.gcmd.info}" if self.gcmd.info else ""),
            "pos": x[0:3].tolist(), "vel": x[3:6].tolist(), "quat": x[6:10].tolist(), "omega": x[10:13].tolist(),
            "air": {
                "alpha": da["alpha"], "beta": da["beta"], "mach": da["mach"], "tas": da["tas"],
                "eas": da["eas"], "keas": da["keas"], "qbar": da["qbar"], "rho": o["rho"],
                "nz": -n[2], "ny": n[1], "nx": n[0], "gamma": da["gamma"], "gamma_air": da["gamma_air"],
                "track": da["track"], "bank": da["bank"], "phi": o["phi"] * R2D, "theta": o["theta"] * R2D,
                "psi": (o["psi"] * R2D) % 360.0, "alt": x[2], "agl": o["agl"], "agl_gear": o["agl_gear"],
                "vs": da["vs"], "gs": da["gs"], "ld": da["ld"], "air_valid": da["air_valid"], "wind": o["wind"].tolist(),
                "p": x[10] * R2D, "q": x[11] * R2D, "r": x[12] * R2D,
            },
            "surf": {"elevon_li": s[CH_LI], "elevon_lo": s[CH_LO], "elevon_ri": s[CH_RI], "elevon_ro": s[CH_RO],
                     "bodyflap": s[CH_BF], "rudder": s[CH_RUD], "speedbrake": sb * R2D},
            "surf_cmd": {k: float(v) for k, v in zip(CTRL_NAMES, self.fcs.cmd * R2D)},
            "hinge": self.last_hinge,
            "cmd": {"alpha": cmd_alpha, "bank": cmd_bank, "speedbrake": cmd_sb,
                    "p": self.fcs.omega_cmd[0] * R2D, "q": self.fcs.omega_cmd[1] * R2D, "r": self.fcs.omega_cmd[2] * R2D},
            "gear": {"pos": self.gear.pos, "down_cmd": bool(self.gear.cmd_down), "wow": [bool(w) for w in self.gear.wow],
                     "compress": [float(c) for c in self.gear.compress]},
            "chute": self.chute_state, "chute_jettisoned": self.chute_jettisoned, "brakes": self.brakes,
            "steer": self.steer * R2D,
            "rwy": {"along": o["along"], "cross": o["cross"],
                    "dist_thr33": float(np.linalg.norm(rw - thr33)), "dist_thr15": float(np.linalg.norm(rw - thr15))},
            "gnc": self._gnc_telemetry(),
            "env": {"wind": self.wind.speed, "dir": self.wind.dir_from, "turb": self.wind.intensity,
                    "draft": self.wind.vertical(o["agl"]), "storm": self.wind._st, "auto": self.wind.auto},
        }

    def _gnc_telemetry(self):
        f = getattr(self.guidance, "telemetry", None)
        try:
            return (f() or {}).get("gnc") if f else None
        except Exception:
            return None

    def aero_message(self):
        o = self.obs
        r = self.aero.evaluate(o["v_air"], self.x[10:13], o["rho"], o["a"], o["mu_visc"], o["agl"], o["R"][2, :],
                               self.act.pos, gear_pos=self.gear.pos, chute=self.chute_state, detail=True)
        self.last_detail = r
        self.last_hinge = r.hinge
        return {"type": "aero", "t": self.t, "f": r.elem_f.tolist(), "comp": r.comp, "coef": r.coef,
                "xcp": r.xcp, "lift": r.lift, "drag": r.drag, "side": r.side,
                "mach": r.mach, "qbar": r.qbar}

    def hello_message(self):
        return {"type": "hello", "version": 1, "geometry": self.geo.raw, "world": self.geo.world,
                "elements": self.geo.elements, "controls": CTRL_NAMES,
                "guidance": {"active": self.guidance.name, "available": sorted(REGISTRY.keys())}}

    # ------------------------------------------------------------------ messages
    def handle(self, msg):
        t = msg.get("type")
        if t == "control":
            self.set_control(msg)
        elif t == "mode":
            self.set_mode(msg.get("mode", "AUTO"))
        elif t == "sim":
            a = msg.get("action")
            if a == "reset":
                seed = msg.get("seed")
                if msg.get("scenario") == "final":
                    # short-final demo (RWY33): on the A/L reference profile 1 km before the threshold
                    # (end of the pull-up, ~260 KEAS); exercises the transition onto the inner glide
                    # slope, final flare, touchdown, derotation, chute and rollout
                    from .gnc.approach import vertical_profile
                    H, _, _ = vertical_profile(ApproachConfig())
                    x_thr = -1000.0
                    h_cg = float(H(x_thr))
                    gam = math.degrees(math.atan((float(H(x_thr + 5.0)) - float(H(x_thr - 5.0))) / 10.0))
                    self.reset(init=approach_state(self.terrain, self.tables, height_agl=h_cg - 4.9, V=135.0,
                                                   gamma_deg=gam, offset_along=-self.terrain.rwy_half_len + x_thr))
                elif msg.get("scenario") == "custom":
                    init, warn = self._custom_init(msg.get("spawn") or {})
                    a = self.assess_spawn(init)
                    self.reset(init=init)
                    for w in warn:
                        self.event("warn", f"custom spawn: {w}")
                    self.event("info" if a.get("reachable") else "warn", f"custom spawn {a['summary']}")
                else:
                    self.reset(seed=int(seed) if seed is not None else None)
            elif a == "spawn_check":
                # reachability preview of a spawn spec, without moving the vehicle
                init, warn = self._custom_init(msg.get("spawn") or {})
                r = self.assess_spawn(init)
                hg = float(self.terrain.height(init.pos[0], init.pos[1]))
                self.events.append({"type": "spawn_check", "id": msg.get("id"), "warnings": warn,
                                    "pos": [float(v) for v in init.pos], "agl": float(init.pos[2] - hg),
                                    "alpha": float(init.alpha_deg), "desc": init.desc, **r})
            elif a == "pause":
                self.paused = True
            elif a == "resume":
                self.paused = False
            elif a == "toggle_pause":
                self.paused = not self.paused
            elif a == "set_time_scale":
                self.time_scale = float(np.clip(float(msg.get("value", 1.0)), 0.25, 8.0))
            elif a == "set_guidance":
                name = msg.get("value")
                if name in REGISTRY:
                    old = self.guidance
                    self.guidance_name = name
                    self.guidance = REGISTRY[name](self.gctx)
                    if hasattr(old, "close"):
                        old.close()
                    self.guidance.reset(self.snapshot())
                    self.event("info", f"guidance -> {name}")
        elif t == "env":
            self.wind.set(speed=msg.get("wind_speed"), dir_from=msg.get("wind_dir"),
                          intensity=msg.get("turbulence"), auto=msg.get("auto"))
            if not msg.get("quiet"):
                self.event("info", f"wind {self.wind.base_speed:.1f} m/s from {self.wind.base_dir:.0f} deg, "
                                   f"turbulence {self.wind.base_int:.2f}" + (" (auto)" if self.wind.auto else ""))
        elif t == "storm":
            self.wind.set_storm(turb=float(msg.get("turb", 0.0)), gust=float(msg.get("gust", 0.0)),
                                draft=float(msg.get("draft", 0.0)))
