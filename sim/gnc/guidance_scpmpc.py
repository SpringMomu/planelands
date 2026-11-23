"""Autonomous SCP + MPC guidance ("scp_mpc"), the default guidance law.

Phases:  PLAN -> TAEM (ACQ / HAC / PRF) -> A/L (OGS / PREFLARE / IGS / FLARE)
         -> TOUCHDOWN -> ROLLOUT -> STOPPED          (MAXRANGE if the field is unreachable)

* planning: runway/HAC selection, Dubins + HAC initial guess, free-final-time SCP to the
  A/L interface (worker process in the real-time server, synchronous in headless tests);
  replans every ~20 s (warm-started from the current reference) or on large errors
* tracking: LTV-MPC on the combined reference (TAEM plan + A/L reference) at 2.5 Hz with
  spatial re-indexing; the control sequence is interpolated between MPC updates
* flare / touchdown / rollout: proven sink-rate flare with tail-scrape protection, centreline
  capture by bank, derotation by the FCS, drag chute, brakes, nose-wheel steering
"""
from __future__ import annotations

import math

import numpy as np

from ..guidance import GuidanceBase, GuidanceCommand, register
from .approach import PH_FLARE, PH_IGS, PH_NAMES, PH_OGS, flare_alpha, lateral_bank, rollout_steer
from .geometry import TWO_PI, runway_ends, wrap_pi
from .model import G0, PlanningModel
from .mpc import AL_W, TAEM_W, Mpc
from .planner import AsyncPlanner, SyncPlanner, _Ctx, al_from_dict
from .reference import RefPath

R2D = 180.0 / math.pi
D2R = math.pi / 180.0


@register
class ScpMpcGuidance(GuidanceBase):
    name = "scp_mpc"
    rate_hz = 10.0
    mpc_period = 0.4
    replan_period = 20.0
    flare_h = 22.0           # main-gear height that starts the final flare

    def __init__(self, ctx):
        super().__init__(ctx)
        opts = getattr(ctx, "options", None) or {}
        self.planner_mode = opts.get("planner", "sync")
        self.ends = runway_ends(ctx.world)
        elev = ctx.world["runway"]["elevation"]
        self.pm = PlanningModel(ctx.tables, ctx.mass, ctx.S, ground_elev=elev)
        self.mpc = Mpc(self.pm)
        if self.planner_mode == "async":
            self.planner = AsyncPlanner()
        else:
            self.planner = SyncPlanner(_Ctx(ctx.tables, ctx.mass, ctx.S, ctx.world,
                                            getattr(ctx, "terrain", None)))
        self.msgs = []
        self.plan_log = []           # per-plan statistics (tests / diagnostics)
        self.mpc_stats = [0, 0.0, 0.0]   # count, sum ms, max ms
        self.plan_id = 0
        self.episode = 0
        self.last_plan_msg = None
        self._reset_state()

    # ------------------------------------------------------------------ infrastructure
    def _reset_state(self):
        self.phase = "PLAN"
        self.sub = ""
        self.ref = None
        self.rwy = None
        self.plan = None
        self.t_plan = -1e9
        self.t_req = -1e9
        self.t_mpc = -1e9
        self.t_mpcmsg = -1e9
        self.u_plan = None
        self.u_prev = None
        self.fi = 0.0
        self.gear = False
        self.flare = False
        self.t_nose = None
        self._chute_on = False
        self.mpc_fail = 0
        self.reachable = True
        self.maxrange = False
        self.low_warned = False
        self.tel = {}
        self.scp_info = {"iter": 0, "ms": 0.0, "converged": False, "cost": 0.0, "defect": 0.0, "age": 0.0,
                         "fallback": True}
        self.pending = False

    def event(self, level, text):
        self.msgs.append({"type": "event", "level": level, "text": text})

    def pop_messages(self):
        m, self.msgs = self.msgs, []
        return m

    def latest_plan_message(self):
        return self.last_plan_msg

    def close(self):
        self.planner.close()

    def _wind(self):
        f = getattr(self.ctx, "wind_info", None)
        return f() if f else (0.0, 0.0)

    def _state3(self, s):
        wE, wN = self.pm.wind(s.pos[2])
        vE, vN = s.vel[0] - float(wE), s.vel[1] - float(wN)
        psi = math.atan2(vE, vN)
        if self.ref is not None:
            # keep the heading on the reference branch (unwrapped)
            pr = self.ref.X[min(int(self.fi), len(self.ref.t) - 1), 5]
            psi = pr + wrap_pi(psi - pr)
        return np.array([s.pos[0], s.pos[1], s.pos[2], s.tas, math.radians(s.gamma), psi])

    # ------------------------------------------------------------------ planning
    def _request(self, t, s, x, warm=True, prefer=None):
        if self.planner.busy:
            return
        u0 = self.u_prev if self.u_prev is not None else np.array([s.alpha, 0.0, 0.0])
        req = dict(x0=x, u0=u0, t0=t, wind=self._wind(), prefer=prefer, episode=self.episode)
        if warm and self.ref is not None and self.plan is not None:
            fi, tau = self.ref.project(x)
            ia = self.ref.i_ali
            if int(fi) < ia - 3:
                i0 = int(fi)
                sl = slice(i0, ia + 1)
                tt = self.ref.t[sl].copy()
                tt[0] = tau
                req["warm"] = dict(rwy=self.plan["rwy"], cfg=self.plan["cfg"], hac=self.plan["hac"], t=tt,
                                   X=self.ref.X[sl], U=self.ref.U[sl], reachable=self.reachable)
        self.t_req = t
        self.pending = True
        self.planner.submit(req)
        if self.planner_mode == "sync":
            self._poll(t)

    def _poll(self, t):
        r = self.planner.poll()
        if r is None:
            return
        if r.get("episode") is not None and r.get("episode") != self.episode:
            return                       # stale result from before a reset
        self.pending = False
        if "error" in r:
            self.event("error", f"planner error: {r['error']}")
            return
        al = al_from_dict(r["al"], self.ends)
        prev_rwy = self.rwy
        ref = RefPath(self.pm, r["t0"], r["X"], r["U"], r["tf"], al, hac=r["hac"])
        self.ref = ref
        self.plan = r
        self.rwy = self.ends[r["rwy"]]
        self.t_plan = t
        self.fi = 0.0
        self.plan_id += 1
        self.reachable = r["reachable"]
        self.plan_log.append(dict(t=t, kind=r["kind"], iterations=int(r["iterations"]), ms=float(r["ms"]),
                                  wall_ms=float(r.get("wall_ms", r["ms"])), converged=bool(r["converged"]),
                                  fallback=bool(r["fallback"]), rwy=r["rwy"], reachable=bool(r["reachable"])))
        self.scp_info = {"iter": int(r["iterations"]), "ms": float(r["ms"]), "converged": bool(r["converged"]),
                         "cost": float(r["cost"]) if np.isfinite(r["cost"]) else 0.0,
                         "defect": float(r["defect"]) if np.isfinite(r["defect"]) else 0.0,
                         "age": 0.0, "fallback": bool(r["fallback"])}
        if self.phase == "PLAN":
            self.phase = "TAEM"
        nodes = []
        for k in range(r["X"].shape[0]):
            X, U = r["X"][k], r["U"][k]
            nodes.append([r["t0"] + r["tf"] * k / (r["X"].shape[0] - 1), X[0], X[1], X[2], X[3], X[4] * R2D,
                          (X[5] * R2D) % 360.0, U[0], U[1] * R2D, U[2]])
        self.last_plan_msg = {
            "type": "plan", "id": self.plan_id, "t0": r["t0"], "rwy": r["rwy"], "nodes": nodes,
            "hac": r["hac"], "ali": [float(v) for v in al.X[0, :3]],
            "approach": [[float(a), float(b), float(c)] for a, b, c in al.polyline(step=8)],
            "converged": bool(r["converged"]), "fallback": bool(r["fallback"]), "kind": r["kind"],
            "iterations": int(r["iterations"]), "ms": float(r["ms"]), "tf": float(r["tf"]),
        }
        self.msgs.append(self.last_plan_msg)
        txt = (f"plan #{self.plan_id} ({r['kind']}) RWY{r['rwy']}: SCP {r['iterations']} it, {r['ms']:.0f} ms, "
               f"{'converged' if r['converged'] else 'not converged'}, tf {r['tf']:.0f} s to ALI"
               + (" [geometric fallback]" if r["fallback"] else ""))
        if r["kind"] == "cold" or r["fallback"] or prev_rwy is None or prev_rwy.name != r["rwy"]:
            self.event("info", txt)
        if r.get("note") and r["note"] != getattr(self, "_last_note", None):
            self.event("warn" if not r["reachable"] else "info", r["note"])
        self._last_note = r.get("note")
        if not r["reachable"] and not self.low_warned:
            self.low_warned = True
            self.event("warn", "LOW ENERGY: runway not reachable - flying maximum range toward the field")
        self.maxrange = not r["reachable"] and r["fallback"]

    # ------------------------------------------------------------------ main update
    def reset(self, s):
        self.episode += 1
        if hasattr(self.planner, "cancel"):
            self.planner.cancel()
        self._reset_state()
        x = self._state3(s)
        self.u_prev = np.array([s.alpha, 0.0, 0.0])
        self.mpc._solver = None          # no warm-start state across episodes (reproducibility)
        self._request(s.t, s, x, warm=False)

    def update(self, t, s):
        self._poll(t)
        x = self._state3(s)
        if self.u_prev is None:
            self.u_prev = np.array([s.alpha, 0.0, 0.0])
        status = s.status
        # ---------------------------------------------------------- ground phases
        if status in ("ROLLOUT", "STOPPED", "CRASHED"):
            return self._ground(t, s, status)
        if self.ref is None:
            return self._no_plan(t, s)
        # ---------------------------------------------------------- projection & phase
        al_seg = self.ref.seg[min(int(self.fi), len(self.ref.t) - 1)] == 1
        fi, tau = self.ref.project(x, hint=int(self.fi), w_h=0.15 if al_seg else 3.0)
        self.fi = fi
        i = min(int(round(fi)), len(self.ref.t) - 1)
        seg = int(self.ref.seg[i])
        xr, ur, gr = self.ref.interp_index(fi)
        if s.agl_gear < self.flare_h and seg == 1:
            self.flare = True
        prev = (self.phase, self.sub)
        if self.flare:
            self.phase, self.sub = "A/L", "FLARE"
        elif seg == 1:
            self.phase, self.sub = "A/L", self.ref.sub_name(i)
        else:
            self.phase = "TAEM"
            self.sub = self.ref.sub_name(i)
        if self.maxrange:
            self.sub = "MAXRANGE"
        if (self.phase, self.sub) != prev and prev[0] != "PLAN":
            self.event("info", f"phase {self.phase} / {self.sub}  (h {s.alt:.0f} m, {s.eas / 0.514444:.0f} KEAS)")
        if s.agl_gear < 90.0 or (seg == 1 and self.ref.sub[i] >= PH_IGS):
            if not self.gear:
                self.event("info", f"gear down at {s.agl_gear:.0f} m")
            self.gear = True
        # ---------------------------------------------------------- replanning (TAEM)
        eh = float(self.pm.energy_height(x[2], x[3]) - self.pm.energy_height(xr[2], xr[3]))
        dpos = math.hypot(x[0] - xr[0], x[1] - xr[1])
        t_to_ali = self.ref.t_ali - tau
        if seg == 0 and not self.pending and t_to_ali > 35.0:
            due = t - self.t_plan > (30.0 if self.maxrange else self.replan_period)
            big = (not self.maxrange) and (dpos > 1500.0 or abs(x[2] - xr[2]) > 600.0 or abs(eh) > 900.0
                                           or self.mpc_fail >= 3)
            if due or big:
                if big:
                    self.event("info", f"replan: tracking error {dpos:.0f} m / dh {x[2] - xr[2]:.0f} m / "
                                       f"dE {eh:.0f} m")
                self.mpc_fail = 0
                self._request(t, s, x, warm=True)
                if self.planner_mode == "sync" and self.ref is not None:
                    fi, tau = self.ref.project(x, hint=0)
                    self.fi = fi
        # ---------------------------------------------------------- commands
        if self.flare:
            cmd = self._flare(t, s)
        elif self.maxrange:
            cmd = self._max_range(t, s, x)
        else:
            if t - self.t_mpc >= self.mpc_period - 1e-6 or self.u_plan is None:
                w = AL_W if seg == 1 else TAEM_W
                if seg == 1:
                    blim = math.radians(30.0 if s.agl > 300 else 12.0)
                else:
                    blim = math.radians(70.0 if s.mach > 1.05 else 60.0)
                u, Up, Xp, ok = self.mpc.solve(x, self.u_prev, self.ref, self.fi,
                                               gear_now=1.0 if self.gear else 0.0, w=w, bank_lim=blim)
                st = self.mpc_stats
                st[0] += 1
                st[1] += self.mpc.ms
                st[2] = max(st[2], self.mpc.ms)
                if ok:
                    self.u_plan = Up
                    self.mpc_fail = 0
                else:
                    self.mpc_fail += 1
                    self.u_plan = np.repeat(ur[None], self.mpc.K, axis=0)
                self.t_mpc = t
                if t - self.t_mpcmsg >= 0.35:
                    self.t_mpcmsg = t
                    self.msgs.append({"type": "mpc", "t": t, "pred": [[float(a), float(b), float(c)]
                                                                      for a, b, c in Xp[:, :3]]})
            k = (t - self.t_mpc) / self.mpc.dt
            j = int(min(max(math.floor(k), 0), len(self.u_plan) - 2))
            f = min(max(k - j, 0.0), 1.0)
            u = self.u_plan[j] + f * (self.u_plan[j + 1] - self.u_plan[j])
            cmd = GuidanceCommand(alpha_deg=float(u[0]), bank_deg=float(u[1] * R2D), speedbrake_deg=float(u[2]),
                                  gear_down=self.gear, info=f"{self.phase}/{self.sub}")
        self.u_prev = np.array([cmd.alpha_deg, cmd.bank_deg * D2R, cmd.speedbrake_deg])
        # ---------------------------------------------------------- telemetry
        self.tel = {
            "phase": self.phase, "sub": self.sub, "rwy": self.rwy.name if self.rwy else None,
            "energy_err": eh, "range_to_go": self.ref.range_to_go(fi), "time_to_go": max(self.ref.t_end - tau, 0.0),
            "time_to_ali": max(t_to_ali, 0.0), "reachable": bool(self.reachable),
            "scp": dict(self.scp_info, age=t - self.t_plan, pending=self.pending),
            "mpc": {"ms": self.mpc.ms, "status": self.mpc.status},
            "ref": {"alpha": float(ur[0]), "bank": float(ur[1] * R2D), "sb": float(ur[2]), "h": float(xr[2]),
                    "V": float(xr[3])},
            "plan_id": self.plan_id, "track_err": dpos, "h_err": float(x[2] - xr[2]),
        }
        return cmd

    # ------------------------------------------------------------------ sub-laws
    def _no_plan(self, t, s):
        self.phase = "PLAN"
        a = min(float(self.ctx.tables.best_ld_alpha(s.mach)), float(self.pm.alpha_max(s.mach)))
        self.tel = {"phase": "PLAN", "sub": "WAIT", "rwy": None, "energy_err": 0.0, "range_to_go": 0.0,
                    "time_to_go": 0.0, "reachable": True, "scp": dict(self.scp_info, age=0.0, pending=self.pending),
                    "mpc": {"ms": 0.0, "status": "idle"},
                    "ref": {"alpha": a, "bank": 0.0, "sb": 0.0, "h": s.alt, "V": s.tas}}
        return GuidanceCommand(alpha_deg=a, bank_deg=0.0, speedbrake_deg=0.0, info="PLAN/WAIT")

    def _max_range(self, t, s, x):
        """Unreachable field: best L/D, turn toward the threshold."""
        thr = self.rwy.thr if self.rwy is not None else np.zeros(2)
        brg = math.atan2(thr[0] - s.pos[0], thr[1] - s.pos[1])
        err = wrap_pi(brg - x[5])
        bank = float(np.clip(err * 1.5, -math.radians(40), math.radians(40)))
        a = float(self.ctx.tables.best_ld_alpha(s.mach, 0.0, 1.0 if self.gear else 0.0))
        a = min(a, float(self.pm.alpha_max(s.mach)))
        bank = float(np.clip(bank, -math.radians(30), math.radians(30)))
        return GuidanceCommand(alpha_deg=a, bank_deg=bank * R2D, speedbrake_deg=0.0, gear_down=self.gear,
                               info="TAEM/MAXRANGE")

    def _flare(self, t, s):
        a = flare_alpha(self.ctx.tables, self.ctx.mass, self.ctx.S, s, gear=1.0)
        bank = lateral_bank(self.rwy, s, bank_lim_deg=5.0) if self.rwy is not None else 0.0
        sb = float(self.ref.al.sb_pf) if self.ref is not None else 0.0
        return GuidanceCommand(alpha_deg=a, bank_deg=bank, speedbrake_deg=sb, gear_down=True, info="A/L/FLARE")

    def _ground(self, t, s, status):
        nose = bool(s.wow[0])
        if status == "STOPPED":
            self.phase, self.sub = "STOPPED", ""
        elif status == "CRASHED":
            self.phase, self.sub = "CRASHED", ""
        elif not nose:
            self.phase, self.sub = "TOUCHDOWN", "DEROTATION"
        else:
            self.phase, self.sub = "ROLLOUT", ""
        if nose and self.t_nose is None:
            self.t_nose = t
        rwy = self.rwy or self.ends["33"]
        steer, xr, yr = rollout_steer(rwy, s)
        gs = s.ground_speed
        brakes = 0.0
        if nose and gs < 85.0:
            brakes = 0.55
        if gs < 15.0:
            brakes = 1.0
        # drag chute once the nose wheel is firmly down and below ~180 KTAS (limits the nose-up
        # moment of the chute); speedbrake fully open only at lower speed for the same reason
        chute = (nose and self.t_nose is not None and t - self.t_nose > 1.5 and gs > 32.0
                 and (s.tas < 93.0 or self._chute_on))
        self._chute_on = chute or self._chute_on
        sb_fl = float(self.ref.al.sb_pf) if self.ref is not None else 0.0
        sb = 98.6 if (nose and s.tas < 80.0) else (sb_fl if nose else sb_fl)
        self.tel = dict(self.tel, phase=self.phase, sub=self.sub, range_to_go=0.0, time_to_go=0.0)
        return GuidanceCommand(alpha_deg=0.0, bank_deg=0.0, speedbrake_deg=sb, gear_down=True,
                               chute=chute, brakes=brakes, steer_deg=steer, info=self.phase)

    def telemetry(self):
        return {"gnc": self.tel}
