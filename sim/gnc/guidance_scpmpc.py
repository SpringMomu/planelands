"""Autonomous SCP + MPC guidance ("scp_mpc"), the default guidance law.

Phases:  PLAN -> TAEM (ACQ / HAC / PRF) -> A/L (OGS / PREFLARE / IGS / FLARE)
         -> TOUCHDOWN -> ROLLOUT -> STOPPED

* degraded modes, while no SCP plan is usable (SCP failed; a failed re-plan first keeps a good
  plan that is still tracked, up to ``keep_fail_max`` times), chosen by the planner's energy
  classification: GEOREF (energy fine) - the MPC tracks the geometric reference of the selected
  route (Dubins + HAC energy profile); MAXRANGE (energy low or turn limited) flies at best L/D
  toward the farthest point of the approach geometry it can still reach, then along the extended
  centreline; HOLD (energy high) circles the HAC centre with the speedbrake out.  MAXRANGE / HOLD
  look ahead for terrain (the SCP's clearance constraint is not active there) and limit bank close
  to the ground; all three re-plan from scratch every ``degraded_replan`` s until a plan is usable.

* planning: runway/HAC selection, Dubins + HAC initial guess, free-final-time SCP to the
  A/L interface (worker process in the real-time server, synchronous in headless tests);
  replans every ~20 s (warm-started from the current reference), on large errors or when the
  wind estimate changes
* approach re-selection: until HAC acquisition (and > ``freeze_t_ali`` s before the ALI) a replan
  re-runs the runway-end / HAC-side choice with the current wind and switches to a different
  approach when it is clearly better (hysteresis, at most ``max_switches``) - the discrete
  re-decision that a warm-started SCP alone can never make.  ``reselect_mode`` "wind" (default):
  only once the wind differs from the one the approach was chosen with, or on a large energy /
  tracking error; "always": on every replan; "off": never
* tracking: LTV-MPC on the combined reference (TAEM plan + A/L reference) at 2.5 Hz with
  spatial re-indexing; the control sequence is interpolated between MPC updates
* flare / touchdown / rollout: proven sink-rate flare with tail-scrape protection, centreline
  capture by bank, derotation by the FCS, drag chute, brakes, nose-wheel steering
"""
from __future__ import annotations

import math
import os

import numpy as np

from ..guidance import GuidanceBase, GuidanceCommand, register
from .approach import PH_FLARE, PH_IGS, PH_NAMES, PH_OGS, flare_alpha, lateral_bank, rollout_steer
from .geometry import TWO_PI, ApproachConfig, runway_ends, wrap_pi
from .model import G0, PlanningModel
from .mpc import AL_W, TAEM_W, Mpc
from types import SimpleNamespace

from .planner import AsyncPlanner, SyncPlanner, _Ctx, _final_approach, al_from_dict
from .reference import RefPath
from .scp import TerrainField

R2D = 180.0 / math.pi
D2R = math.pi / 180.0


@register
class ScpMpcGuidance(GuidanceBase):
    name = "scp_mpc"
    rate_hz = 10.0
    mpc_period = 0.4
    replan_period = 20.0
    freeze_t_ali = 60.0      # no approach re-selection closer than this to the ALI ...
    max_switches = 2         # ... or after this many switches per flight
    switch_dwell = 40.0      # min time between switches (s)
    wind_replan_dv = 3.0     # wind change (vector, m/s at 10 m) that triggers an immediate replan ...
    wind_replan_dt = 10.0    # ... at most this often (s)
    reselect_mode = "wind"   # "wind" | "always" | "off"
    flare_h = 22.0           # main-gear height that starts the final flare
    degraded_replan = 5.0    # degraded modes: cold re-plan this often (s)
    terrain_clear = 250.0    # degraded modes: required clearance over terrain above the field (m) ...
    terrain_look = (8.0, 16.0, 24.0, 32.0, 45.0, 60.0)    # ... along the predicted glide (s ahead)
    hold_sb = 80.0           # HOLD: speedbrake (deg)
    keep_fail_max = 3        # failed re-plans in a row that keep the (good) plan being flown ...
    keep_track_max = 4000.0  # ... while its tracking error stays below this (m)

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
        terr = getattr(ctx, "terrain", None)
        self.tfield = TerrainField(terr) if terr is not None else None
        self.terrain_top = float(self.tfield.g.max()) if self.tfield is not None and self.tfield.ok else 0.0
        self.rwy_elev = float(elev)
        # tests: GNC_FAULT_SCP=<s> - every plan requested in the first <s> s of a flight fails
        self.fault_scp_until = float(os.environ.get("GNC_FAULT_SCP", "0") or 0.0)
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
        self.energy_high = False
        self.maxrange = False        # a degraded mode is active (no usable plan)
        self.degraded = None         # "MAXRANGE" | "HOLD" | None
        self.energy_dir = "ok"       # ok / high / low / turn (planner's energy classification)
        self.hold = None             # HOLD circle: centre, direction
        self.aim = None              # MAXRANGE aim point (runway end, x along it)
        self.t_aim = -1e9
        self.captured = False        # MAXRANGE: on the extended centreline
        self.terrain_conflict = False
        self.t_deg = None
        self.keep_fail = 0
        self.replan_block_until = -1e9
        self.low_warned = False
        self.tel = {}
        self.scp_info = {"iter": 0, "ms": 0.0, "converged": False, "cost": 0.0, "defect": 0.0, "age": 0.0,
                         "fallback": True}
        self.pending = False
        self.n_switch = 0
        self.t_switch = -1e9
        self.wind_plan = None        # wind (speed, dir_from) the current plan was made with
        self.wind_class = None       # ... the current runway end / HAC side was chosen with
        self.switch_log = []
        self.decision = None
        self.prev_approach = None
        self.last_prev_msg = None

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

    def _wind_change(self, ref=None):
        """Vector change (m/s) of the 10 m wind since the current plan (or ``ref`` wind) was made."""
        ref = self.wind_plan if ref is None else ref
        if ref is None:
            return 0.0
        (s1, d1), (s0, d0) = self._wind(), ref
        a1, a0 = math.radians(d1), math.radians(d0)
        return math.hypot(s1 * math.sin(a1) - s0 * math.sin(a0), s1 * math.cos(a1) - s0 * math.cos(a0))

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
    def _request(self, t, s, x, warm=True, prefer=None, reselect=False):
        if self.planner.busy:
            return
        u0 = self.u_prev if self.u_prev is not None else np.array([s.alpha, 0.0, 0.0])
        req = dict(x0=x, u0=u0, t0=t, wind=self._wind(), prefer=prefer, episode=self.episode,
                   reselect=bool(reselect))
        if t < self.fault_scp_until:
            req["fault_scp"] = True
            warm = False
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
        self._accept(r, t)

    def _accept(self, r, t):
        if r.get("episode") is not None and r.get("episode") != self.episode:
            return                       # stale result from before a reset
        self.pending = False
        if "error" in r:
            self.event("error", f"planner error: {r['error']}")
            return
        if (r.get("fallback") and not self.maxrange and self.plan is not None and self.plan.get("ok")
                and self.ref is not None and self.keep_fail < self.keep_fail_max
                and float(self.tel.get("track_err", 0.0)) < self.keep_track_max):
            # a re-plan failed but the plan being flown was good and is still tracked: keep flying
            # it and try again shortly, instead of dropping it for the geometric fallback
            self.keep_fail += 1
            self.replan_block_until = t + self.degraded_replan
            self.t_plan = t - self.replan_period + self.degraded_replan
            self.event("warn", f"re-plan failed ({r.get('note') or 'SCP'}) - keeping plan #{self.plan_id} "
                               f"({self.keep_fail}/{self.keep_fail_max})")
            return
        if not r.get("fallback"):
            self.keep_fail = 0
        al = al_from_dict(r["al"], self.ends)
        prev_rwy = self.rwy
        prev_plan = self.plan
        if r.get("wind") is not None:
            self.wind_plan = tuple(r["wind"])
            if r["kind"] != "warm" or self.wind_class is None:
                self.wind_class = tuple(r["wind"])
        dec = r.get("decision")
        if dec is not None:
            self.decision = dict(dec, t=t)
        if r["kind"] == "switch":
            self.n_switch += 1
            self.t_switch = t
            self.switch_log.append(dict(t=t, h=float(r["X"][0, 2]), **dec))
            self.event("warn", f"HAC switch: {dec['frm']} -> {dec['to']} ({dec['reason']}; energy band "
                               f"{dec['cur_band']:.2f} -> {dec['new_band']:.2f}, wind "
                               f"{r['wind'][0]:.0f} m/s from {r['wind'][1]:03.0f})")
        elif dec is not None and dec.get("rejected"):
            self.event("info", f"approach re-selection: {dec['new_rwy']} {'right' if dec['new_side'] > 0 else 'left'}"
                               f" HAC rejected ({dec['rejected']})")
        ref = RefPath(self.pm, r["t0"], r["X"], r["U"], r["tf"], al, hac=r["hac"])
        self.ref = ref
        self.plan = r
        self.rwy = self.ends[r["rwy"]]
        self.t_plan = t
        self.fi = 0.0
        self.plan_id += 1
        self.reachable = r["reachable"]
        note_ = r.get("note") or ""
        self.energy_dir = r.get("energy") or ("high" if "HIGH ENERGY" in note_ else "ok" if r["reachable"] else "low")
        self.energy_high = (not r["reachable"]) and self.energy_dir == "high"
        self.plan_log.append(dict(t=t, kind=r["kind"], iterations=int(r["iterations"]), ms=float(r["ms"]),
                                  wall_ms=float(r.get("wall_ms", r["ms"])), converged=bool(r["converged"]),
                                  fallback=bool(r["fallback"]), rwy=r["rwy"], reachable=bool(r["reachable"]),
                                  hac_dir=int(r["hac"]["dir"]) if r.get("hac") else 0, decision=dec))
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
            "switch": r["kind"] == "switch",
        }
        if r["kind"] == "switch" and prev_plan is not None:
            # previous approach, for the displays (abandoned HAC + approach line)
            pm_ = self.last_prev_msg or {}
            self.prev_approach = {"rwy": prev_plan["rwy"], "hac": prev_plan.get("hac"), "t": t,
                                  "approach": pm_.get("approach"),
                                  "track": [[n[1], n[2]] for n in pm_.get("nodes", [])]}
        if self.prev_approach is not None:
            self.last_plan_msg["prev"] = self.prev_approach
        self.msgs.append(self.last_plan_msg)
        self.last_prev_msg = self.last_plan_msg
        txt = (f"plan #{self.plan_id} ({r['kind']}) RWY{r['rwy']}: SCP {r['iterations']} it, {r['ms']:.0f} ms, "
               f"{'converged' if r['converged'] else 'not converged'}, tf {r['tf']:.0f} s to ALI"
               + (" [geometric fallback]" if r["fallback"] else ""))
        if r["kind"] in ("cold", "switch") or r["fallback"] or prev_rwy is None or prev_rwy.name != r["rwy"]:
            self.event("info", txt)
        if r.get("note") and r["note"] != getattr(self, "_last_note", None):
            self.event("warn" if not r["reachable"] else "info", r["note"])
        self._last_note = r.get("note")
        if not r["reachable"] and not self.low_warned:
            self.low_warned = True
            if self.energy_high:
                self.event("warn", "HIGH ENERGY: too close for this altitude / speed - best-effort energy dissipation")
            elif self.energy_dir == "turn":
                self.event("warn", "TURN LIMITED: too fast to turn back yet - gliding toward the field")
            elif self.energy_dir == "low":
                self.event("warn", "LOW ENERGY: runway not reachable - flying maximum range toward the field")
        deg = (not r["reachable"]) and bool(r["fallback"])
        mode = (({"high": "HOLD", "ok": "GEOREF"}).get(self.energy_dir, "MAXRANGE")) if deg else None
        if mode != self.degraded:
            if mode is None:
                self.event("info", f"degraded mode {self.degraded} ended after {t - (self.t_deg or t):.0f} s: "
                                   f"plan #{self.plan_id} usable")
                self.u_plan = None           # MPC solves on the new plan right away (no stale sequence)
                self.t_deg = None
            else:
                self.t_deg = t if self.degraded is None else self.t_deg
                self.aim, self.captured = None, False
                why = {"HOLD": "energy high: circling the HAC centre, speedbrake out",
                       "GEOREF": "energy fine: tracking the geometric reference",
                       "MAXRANGE": "best L/D toward the approach"}[mode]
                self.event("warn", f"no usable plan (SCP failed): degraded mode {mode} ({why}), "
                                   f"re-planning every {self.degraded_replan:.0f} s")
        if mode == "HOLD" and r.get("hac"):
            self.hold = {"center": np.array(r["hac"]["center"], float), "dir": int(r["hac"]["dir"]) or 1}
        self.degraded = mode
        self.maxrange = mode in ("MAXRANGE", "HOLD")      # own control laws; GEOREF flies the MPC
        if mode is not None:
            self.replan_block_until = t + self.degraded_replan

    # ------------------------------------------------------------------ main update
    def reset(self, s):
        self.episode += 1
        if hasattr(self.planner, "cancel"):
            self.planner.cancel()
        self._reset_state()
        x = self._state3(s)
        self.u_prev = np.array([s.alpha, 0.0, 0.0])
        self.mpc._solver = None          # no warm-start state across episodes (reproducibility)
        if self.planner_mode != "sync":
            # already on final (e.g. the short-final demo): the A/L reference needs no SCP, build it
            # here instead of flying the first seconds without a plan while the worker is busy
            fin = _final_approach(SimpleNamespace(ends=self.ends), self.pm, x)
            if fin is not None:
                fin.update(t0=s.t, episode=self.episode, wind=self._wind())
                self.pending = False
                self._accept(fin, s.t)
                return
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
        if self.degraded is not None:
            self.sub = self.degraded
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
        if self.degraded is not None:
            # degraded: no usable plan - cold re-plan from the current state, often (route kept on
            # the same runway end unless the selection finds it clearly better)
            if not self.pending and t - self.t_req > self.degraded_replan and t >= self.replan_block_until:
                self._request(t, s, x, warm=False, prefer=self.rwy.name if self.rwy is not None else None)
                if self.planner_mode == "sync" and self.ref is not None and not self.maxrange:
                    fi, tau = self.ref.project(x, hint=0)
                    self.fi = fi
                    i = min(int(round(fi)), len(self.ref.t) - 1)
                    seg = int(self.ref.seg[i])
                    xr, ur, gr = self.ref.interp_index(fi)
                    t_to_ali = self.ref.t_ali - tau
        elif seg == 0 and not self.pending and t_to_ali > 35.0 and t >= self.replan_block_until:
            due = t - self.t_plan > self.replan_period
            big = (dpos > 1500.0 or abs(x[2] - xr[2]) > 600.0 or abs(eh) > 900.0 or self.mpc_fail >= 3)
            dw = self._wind_change()
            windy = dw > self.wind_replan_dv and t - self.t_plan > self.wind_replan_dt
            if due or big or windy:
                if big:
                    self.event("info", f"replan: tracking error {dpos:.0f} m / dh {x[2] - xr[2]:.0f} m / "
                                       f"dE {eh:.0f} m")
                elif windy:
                    w = self._wind()
                    self.event("info", f"replan: wind update {w[0]:.0f} m/s from {w[1]:03.0f} "
                                       f"(changed {dw:.1f} m/s since the last plan)")
                # approach re-selection only before HAC acquisition, not too close to the ALI
                news = (self.reselect_mode == "always" or big
                        or self._wind_change(self.wind_class) > self.wind_replan_dv)
                resel = (self.reselect_mode != "off" and news and self.sub == "ACQ"
                         and t_to_ali > self.freeze_t_ali and self.n_switch < self.max_switches
                         and t - self.t_switch > self.switch_dwell)
                self.mpc_fail = 0
                self._request(t, s, x, warm=True, reselect=resel)
                if self.planner_mode == "sync" and self.ref is not None:
                    fi, tau = self.ref.project(x, hint=0)
                    self.fi = fi
        # ---------------------------------------------------------- commands
        if self.flare:
            cmd = self._flare(t, s)
        elif self.maxrange:
            cmd = self._degraded_cmd(t, s, x)
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
            "energy_high": bool(self.energy_high),
            "scp": dict(self.scp_info, age=t - self.t_plan, pending=self.pending),
            "mpc": {"ms": self.mpc.ms, "status": self.mpc.status},
            # degraded: the reference is not flown - show the active command, no h / V reference
            "ref": ({"alpha": cmd.alpha_deg, "bank": cmd.bank_deg, "sb": cmd.speedbrake_deg, "h": None, "V": None}
                    if self.maxrange else
                    {"alpha": float(ur[0]), "bank": float(ur[1] * R2D), "sb": float(ur[2]), "h": float(xr[2]),
                     "V": float(xr[3])}),
            "energy": self.energy_dir, "degraded": self.degraded, "terrain_conflict": bool(self.terrain_conflict),
            "plan_id": self.plan_id, "track_err": dpos, "h_err": float(x[2] - xr[2]),
            "switches": self.n_switch,
            "hac_side": (int(self.plan["hac"]["dir"]) if self.plan and self.plan.get("hac") else 0),
            "reselect": ({k: self.decision[k] for k in ("cur_band", "new_band", "cur_score", "new_score",
                                                         "new_rwy", "new_side") if k in self.decision}
                         if self.decision else None),
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
        # never retract a gear that is already down close to the ground while waiting for a plan
        gear = bool(s.gear_pos > 0.5 and s.agl_gear < 600.0)
        return GuidanceCommand(alpha_deg=a, bank_deg=0.0, speedbrake_deg=0.0, gear_down=gear, info="PLAN/WAIT")

    # ------------------------------------------------------------------ degraded modes
    def _bank_to(self, psi_cmd, s, ff=0.0, lim_deg=45.0):
        """Bank for a ground-track heading command: first-order heading response (time constant
        ~12 s) plus a feed-forward bank, limited."""
        trk = math.atan2(s.vel[0], s.vel[1])
        err = wrap_pi(psi_cmd - trk)
        V = max(s.tas, 60.0)
        bank = math.atan(V * float(np.clip(err / 12.0, -0.15, 0.15)) / G0) + ff
        return float(np.clip(bank, -math.radians(lim_deg), math.radians(lim_deg)))

    def _alpha_glide(self, s, sb):
        a = float(self.ctx.tables.best_ld_alpha(s.mach, sb, 1.0 if self.gear else 0.0))
        # dynamic-pressure guard: more alpha when fast (slows down), less when slow
        a += float(np.clip((s.qbar - 9500.0) / 2000.0, -2.0, 2.0))
        return float(np.clip(a, 1.0, float(self.pm.alpha_max(s.mach))))

    def _maxrange_aim(self, s, x):
        """Farthest point of the approach geometry (WP2, ALI or the extended centreline, either runway
        end) still in reach at best L/D, with a turn-to-align allowance; if none is, the one closest
        to being reachable.  Returns (runway end, x along it)."""
        cfg = ApproachConfig()
        Eh = float(self.pm.energy_height(s.alt, s.tas)) - self.rwy_elev
        ldb = float(self.pm.ld_best(s.mach))
        ld_eff = 0.9 * (ldb if s.mach < 0.9 else 0.5 * (ldb + float(self.pm.ld_best(0.6))))
        opts = []
        for end in self.ends.values():
            for xa in (cfg.x_wp2, cfg.x_ali, -6000.0, -3000.0, 0.0):
                Ea, Na = end.from_rwy(xa, 0.0)
                dE, dN = float(Ea) - s.pos[0], float(Na) - s.pos[1]
                D = math.hypot(dE, dN)
                ang = abs(wrap_pi(end.heading - math.atan2(dE, dN)))
                need = 600.0 + abs(xa) / 4.0 + (D + 3500.0 * ang) / ld_eff
                opts.append((end, xa, Eh - need, need))
        ok = [o for o in opts if o[2] > 0.08 * o[3]]
        if ok:
            o = max(ok, key=lambda o: (-o[1], o[2]))       # farthest out (most negative x), then margin
        else:
            o = max(opts, key=lambda o: o[2])
        return o[0], o[1]

    def _maxrange_law(self, t, s, x):
        """Best L/D toward the farthest reachable point of the approach, then along the centreline
        (the planner takes over again with a final-approach plan once established)."""
        if self.aim is None or (t - self.t_aim > 2.0 and not self.captured):
            self.aim = self._maxrange_aim(s, x)
            self.t_aim = t
        end, xa = self.aim
        self.rwy = end
        xr, yr = end.to_rwy(s.pos[0], s.pos[1])
        xr, yr = float(xr), float(yr)
        trk = math.atan2(s.vel[0], s.vel[1])
        herr = abs(wrap_pi(trk - end.heading))
        Ea, Na = end.from_rwy(xa, 0.0)
        D = math.hypot(float(Ea) - s.pos[0], float(Na) - s.pos[1])
        if not self.captured and (D < 2500.0 or (xa - 1500.0 < xr < end.length and abs(yr) < 1500.0
                                                 and herr < math.radians(45.0))):
            self.captured = True
            self.event("info", f"MAXRANGE: on the RWY{end.name} centreline {abs(xr) / 1000:.1f} km out, "
                               f"{s.agl:.0f} m AGL")
        if self.captured and (xr > 0.0 and s.agl > 300.0 or herr > math.radians(100.0)):
            # overflew the threshold still high, or turned away: not an approach any more - re-aim
            self.captured = False
            self.aim = self._maxrange_aim(s, x)
            self.t_aim = t
            self.event("info", f"MAXRANGE: left the RWY{end.name} centreline ({s.agl:.0f} m AGL) - re-aiming")
            end, xa = self.aim
            Ea, Na = end.from_rwy(xa, 0.0)
        if self.captured:
            psi_d = end.heading - math.atan(float(np.clip(yr / 2000.0, -0.8, 0.8)))
        else:
            psi_d = math.atan2(float(Ea) - s.pos[0], float(Na) - s.pos[1])
        return psi_d, 0.0, 0.0

    def _hold_law(self, t, s, x):
        """Energy high: circle the HAC centre (radius for ~35 deg bank, shrinking as the vehicle
        slows), speedbrake out; re-plans pick a route as soon as the energy fits."""
        if self.hold is None:
            end = self.rwy or self.ends["33"]
            c = ApproachConfig()
            self.hold = {"center": np.array(end.from_rwy(c.x_wp2, c.r_hac), float), "dir": 1}
        C, d = self.hold["center"], self.hold["dir"]
        V = max(s.ground_speed, 60.0)
        R = max(V * V / (G0 * math.tan(math.radians(35.0))), 4000.0)
        dE, dN = s.pos[0] - C[0], s.pos[1] - C[1]
        dist = math.hypot(dE, dN)
        th = math.atan2(dE, dN)
        psi_d = th + d * (math.pi / 2.0 + math.atan((dist - R) / 3000.0))
        ff = d * math.atan(V * V / (G0 * R)) if abs(dist - R) < 0.5 * R else 0.0
        return psi_d, ff, self.hold_sb

    def _terrain_heading(self, s, psi_d):
        """Terrain look-ahead along the predicted glide (current flight-path angle, at least the best
        L/D glide) for the desired heading; if it conflicts with terrain that rises above the field,
        the nearest clear heading (up to +-180 deg).  Ground at field elevation is not a conflict -
        the vehicle is meant to get down there.  Returns (heading, min clearance, conflict)."""
        tf = self.tfield
        if tf is None or not tf.ok or s.pos[2] > self.terrain_top + 3000.0:
            return psi_d, float("inf"), False
        Vg = max(s.ground_speed, 50.0)
        d = Vg * np.asarray(self.terrain_look)
        # glide slope (average L/D) - a dive steeper than that only for the first ~10 s
        glide = -1.0 / max(0.85 * float(self.pm.ld_best(s.mach)), 1.0)
        tg = math.tan(math.radians(s.gamma_i))
        h_pred = s.pos[2] + d * glide + np.minimum(d, Vg * 10.0) * min(tg - glide, 0.0)
        best = (psi_d, -float("inf"))
        for off in (0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 120, -120, 150, -150, 180):
            psi = psi_d + math.radians(off)
            E = s.pos[0] + d * math.sin(psi)
            N = s.pos[1] + d * math.cos(psi)
            hT = tf.lin(E, N)[0]
            near = np.zeros(len(d), bool)
            for end in self.ends.values():
                a, c = end.to_rwy(E, N)
                near |= (a > -4000.0) & (a < end.length + 1000.0) & (np.abs(c) < 1500.0)
            relevant = (hT > self.rwy_elev + 100.0) & ~near
            clr = float(np.min(np.where(relevant, h_pred - hT, np.inf)))
            if clr > self.terrain_clear:
                return psi, clr, off != 0
            if clr > best[1]:
                best = (psi, clr)
        return best[0], best[1], True

    def _degraded_cmd(self, t, s, x):
        if self.degraded == "HOLD":
            psi_d, ff, sb = self._hold_law(t, s, x)
        else:
            psi_d, ff, sb = self._maxrange_law(t, s, x)
        psi_c, clr, conflict = self._terrain_heading(s, psi_d)
        if conflict != self.terrain_conflict:
            self.event("warn" if conflict else "info",
                       f"{self.degraded}: terrain ahead - turning {math.degrees(wrap_pi(psi_c - psi_d)):+.0f} deg, "
                       f"clearance {clr:.0f} m" if conflict else f"{self.degraded}: terrain clear")
        self.terrain_conflict = conflict
        if conflict:
            ff, sb = 0.0, 0.0                       # stretch the glide over the terrain
        near_rwy = any(abs(float(e.to_rwy(s.pos[0], s.pos[1])[1])) < 400.0 and
                       -6000.0 < float(e.to_rwy(s.pos[0], s.pos[1])[0]) < e.length for e in self.ends.values())
        lim = 35.0 if s.mach > 1.0 else 45.0
        if s.agl < 600.0 and not near_rwy:
            lim = 25.0
        if s.agl < 150.0:
            lim = 10.0
        bank = self._bank_to(psi_c, s, ff, lim)
        alpha = self._alpha_glide(s, sb)
        if s.agl < 50.0 and not near_rwy:
            # off-field forced landing: wings level, flare to reduce the sink rate at impact
            bank = 0.0
            alpha = min(alpha + 4.0 * (1.0 - max(s.agl, 0.0) / 50.0), float(self.pm.alpha_max(s.mach)))
            sb = 0.0
        return GuidanceCommand(alpha_deg=alpha, bank_deg=bank * R2D, speedbrake_deg=sb, gear_down=self.gear,
                               info=f"TAEM/{self.degraded}")

    def _flare(self, t, s):
        sb = float(self.ref.al.sb_pf) if self.ref is not None else 0.0
        sink_td = 0.9
        if self.ref is not None and self.rwy is not None:
            # touchdown-zone awareness: floating past the aim point (excess speed, tailwind, shear
            # balloon) -> settle it with a firmer sink and the speedbrake open instead of using
            # up the runway
            x, _ = self.rwy.to_rwy(s.pos[0], s.pos[1])
            over = float(x) - float(self.ref.al.keys["x_td"])
            if over > 0.0:
                sink_td += float(np.clip(over / 300.0, 0.0, 1.0)) * 1.0
                sb += (98.6 - sb) * float(np.clip(over / 250.0, 0.0, 1.0))
        a = flare_alpha(self.ctx.tables, self.ctx.mass, self.ctx.S, s, gear=1.0, sink_td=sink_td)
        bank = lateral_bank(self.rwy, s, bank_lim_deg=5.0) if self.rwy is not None else 0.0
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
        # drag chute: deployed as soon as the main gear is on the ground (before derotation);
        # jettisoned by the simulator below ~60 KGS
        main = bool(s.wow[1] or s.wow[2])
        chute = self._chute_on or (main and gs > 32.0)
        self._chute_on = chute
        sb_fl = float(self.ref.al.sb_pf) if self.ref is not None else 0.0
        sb = 98.6 if (nose and s.tas < 80.0) else (sb_fl if nose else sb_fl)
        self.tel = dict(self.tel, phase=self.phase, sub=self.sub, range_to_go=0.0, time_to_go=0.0)
        return GuidanceCommand(alpha_deg=0.0, bank_deg=0.0, speedbrake_deg=sb, gear_down=True,
                               chute=chute, brakes=brakes, steer_deg=steer, info=self.phase)

    def telemetry(self):
        return {"gnc": self.tel}
