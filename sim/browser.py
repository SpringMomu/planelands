"""Browser build entry point (Pyodide in a Web Worker - see web/js/sim/sim_worker.js).

The page talks to this exactly as it talks to server.py over the WebSocket: the same JSON messages in
both directions. The JavaScript worker drives the real-time loop and calls ``step(dt)`` ~60 times per
second; the SCP planner runs in a second worker (``ExternalPlanner``), like the planner process of
the server build.
"""
from __future__ import annotations

import json
import time

import numpy as np

from server import AERO_EVERY


def _np(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, np.generic):
        return o.item()
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")


def dumps(msg) -> str:
    """server.dumps without the significant-digit rounding (which only shrinks network traffic and
    costs ~10 % of a tick in Python): messages stay in the browser. Still raises ValueError on NaN/inf."""
    return json.dumps(msg, separators=(",", ":"), allow_nan=False, default=_np)

from .gnc.planner import ExternalPlanner
from .simulator import Simulator


class BrowserSim:
    def __init__(self, post_plan, seed=None, guidance="scp_mpc", spawn="feasible"):
        ExternalPlanner.post = post_plan
        self.sim = Simulator(seed=seed, guidance=guidance, planner="external", spawn=spawn)
        self.seq = 0
        self.tick = 0
        self.perf = {"rtf": 0.0, "load": 0.0}
        self._acc_sim = self._acc_cpu = 0.0
        self._acc_t0 = time.perf_counter()

    def _state(self):
        m = self.sim.state_message(self.seq)
        m["perf"] = self.perf
        return m

    def hello(self) -> str:
        out = [dumps(self.sim.hello_message()), dumps(self._state())]
        plan = self.sim.latest_plan_message()
        if plan is not None:
            out.append(dumps(plan))
        return "\n".join(out)

    def handle(self, raw: str):
        try:
            msg = json.loads(raw)
        except (ValueError, TypeError):
            return
        if isinstance(msg, dict):
            try:
                self.sim.handle(msg)
            except Exception as e:          # malformed fields etc. - never kill the simulation
                print(f"[browser] ignored message {msg!r}: {e!r}")

    def plan_result(self, seq, data):
        p = ExternalPlanner.__dict__.get("active")
        if p is not None and getattr(self.sim.guidance, "planner", None) is p:
            p.deliver(int(seq), data)

    def plan_error(self, seq, text):
        p = ExternalPlanner.__dict__.get("active")
        if p is not None and getattr(self.sim.guidance, "planner", None) is p:
            p.fail(int(seq), text)

    def step(self, dt: float, budget: float) -> str:
        """Advance by the elapsed wall time ``dt`` (s), spending at most ``budget`` s of CPU.
        Returns the messages for the page, newline-separated JSON."""
        sim = self.sim
        t0 = time.perf_counter()
        t_sim0 = sim.t
        if not sim.paused:
            try:
                sim.advance(min(dt, 0.1) * sim.time_scale, max_wall=budget)
            except Exception as e:
                sim.event("error", f"simulation error: {e!r} - resetting")
                sim.reset()
        self._acc_sim += max(sim.t - t_sim0, 0.0)
        self._acc_cpu += time.perf_counter() - t0
        if t0 - self._acc_t0 > 1.0:
            wall = t0 - self._acc_t0
            self.perf = {"rtf": round(self._acc_sim / max(self._acc_cpu, 1e-6), 2),
                         "load": round(self._acc_cpu / wall, 3)}
            self._acc_sim = self._acc_cpu = 0.0
            self._acc_t0 = t0
        self.seq += 1
        out = []
        try:
            out.append(dumps(self._state()))
            if self.tick % AERO_EVERY == 0:
                out.append(dumps(sim.aero_message()))
        except ValueError as e:
            sim.event("warn", f"non-finite telemetry ({e}) - resetting")
            sim.reset()
            out = []
        for ev in sim.drain_messages():
            try:
                out.append(dumps(ev))
            except ValueError:
                pass
        self.tick += 1
        return "\n".join(out)
