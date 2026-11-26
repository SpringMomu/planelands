#!/usr/bin/env python
"""Space-shuttle landing simulator - entry point.

    python server.py [--http-port 8000] [--ws-port 8765] [--seed N] [--guidance scp_mpc]
                     [--spawn feasible|any]

* static HTTP server for ./web (threaded)       -> http://localhost:8000
* WebSocket simulation server (websockets 13)   -> ws://localhost:8765
* real-time 6-DOF simulation loop (asyncio), state broadcast ~60 Hz, aero ~20 Hz

See sim/ for the plant (aero, 6-DOF, gear), FCS and guidance interface.
"""
from __future__ import annotations

import os as _os
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    _os.environ.setdefault(_v, "1")   # small matrices: BLAS threads only cost memory/latency

import argparse
import asyncio
import json
import math
import os
import sys
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(ROOT, "web")
sys.path.insert(0, ROOT)

STATE_HZ = 60.0
AERO_EVERY = 3          # ticks -> 20 Hz


# --------------------------------------------------------------------------- HTTP
class Handler(SimpleHTTPRequestHandler):
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".html": "text/html; charset=utf-8",
        ".js": "text/javascript; charset=utf-8",
        ".mjs": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".json": "application/json",
        ".wasm": "application/wasm",
        ".glsl": "text/plain; charset=utf-8",
        ".svg": "image/svg+xml",
        ".png": "image/png",
        ".jpg": "image/jpeg",
        ".ico": "image/x-icon",
        ".bin": "application/octet-stream",
        ".f32": "application/octet-stream",
        ".u8": "application/octet-stream",
        ".u16": "application/octet-stream",
        "": "application/octet-stream",
    }
    ws_port = 8765

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_GET(self):
        if self.path.split("?")[0] == "/api/config":
            body = json.dumps({"ws_port": self.ws_port}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.split("?")[0] == "/py/sim.zip":        # dev: ?sim=browser runs the simulator in the page
            from tools.build_web import make_bundle
            body = make_bundle()
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def log_message(self, fmt, *args):   # quiet
        pass


def start_http(host, port, ws_port):
    os.makedirs(WEB, exist_ok=True)
    Handler.ws_port = ws_port
    ThreadingHTTPServer.request_queue_size = 128      # browsers open many parallel module requests; default backlog 5 refuses some on Windows
    httpd = ThreadingHTTPServer((host, port), partial(Handler, directory=WEB))
    httpd.daemon_threads = True
    th = threading.Thread(target=httpd.serve_forever, name="http", daemon=True)
    th.start()
    return httpd


# --------------------------------------------------------------------------- JSON
_FIXED = {"pos": 3, "vel": 4, "quat": 7, "omega": 6}


def _sig(x: float) -> float:
    if not math.isfinite(x):
        raise ValueError("non-finite")
    if x == 0.0:
        return 0.0
    return float(f"{x:.6g}")


def clean(o, key=None, nd=None):
    """Round floats for compact JSON; raise ValueError on NaN/inf."""
    if isinstance(o, (bool, np.bool_)):
        return bool(o)
    if isinstance(o, (int, np.integer)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        x = float(o)
        if not math.isfinite(x):
            raise ValueError(f"non-finite value in '{key}'")
        return round(x, nd) if nd is not None else _sig(x)
    if isinstance(o, dict):
        return {k: clean(v, k, _FIXED.get(k, nd)) for k, v in o.items()}
    if isinstance(o, (list, tuple, np.ndarray)):
        return [clean(v, key, nd) for v in o]
    return o


def dumps(msg):
    return json.dumps(clean(msg), separators=(",", ":"))


# --------------------------------------------------------------------------- app
class App:
    def __init__(self, sim):
        self.sim = sim
        self.clients = set()
        self.seq = 0
        self.perf = {"rtf": 0.0, "load": 0.0}
        self._hello = None

    def hello(self):
        if self._hello is None:
            self._hello = dumps(self.sim.hello_message())
        return self._hello

    async def handler(self, ws):
        self.clients.add(ws)
        peer = getattr(ws, "remote_address", None)
        print(f"[ws] client connected {peer} ({len(self.clients)} total)", flush=True)
        try:
            await ws.send(self.hello())
            await ws.send(dumps(self.state()))
            plan = self.sim.latest_plan_message()
            if plan is not None:
                await ws.send(dumps(plan))
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                if not isinstance(msg, dict):
                    continue
                try:
                    self.sim.handle(msg)
                except Exception as e:      # malformed fields etc. - never kill the server
                    print(f"[ws] ignored message {msg!r}: {e!r}", flush=True)
        except Exception:
            pass
        finally:
            self.clients.discard(ws)
            print(f"[ws] client disconnected {peer} ({len(self.clients)} left)", flush=True)

    def state(self):
        m = self.sim.state_message(self.seq)
        m["perf"] = self.perf
        return m

    def _send_all(self, text):
        from websockets.asyncio.server import broadcast
        broadcast(self.clients, text)

    async def loop(self):
        sim = self.sim
        period = 1.0 / STATE_HZ
        last = time.perf_counter()
        tick = 0
        acc_sim = acc_cpu = 0.0
        acc_wall_t0 = last
        deadline = time.perf_counter()
        while True:
            deadline += period
            t0 = time.perf_counter()
            dt = min(t0 - last, 0.1)
            last = t0
            t_sim0 = sim.t
            if not sim.paused:
                try:
                    sim.advance(dt * sim.time_scale, max_wall=0.75 * period)
                except Exception as e:
                    sim.event("error", f"simulation error: {e!r} - resetting")
                    sim.reset()
            cpu = time.perf_counter() - t0
            acc_sim += max(sim.t - t_sim0, 0.0)
            acc_cpu += cpu
            if t0 - acc_wall_t0 > 1.0:
                wall = t0 - acc_wall_t0
                self.perf = {"rtf": round(acc_sim / max(acc_cpu, 1e-6), 2),
                             "load": round(acc_cpu / wall, 3)}
                acc_sim = acc_cpu = 0.0
                acc_wall_t0 = t0
            self.seq += 1
            msgs = []
            try:
                msgs.append(dumps(self.state()))
                if tick % AERO_EVERY == 0 and self.clients:
                    msgs.append(dumps(sim.aero_message()))
            except ValueError as e:
                sim.event("warn", f"non-finite telemetry ({e}) - resetting")
                sim.reset()
                msgs = []
            for ev in sim.drain_messages():
                try:
                    msgs.append(dumps(ev))
                except ValueError:
                    pass
            if self.clients:
                for m in msgs:
                    self._send_all(m)
            tick += 1
            # deadline scheduling (Windows asyncio timers are coarse and may fire early)
            now = time.perf_counter()
            if now - deadline > 0.25:
                deadline = now                      # fell far behind: resynchronise
            yielded = False
            while True:
                rem = deadline - time.perf_counter()
                if rem <= 0.0015:
                    break
                await asyncio.sleep(rem)
                yielded = True
            if not yielded:
                # behind schedule: still give the event loop a slice for socket I/O and pings
                await asyncio.sleep(0.002)


async def main_async(args):
    from websockets.asyncio.server import serve

    from sim.simulator import Simulator
    sim = Simulator(seed=args.seed, guidance=args.guidance, planner="async", spawn=args.spawn)
    app = App(sim)
    async with serve(app.handler, args.host, args.ws_port, max_size=2 ** 20, compression=None,
                     ping_interval=20, ping_timeout=20):
        print(f"[ws] simulation server on ws://localhost:{args.ws_port}", flush=True)
        print(f"\n  >>> open  http://localhost:{args.http_port}  in a WebGL2 browser <<<\n", flush=True)
        try:
            await app.loop()
        finally:
            if hasattr(sim.guidance, "close"):
                sim.guidance.close()


def main():
    ap = argparse.ArgumentParser(description="Space-shuttle landing simulator server")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--guidance", default="scp_mpc", help="scp_mpc (default) or hold_glide")
    ap.add_argument("--spawn", default="feasible", choices=["feasible", "any"],
                    help="random spawns limited to the reachable energy band (default) or anywhere")
    ap.add_argument("--no-http", action="store_true", help="do not start the static file server")
    args = ap.parse_args()

    try:
        from worldgen.build import ensure_assets
    except ImportError:
        print("[assets] worldgen not available - skipping asset generation", flush=True)
    else:
        try:
            ensure_assets()
        except Exception as e:
            print(f"[assets] WARNING: asset generation failed: {e!r}", flush=True)

    if not args.no_http:
        start_http(args.host, args.http_port, args.ws_port)
        print(f"[http] serving {WEB} on http://localhost:{args.http_port}", flush=True)
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[server] stopped", flush=True)


if __name__ == "__main__":
    main()
