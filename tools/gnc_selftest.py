"""Headless success-rate test of the autonomous GNC (SCP + MPC + A/L) on the full 6-DOF plant.

    python tools/gnc_selftest.py                 # 24 random feasible spawns + 6 wind/turbulence cases
    python tools/gnc_selftest.py --n 40 --wind 10 --workers 12
    python tools/gnc_selftest.py --seeds 1003 1007   # specific seeds, verbose events

Success = touchdown on the runway with sink < 3 m/s, |cross| < 20 m, and the vehicle stops
on the runway (incl. overruns).  Each case runs in its own process (synchronous planner).
"""
from __future__ import annotations

import os as _os
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    _os.environ.setdefault(_v, "1")   # small matrices: BLAS threads only cost memory/latency

import argparse
import math
import multiprocessing as mp
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

R2D = 180.0 / math.pi
KT = 1.0 / 0.514444


def run_case(case):
    from sim.simulator import Simulator
    seed, ws, wd, turb, tag, verbose = case
    t_wall = time.perf_counter()
    sim = Simulator(seed=seed, verbose=False)
    sim.wind.set(speed=ws, dir_from=wd, intensity=turb)
    sim.drain_messages()                     # discard the init episode
    sim.reset(seed=seed)
    ev = []
    spawn = None
    qmax = nzmax = 0.0
    nzmin = 9.0
    tmax = 1500.0
    while sim.t < tmax:
        sim.step()
        if sim.status == "FLIGHT" and int(sim.t * 100) % 10 == 0:
            o = sim.obs
            qmax = max(qmax, o["qbar"])
            n = sim.last_Fb / (sim.geo.mass * 9.80665)
            nzmax = max(nzmax, -n[2])
            nzmin = min(nzmin, -n[2])
        for m in sim.drain_messages():
            if m.get("type") == "event":
                ev.append(f"[{sim.t:6.1f}] {m['level']}: {m['text']}")
                if spawn is None and m["text"].startswith("spawn"):
                    spawn = m["text"]
        if sim.status in ("STOPPED", "CRASHED"):
            break
    g = sim.guidance
    td = sim.touchdown or {}
    st = sim.stop_info or {}
    ok_td = (bool(td) and td.get("on_runway", False) and td["sink"] < 3.0 and abs(td.get("cross_rwy", 99)) < 20.0
             and td.get("past_thr", -1.0) >= 0.0)
    ok_stop = sim.status == "STOPPED" and st.get("on_runway", False)
    plans = getattr(g, "plan_log", [])
    cold = [p for p in plans if p["kind"] == "cold"]
    mstat = getattr(g, "mpc_stats", [0, 0.0, 0.0])
    res = dict(seed=seed, tag=tag, wind=(ws, wd, turb), status=sim.status, t=sim.t, success=ok_td and ok_stop,
               td=td, stop=st, qmax=qmax, nzmax=nzmax, nzmin=nzmin, spawn=spawn, events=ev,
               rwy=(plans[-1]["rwy"] if plans else None), n_plans=len(plans),
               scp_iter=[p["iterations"] for p in plans], scp_ms=[p["ms"] for p in plans],
               scp_conv=[p["converged"] for p in plans], scp_fb=[p["fallback"] for p in plans],
               cold_ms=[p["ms"] for p in cold], mpc_n=mstat[0], mpc_mean=mstat[1] / max(mstat[0], 1),
               mpc_max=mstat[2], wall=time.perf_counter() - t_wall)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--wind", type=int, default=6, help="number of 10 m/s wind + moderate turbulence cases")
    ap.add_argument("--wseed0", type=int, default=2000)
    ap.add_argument("--wind-speed", type=float, default=10.0)
    ap.add_argument("--turb", type=float, default=0.6)
    ap.add_argument("--workers", type=int, default=min(12, os.cpu_count() or 4))
    ap.add_argument("--seeds", type=int, nargs="*")
    args = ap.parse_args()
    cases = []
    if args.seeds:
        cases = [(s, 0.0, 0.0, 0.0, "calm", True) for s in args.seeds]
    else:
        cases = [(args.seed0 + i, 0.0, 0.0, 0.0, "calm", False) for i in range(args.n)]
        dirs = [0, 60, 120, 180, 240, 300, 330, 150]
        tag = f"wind{args.wind_speed:g}+turb{args.turb:g}"
        cases += [(args.wseed0 + i, args.wind_speed, float(dirs[i % len(dirs)]), args.turb, tag, False)
                  for i in range(args.wind)]
    t0 = time.perf_counter()
    ctx = mp.get_context("spawn")
    with ctx.Pool(processes=min(args.workers, len(cases))) as pool:
        results = []
        for r in pool.imap_unordered(run_case, cases):
            results.append(r)
            td = r["td"]
            tds = (f"td sink {td['sink']:.2f} m/s {td['gs']*KT:.0f} KGS {td.get('past_thr', 0):.0f} m past thr "
                   f"cross {td.get('cross_rwy', 0):+.1f} m" if td else "no touchdown")
            print(f"{'OK ' if r['success'] else 'FAIL'} seed {r['seed']} [{r['tag']}] RWY{r['rwy']} {r['status']} "
                  f"t={r['t']:.0f}s {tds} | qmax {r['qmax']/1000:.1f} kPa nz {r['nzmin']:.2f}..{r['nzmax']:.2f} | "
                  f"plans {r['n_plans']} | wall {r['wall']:.0f}s", flush=True)
            print(f"      {r['spawn']}", flush=True)
            if not r["success"] or args.seeds:
                for e in r["events"][-14:]:
                    print("        " + e)
    results.sort(key=lambda r: (r["tag"], r["seed"]))
    print("\n" + "=" * 100)
    for tag in sorted({r["tag"] for r in results}):
        rs = [r for r in results if r["tag"] == tag]
        succ = [r for r in rs if r["success"]]
        tds = [r["td"] for r in rs if r["td"]]
        print(f"[{tag}] success {len(succ)}/{len(rs)} = {100.0*len(succ)/len(rs):.0f} %   "
              f"(RWY33 {sum(r['rwy']=='33' for r in rs)}, RWY15 {sum(r['rwy']=='15' for r in rs)})")
        if tds:
            sink = np.array([t["sink"] for t in tds])
            kgs = np.array([t["gs"] * KT for t in tds])
            past = np.array([t.get("past_thr", 0) for t in tds])
            cr = np.array([abs(t.get("cross_rwy", 0)) for t in tds])
            print(f"   touchdown: sink {sink.mean():.2f} avg / {sink.max():.2f} max m/s, {kgs.mean():.0f} KGS avg "
                  f"({kgs.min():.0f}..{kgs.max():.0f}), past threshold {past.mean():.0f} m ({past.min():.0f}..{past.max():.0f}),"
                  f" |cross| {cr.mean():.1f} avg / {cr.max():.1f} max m")
        print(f"   max qbar {max(r['qmax'] for r in rs)/1000:.1f} kPa, nz range {min(r['nzmin'] for r in rs):.2f}.."
              f"{max(r['nzmax'] for r in rs):.2f} g")
        it = np.concatenate([r["scp_iter"] for r in rs]) if rs else np.array([])
        ms = np.concatenate([r["scp_ms"] for r in rs]) if rs else np.array([])
        cms = np.concatenate([r["cold_ms"] for r in rs]) if rs else np.array([])
        conv = np.concatenate([r["scp_conv"] for r in rs]) if rs else np.array([])
        fb = np.concatenate([r["scp_fb"] for r in rs]) if rs else np.array([])
        if len(it):
            print(f"   SCP: {len(it)} solves, iterations {it.mean():.1f} avg / {it.max()} max, converged "
                  f"{100*conv.mean():.0f} %, fallback {100*fb.mean():.0f} %, time {ms.mean():.0f} avg / {ms.max():.0f} max ms"
                  f" (cold starts {cms.mean():.0f} avg ms)")
        print(f"   MPC: {sum(r['mpc_n'] for r in rs)} solves, {np.mean([r['mpc_mean'] for r in rs]):.1f} ms avg, "
              f"{max(r['mpc_max'] for r in rs):.1f} ms max")
    allr = [r for r in results]
    print(f"\nOVERALL success {sum(r['success'] for r in allr)}/{len(allr)} "
          f"({100.0*sum(r['success'] for r in allr)/len(allr):.0f} %), wall {time.perf_counter()-t0:.0f} s")


if __name__ == "__main__":
    main()
