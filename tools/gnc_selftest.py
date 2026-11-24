"""Headless success-rate test of the autonomous GNC (SCP + MPC + A/L) on the full 6-DOF plant.

    python tools/gnc_selftest.py                 # 24 random feasible spawns + 6 wind/turbulence cases
    python tools/gnc_selftest.py --n 40 --wind 10 --workers 12
    python tools/gnc_selftest.py --seeds 1003 1007   # specific seeds, verbose events
    python tools/gnc_selftest.py --n 0 --wind 0 --buran 16   # forecast-vs-actual wind (HAC switch)
    python tools/gnc_selftest.py --at 40,200,15 --at 25,90,10,270   # user-placed starts
    python tools/gnc_selftest.py --at 20,150,4 --at-wind 10,330      # ... with wind (10 m speed, from)
    GNC_AUTO_WEATHER=1 python tools/gnc_selftest.py ...              # drifting wind like the web client
        (dist_km,bearing_deg,alt_km[,heading_deg[,mach]]; bearing from the runway centre,
         heading omitted = toward the runway, Mach omitted = ~10 kPa dynamic pressure)

"buran" cases: the vehicle spawns and plans with a light forecast wind; at ``--buran-t`` s the
actual wind ramps (over ``--buran-ramp`` s) to ``--buran-speed`` m/s from a swept direction, like
the much stronger than forecast wind at Buran's landing.  Reports how often the guidance
re-selects the approach (runway end / HAC side) and whether it still lands.

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
    seed, ws, wd, turb, tag, verbose = case[:6]
    sched = case[6] if len(case) > 6 else None      # (t_change, speed, dir_from, ramp_s, ambiguity)
    at = case[7] if len(case) > 7 else None         # custom spawn spec (Simulator._custom_init)
    t_wall = time.perf_counter()
    sim = Simulator(seed=seed, verbose=False)
    sim.wind.set(speed=ws, dir_from=wd, intensity=turb, auto=bool(os.environ.get("GNC_AUTO_WEATHER")))
    sim.drain_messages()                     # discard the init episode
    if sched is not None and len(sched) > 4 and sched[4] > 0:
        # spawn near a decision boundary: the best two approaches of different runway end / HAC
        # side score within sched[4] of each other under the forecast wind
        from sim.gnc.geometry import ApproachConfig, candidates, plan_score
        base = sim._spawn_feasible

        def ambiguous(x3):
            if not base(x3):
                return False
            feas = sorted((c for c in candidates(sim._spawn_pm, x3, sim._spawn_ends, ApproachConfig(), 0)
                           if c.feasible), key=plan_score)
            if not feas:
                return False
            k0 = (feas[0].rwy.name, feas[0].side)
            other = [c for c in feas if (c.rwy.name, c.side) != k0]
            return bool(other) and plan_score(other[0]) - plan_score(feas[0]) < sched[4]
        sim._spawn_feasible = ambiguous
    ev = []
    if at is not None:
        init, warn = sim._custom_init(at)
        sim.reset(init=init)
        ev.extend(f"[   0.0] warn: custom spawn: {w}" for w in warn)
    else:
        sim.reset(seed=seed)
    g0 = sim.guidance
    spawn = None
    qmax = nzmax = 0.0
    nzmin = 9.0
    tmax = 1500.0
    wchg = None
    while sim.t < tmax:
        if sched is not None and sim.t >= sched[0] and sim.status == "FLIGHT":
            # vector ramp forecast -> actual wind
            f = min((sim.t - sched[0]) / max(sched[3], 1e-6), 1.0)
            a0, a1 = math.radians(wd), math.radians(sched[2])
            vx = (1 - f) * ws * math.sin(a0) + f * sched[1] * math.sin(a1)
            vy = (1 - f) * ws * math.cos(a0) + f * sched[1] * math.cos(a1)
            if f < 1.0 or wchg is None:
                sim.wind.set(speed=math.hypot(vx, vy), dir_from=math.degrees(math.atan2(vx, vy)) % 360.0)
                if wchg is None:
                    wchg = dict(rwy0=(g0.plan or {}).get("rwy"), side0=g0.plan_log[-1]["hac_dir"] if g0.plan_log else 0)
                    ev.append(f"[{sim.t:6.1f}] test: actual wind ramps to {sched[1]:.0f} m/s from {sched[2]:03.0f}")
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
                if spawn is None and m["text"].startswith(("spawn:", "custom spawn:")):
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
               mpc_max=mstat[2], wall=time.perf_counter() - t_wall,
               switches=list(getattr(g, "switch_log", [])), wchg=wchg)
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
    ap.add_argument("--at", action="append", default=[],
                    help="custom start dist_km,bearing_deg,alt_km[,heading_deg[,mach]] (repeatable)")
    ap.add_argument("--at-wind", default="0,0,0",
                    help="wind for the --at cases: speed_10m,dir_from[,turbulence] (default calm)")
    ap.add_argument("--buran", type=int, default=0, help="number of forecast-vs-actual wind cases")
    ap.add_argument("--bseed0", type=int, default=3000)
    ap.add_argument("--buran-speed", type=float, default=12.0, help="actual 10 m wind (m/s; ~1.75x aloft)")
    ap.add_argument("--buran-forecast", type=float, default=3.0, help="forecast 10 m wind (m/s)")
    ap.add_argument("--buran-t", type=float, default=40.0, help="time the actual wind sets in (s)")
    ap.add_argument("--buran-ramp", type=float, default=20.0)
    ap.add_argument("--buran-turb", type=float, default=0.3)
    ap.add_argument("--buran-ambig", type=float, default=0.12,
                    help="spawn only where the two best approach classes score within this (0: any spawn)")
    args = ap.parse_args()
    cases = []
    if args.at:
        for i, a in enumerate(args.at):
            v = [float(x) for x in a.split(",")]
            sp = dict(dist_km=v[0], bearing_deg=v[1], alt_km=v[2])
            if len(v) > 3:
                sp["heading_deg"] = v[3]
            if len(v) > 4:
                sp.update(speed=v[4], speed_unit="mach")
            aw = [float(x) for x in args.at_wind.split(",")] + [0.0, 0.0, 0.0]
            cases.append((i, aw[0], aw[1], aw[2], f"at {a}", True, None, sp))
    elif args.seeds:
        cases = [(s, 0.0, 0.0, 0.0, "calm", True) for s in args.seeds]
    else:
        cases = [(args.seed0 + i, 0.0, 0.0, 0.0, "calm", False) for i in range(args.n)]
        dirs = [0, 60, 120, 180, 240, 300, 330, 150]
        tag = f"wind{args.wind_speed:g}+turb{args.turb:g}"
        cases += [(args.wseed0 + i, args.wind_speed, float(dirs[i % len(dirs)]), args.turb, tag, False)
                  for i in range(args.wind)]
        btag = f"buran{args.buran_forecast:g}->{args.buran_speed:g}"
        for i in range(args.buran):
            wd_act = (i * 360.0 / 8 + 22.5 * (i // 8)) % 360.0
            wd_fc = (wd_act + 90.0) % 360.0       # forecast direction unrelated to the actual one
            cases.append((args.bseed0 + i, args.buran_forecast, wd_fc, args.buran_turb, btag, False,
                          (args.buran_t, args.buran_speed, wd_act, args.buran_ramp, args.buran_ambig)))
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
            for sw in r["switches"]:
                print(f"      HAC switch at t={sw['t']:.0f}s h={sw['h']/1000:.1f} km: {sw['frm']} -> {sw['to']} "
                      f"(band {sw['cur_band']:.2f} -> {sw['new_band']:.2f}; {sw['reason']})", flush=True)
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
        nsw = [r for r in rs if r["switches"]]
        if nsw or tag.startswith("buran"):
            print(f"   approach re-selection: {len(nsw)}/{len(rs)} flights switched "
                  f"({sum(len(r['switches']) for r in rs)} switches), {sum(r['success'] for r in nsw)}/{len(nsw)} "
                  f"of those landed")
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
