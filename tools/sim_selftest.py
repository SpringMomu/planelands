"""Headless self-tests of the 6-DOF plant + FCS + placeholder guidance.

    python tools/sim_selftest.py          # all tests
    python tools/sim_selftest.py fcs      # only the FCS step responses
"""
from __future__ import annotations

import os as _os
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    _os.environ.setdefault(_v, "1")   # small matrices: BLAS threads only cost memory/latency

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim.atmosphere import atmosphere  # noqa: E402
from sim.dynamics import R_to_quat  # noqa: E402
from sim.guidance import GuidanceBase, GuidanceCommand, register  # noqa: E402
from sim.simulator import Simulator  # noqa: E402
from sim.spawn import InitialState, approach_state, attitude_from_flight  # noqa: E402

R2D = 180.0 / math.pi


@register
class ScriptedGuidance(GuidanceBase):
    """Test guidance: returns whatever is in self.c (alpha, bank, sb)."""
    name = "test_scripted"
    rate_hz = 50.0

    def reset(self, s):
        self.c = dict(alpha=s.alpha, bank=0.0, sb=0.0)

    def update(self, t, s):
        return GuidanceCommand(alpha_deg=self.c["alpha"], bank_deg=self.c["bank"],
                               speedbrake_deg=self.c["sb"], info="scripted")


def make_state(sim, h, M, gamma_deg=-6.0, hdg_deg=0.0, E=30000.0, N=0.0):
    T, p, rho, a, mu = atmosphere(h)
    V = M * a
    q = 0.5 * rho * V * V
    gam = math.radians(gamma_deg)
    cl = sim.geo.mass * 9.80665 * math.cos(gam) / (q * sim.geo.S)
    alpha = sim.tables.alpha_for_cl(cl, M)
    de, bf = sim.tables.trim_surfaces(alpha, M)
    hdg = math.radians(hdg_deg)
    R = attitude_from_flight(hdg, gam, math.radians(alpha))
    v = V * np.array([math.cos(gam) * math.sin(hdg), math.cos(gam) * math.cos(hdg), math.sin(gam)])
    return InitialState(pos=np.array([E, N, h]), vel=v, quat=R_to_quat(R), omega=np.zeros(3),
                        alpha_deg=alpha, de_deg=de, bf_deg=bf,
                        desc=f"test state h={h:.0f} M={M:.2f} q={q/1000:.1f} kPa alpha={alpha:.1f}")


def run(sim, T, log_dt=0.1, stop_on_crash=True):
    rec = []
    n = int(round(T / 0.01))
    k_log = max(1, int(round(log_dt / 0.01)))
    for k in range(n):
        sim.step()
        if k % k_log == 0:
            m = sim.state_message()
            rec.append((sim.t, m))
        if stop_on_crash and sim.status in ("CRASHED", "STOPPED"):
            break
    return rec


def step_metrics(ts, ys, y0, y1, t0):
    """rise time 10-90 %, overshoot %, settling (5 %) after t0."""
    ts, ys = np.asarray(ts), np.asarray(ys)
    m = ts >= t0
    ts, ys = ts[m] - t0, ys[m]
    span = y1 - y0
    nrm = (ys - y0) / span
    try:
        t10 = ts[np.argmax(nrm >= 0.1)]
        t90 = ts[np.argmax(nrm >= 0.9)]
        rise = t90 - t10 if (nrm >= 0.9).any() else float("nan")
    except ValueError:
        rise = float("nan")
    over = max(0.0, (nrm.max() - 1.0) * 100.0)
    out = np.where(np.abs(nrm - 1.0) > 0.05)[0]
    settle = ts[out[-1]] if len(out) and out[-1] < len(ts) - 1 else (float("nan") if len(out) else 0.0)
    return rise, over, settle


def test_fcs():
    print("\n=== FCS step responses ===")
    ok = True
    for (h, M, label) in ((5000.0, 0.62, "high q (5 km, M0.62)"), (24000.0, 2.3, "low q / supersonic (24 km, M2.3)"),
                          (12000.0, 0.95, "transonic (12 km, M0.95)")):
        sim = Simulator(seed=1, guidance="test_scripted", verbose=False, spawn="any")
        sim.reset(init=make_state(sim, h, M))
        g = sim.guidance
        run(sim, 3.0)
        a0 = sim.obs["alpha"] * R2D
        g.c["alpha"] = a0
        g.c["bank"] = 45.0
        rec = run(sim, 30.0 if M > 1.5 else 12.0, log_dt=0.02)
        ts = [r[0] for r in rec]
        bank = [r[1]["air"]["bank"] for r in rec]
        beta = [r[1]["air"]["beta"] for r in rec]
        alph = [r[1]["air"]["alpha"] for r in rec]
        rise, over, settle = step_metrics(ts, bank, 0.0, 45.0, ts[0])
        bmax = max(abs(b) for b in beta)
        aerr = max(abs(a - a0) for a in alph)
        trk0 = rec[0][1]["air"]["track"]
        trk1 = rec[-1][1]["air"]["track"]
        print(f"  {label}: bank 0->45: rise {rise:.2f} s, overshoot {over:.1f} %, settle {settle:.1f} s, "
              f"|beta|max {bmax:.2f} deg, alpha dev {aerr:.2f} deg, track change {((trk1-trk0+540)%360-180):+.1f} deg")
        ok &= over < 15.0 and bmax < 2.0 and rise < (25.0 if M > 1.5 else 8.0)
        # alpha step +3 deg wings level
        g.c["bank"] = 0.0
        run(sim, 10.0)
        a0 = sim.obs["alpha"] * R2D
        # supersonic case steps DOWN: +3 deg would exceed the alpha envelope protection there
        da = 3.0 if M < 1.5 else -3.0
        g.c["alpha"] = a0 + da
        rec = run(sim, 8.0, log_dt=0.02)
        ts = [r[0] for r in rec]
        alph = [r[1]["air"]["alpha"] for r in rec]
        nz = [r[1]["air"]["nz"] for r in rec]
        rise, over, settle = step_metrics(ts, alph, a0, a0 + da, ts[0])
        print(f"  {label}: alpha {a0:.1f}->{a0+da:.1f}: rise {rise:.2f} s, overshoot {over:.1f} %, "
              f"settle {settle:.1f} s, nz max {max(nz):.2f} g, final err {alph[-1]-a0-da:+.2f} deg")
        ok &= over < 20.0 and abs(alph[-1] - a0 - da) < 0.3
    print("  FCS:", "PASS" if ok else "CHECK")
    return ok


def test_spawns(n=5, T=150.0):
    print(f"\n=== {n} random spawns, AUTO placeholder, {T:.0f} s ===")
    ok = True
    for seed in range(100, 100 + n):
        sim = Simulator(seed=seed, verbose=False, guidance="hold_glide", spawn="any")
        print("  " + sim.drain_events()[0]["text"])
        rec = run(sim, T, log_dt=0.1)
        ae = [abs(r[1]["air"]["alpha"] - r[1]["cmd"]["alpha"]) for r in rec if r[0] > 5.0 and r[1]["status"] == "FLIGHT"]
        be = [abs(r[1]["air"]["beta"]) for r in rec if r[0] > 1.0 and r[1]["status"] == "FLIGHT"]
        mu = [abs(r[1]["air"]["bank"]) for r in rec if r[1]["status"] == "FLIGHT"]
        fin = all(np.isfinite(r[1]["pos"]).all() for r in rec)
        last = rec[-1][1]
        a = last["air"]
        ev = [e["text"] for e in sim.drain_events()]
        print(f"    -> t={sim.t:5.1f} s status {last['status']}, alt {a['alt']:.0f} m, M {a['mach']:.2f}, "
              f"alpha err rms {np.sqrt(np.mean(np.square(ae))) if ae else 0:.3f} max {max(ae) if ae else 0:.2f} deg, "
              f"|beta|max {max(be) if be else 0:.3f}, |bank|max {max(mu) if mu else 0:.2f}, finite={fin}")
        for e in ev:
            print("       event:", e)
        ok &= fin and (max(ae) if ae else 0) < 1.5
    print("  spawns:", "PASS" if ok else "CHECK")
    return ok


def test_manual():
    print("\n=== manual (CSS) inputs ===")
    sim = Simulator(seed=7, verbose=False, guidance="hold_glide", spawn="any")
    sim.reset(init=make_state(sim, 8000.0, 0.7))
    sim.set_mode("MANUAL")
    run(sim, 2.0)
    a0 = sim.obs["alpha"] * R2D
    sim.set_control({"roll": 0.6})
    run(sim, 3.0)
    sim.set_control({"roll": 0.0})
    rec = run(sim, 6.0)
    b_hold = [r[1]["air"]["bank"] for r in rec[-20:]]
    sim.set_control({"pitch": 0.5})
    run(sim, 2.0)
    sim.set_control({"pitch": 0.0})
    rec2 = run(sim, 6.0)
    a_hold = rec2[-1][1]["air"]["alpha"]
    sim.set_control({"roll": -0.6})
    run(sim, 3.0)
    sim.set_control({"roll": 0.0, "speedbrake": 1.0})
    rec3 = run(sim, 15.0)
    sb = rec3[-1][1]["surf"]["speedbrake"]
    print(f"  roll stick 0.6 x 3 s -> bank holds {np.mean(b_hold):.1f} deg (expect ~27, spread {np.ptp(b_hold):.2f})")
    print(f"  pitch stick 0.5 x 2 s -> alpha {a0:.1f} -> {a_hold:.1f} deg (expect +5)")
    print(f"  roll back -> bank {rec3[-1][1]['air']['bank']:.1f} deg, speedbrake {sb:.1f} deg, status {rec3[-1][1]['status']}")
    ok = abs(np.mean(b_hold) - 27.0) < 4.0 and abs(a_hold - a0 - 5.0) < 1.0 and sb > 90.0
    print("  manual:", "PASS" if ok else "CHECK")
    return ok


def test_landing():
    print("\n=== touchdown + rollout (RWY33, 30 m AGL, 115 m/s -> touchdown ~105 m/s) ===")
    sim = Simulator(seed=5, verbose=False, guidance="hold_glide", spawn="any")
    sim.reset(init=approach_state(sim.terrain, sim.tables, height_agl=30.0, V=115.0, gamma_deg=-2.0))
    sim.drain_events()
    t0 = time.perf_counter()
    rec = run(sim, 120.0, log_dt=0.1)
    wall = time.perf_counter() - t0
    for e in sim.drain_events():
        print("  event:", e["text"])
    last = sim.state_message()
    td = sim.touchdown
    print(f"  final status {last['status']} at t={sim.t:.1f} s, along {last['rwy']['along']:.0f} m, "
          f"cross {last['rwy']['cross']:+.1f} m, wall {wall:.1f} s (RTF {sim.t/wall:.1f}x near ground)")
    nzs = [r[1]["air"]["nz"] for r in rec]
    print(f"  max nz {max(nzs):.2f} g, max |bank| {max(abs(r[1]['air']['phi']) for r in rec):.2f} deg")
    ok = (td is not None and td["sink"] < 3.0 and last["status"] == "STOPPED"
          and abs(last["rwy"]["cross"]) < 20.0)
    print("  landing:", "PASS" if ok else "CHECK")
    return ok


def test_turbulence():
    print("\n=== turbulence + crosswind ===")
    sim = Simulator(seed=21, verbose=False, guidance="hold_glide", spawn="any")
    sim.reset(init=make_state(sim, 4000.0, 0.5))
    sim.wind.set(speed=10.0, dir_from=300.0, intensity=0.6)
    rec = [r for r in run(sim, 65.0, log_dt=0.05) if r[0] > 5.0]   # skip the wind switch-on transient
    al = np.array([r[1]["air"]["alpha"] for r in rec])
    be = np.array([r[1]["air"]["beta"] for r in rec])
    mu = np.array([r[1]["air"]["bank"] for r in rec])
    nz = np.array([r[1]["air"]["nz"] for r in rec])
    print(f"  4 km, moderate turbulence 60 s: alpha std {al.std():.2f} deg, |beta|max {np.abs(be).max():.2f}, "
          f"|bank|max {np.abs(mu).max():.2f}, nz {nz.min():.2f}..{nz.max():.2f} g, status {sim.status}")
    ok1 = sim.status == "FLIGHT" and np.abs(be).max() < 3.0 and np.abs(mu).max() < 8.0
    sim = Simulator(seed=22, verbose=False, guidance="hold_glide", spawn="any")
    sim.wind.set(speed=6.0, dir_from=300.0, intensity=0.3)
    sim.reset(init=approach_state(sim.terrain, sim.tables, height_agl=30.0, V=115.0, gamma_deg=-2.0))
    sim.drain_events()
    run(sim, 120.0, log_dt=0.1)
    for e in sim.drain_events():
        print("  event:", e["text"])
    m = sim.state_message()
    print(f"  crosswind landing (6 m/s from 300, light turbulence): status {m['status']}, "
          f"cross {m['rwy']['cross']:+.1f} m")
    ok2 = m["status"] == "STOPPED" and abs(m["rwy"]["cross"]) < 25.0
    print("  turbulence:", "PASS" if ok1 and ok2 else "CHECK")
    return ok1 and ok2


def test_reduced_model():
    print("\n=== reduced 3-DOF model vs 6-DOF plant (30 s, alpha hold, wings level) ===")
    import math as _m
    from sim.atmosphere import atmosphere as _atm
    ok = True
    for (h, M) in ((9000.0, 0.7), (20000.0, 1.8)):
        sim = Simulator(seed=3, guidance="test_scripted", verbose=False, spawn="any")
        sim.reset(init=make_state(sim, h, M))
        run(sim, 2.0)
        sim.guidance.c["alpha"] = sim.obs["alpha"] * R2D
        o = sim.obs
        x0 = np.array([sim.x[0], sim.x[1], sim.x[2], o["V"], o["gamma"], _m.atan2(sim.x[3], sim.x[4])])
        u = np.array([sim.obs["alpha"] * R2D, 0.0, 0.0])
        x = x0.copy()
        for _ in range(300):
            x = sim.model.step(x, u, 0.1)
        run(sim, 30.0)
        o = sim.obs
        dh = x[2] - sim.x[2]
        dV = x[3] - o["V"]
        dg = _m.degrees(x[4] - o["gamma"])
        dpos = _m.hypot(x[0] - sim.x[0], x[1] - sim.x[1])
        A, B = sim.model.linearize(x0, u)
        print(f"  h={h/1000:.0f} km M={M}: after 30 s  dh={dh:+.0f} m  dV={dV:+.1f} m/s  dgamma={dg:+.2f} deg  "
              f"dpos={dpos:.0f} m   (dVdot/dalpha={B[3,0]:+.3f} m/s^2/deg, dgamdot/dalpha={_m.degrees(B[4,0]):+.3f} deg/s/deg)")
        ok &= abs(dh) < 300 and abs(dV) < 10
    print("  reduced model:", "PASS" if ok else "CHECK")
    return ok


def test_rtf():
    print("\n=== real-time factor ===")
    sim = Simulator(seed=11, verbose=False, guidance="hold_glide", spawn="any")
    sim.reset(init=make_state(sim, 15000.0, 1.4))
    t0 = time.perf_counter()
    run(sim, 30.0, log_dt=1.0 / 60.0)
    w = time.perf_counter() - t0
    print(f"  flight (incl. state messages at 60 Hz): {30.0/w:.1f}x real time")
    sim.reset(init=approach_state(sim.terrain, sim.tables, height_agl=4.0, V=95.0, gamma_deg=-1.0))
    t0 = time.perf_counter()
    run(sim, 10.0, log_dt=1.0 / 60.0, stop_on_crash=False)
    w = time.perf_counter() - t0
    print(f"  ground contact (400 Hz sub-steps): {10.0/w:.1f}x real time")


if __name__ == "__main__":
    which = sys.argv[1:] or ["fcs", "spawns", "manual", "landing", "turbulence", "reduced", "rtf"]
    res = {}
    if "fcs" in which:
        res["fcs"] = test_fcs()
    if "spawns" in which:
        res["spawns"] = test_spawns()
    if "manual" in which:
        res["manual"] = test_manual()
    if "landing" in which:
        res["landing"] = test_landing()
    if "turbulence" in which:
        res["turbulence"] = test_turbulence()
    if "reduced" in which:
        res["reduced"] = test_reduced_model()
    if "rtf" in which:
        test_rtf()
    print("\nRESULTS:", res)
