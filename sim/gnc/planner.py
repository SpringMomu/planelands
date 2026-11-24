"""Planner front-end: runway / HAC selection + geometric guess + SCP, either synchronous
(headless tests) or in a worker process (real-time server; no GIL stalls).

``plan(req)`` is a pure function of a picklable request dict and returns a picklable dict.
"""
from __future__ import annotations

import math
import multiprocessing as mp
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict

import numpy as np

from .approach import ALRef, build_reference
from .geometry import (BAND_TARGET, TWO_PI, ApproachConfig, candidates, initial_guess, plan_score, rate_track,
                       runway_ends, select_plan, stretched_pick)
from .model import G0, PlanningModel
from .scp import DX, ScpSolver, TerrainField

_CTX = None


class _Ctx:
    def __init__(self, tables=None, mass=None, S=None, world=None, terrain=None):
        if tables is None:
            from ..aero import AeroModel
            from ..geometry import build_geometry
            from ..reduced_model import TrimTables
            from ..terrain import Terrain
            from ..trim import load_or_build_trim_tables
            geo = build_geometry()
            tables = TrimTables(load_or_build_trim_tables(AeroModel(geo), verbose=False))
            mass, S, world = geo.mass, geo.S, geo.world
            terrain = Terrain(world, verbose=False)
        self.world = world
        self.ends = runway_ends(world)
        self.model = PlanningModel(tables, mass, S, ground_elev=world["runway"]["elevation"])
        self.tfield = TerrainField(terrain) if terrain is not None else None
        self.solver = ScpSolver(self.model, self.tfield)


def init_worker():
    global _CTX
    if _CTX is None:
        _CTX = _Ctx()


def set_context(ctx: _Ctx):
    global _CTX
    _CTX = ctx


def _nodes_for(tf):
    return int(np.clip(5 * round(tf / 7.5 / 5.0), 20, 45))


def _resample(t, X, U, N):
    tn = np.linspace(t[0], t[-1], N)
    Xn = np.column_stack([np.interp(tn, t, X[:, j]) for j in range(6)])
    Un = np.column_stack([np.interp(tn, t, U[:, j]) for j in range(3)])
    return Xn, Un, float(t[-1] - t[0])


def _al_dict(al: ALRef):
    return dict(t=al.t, X=al.X, U=al.U, gear=al.gear, x=al.x, ph=al.ph, keys=al.keys, sb_pf=al.sb_pf,
                cfg=asdict(al.cfg), rwy=al.rwy.name)


def al_from_dict(d, ends):
    return ALRef(rwy=ends[d["rwy"]], cfg=ApproachConfig(**d["cfg"]), t=np.asarray(d["t"]), X=np.asarray(d["X"]),
                 U=np.asarray(d["U"]), gear=np.asarray(d["gear"]), x=np.asarray(d["x"]), ph=np.asarray(d["ph"]),
                 keys=d["keys"], sb_pf=d["sb_pf"])


# re-selection of the discrete approach (runway end / HAC side) on warm replans
SWITCH_GAIN = 0.12       # score improvement needed to leave a still-feasible plan (hysteresis)
SWITCH_GAIN_BAD = 0.02   # ... when the current plan has left the energy band


def _side_name(rwy, side):
    return f"RWY{rwy} {'right' if side > 0 else 'left'} HAC"


def _reselect(pm, ctx, x0, Xw, rwy, cfg, hac):
    """Re-run the runway / HAC-side selection from the current state with the current wind and
    compare it with the plan being flown (``Xw``: its remaining TAEM nodes, starting at x0).
    Returns (candidate or None, decision dict)."""
    try:
        cur = rate_track(pm, x0, Xw, rwy, cfg, hac)
    except Exception as e:                       # degenerate remaining track: keep the plan
        return None, dict(error=repr(e))
    best, ok, _ = select_plan(pm, x0, ctx.ends, ApproachConfig())
    s_cur, s_new = plan_score(cur), plan_score(best)
    dec = dict(cur_rwy=rwy.name, cur_side=int(hac["dir"]) if hac else 0, cur_band=float(cur.band_pos),
               cur_feasible=bool(cur.feasible), cur_score=float(s_cur), new_rwy=best.rwy.name,
               new_side=int(best.side), new_band=float(best.band_pos), new_loops=int(best.path.loops),
               new_score=float(s_new), new_ok=bool(ok), switch=False)
    same = hac is not None and best.rwy.name == rwy.name and best.side == int(hac["dir"])
    if not ok or same:
        return None, dec
    if cur.feasible and best.level > 0:
        return None, dec          # never trade a flyable plan for a shortened (low-energy) approach
    gain = s_cur - s_new
    if gain > (SWITCH_GAIN_BAD if not cur.feasible else SWITCH_GAIN):
        dec["reason"] = ("current plan out of the energy band" if not cur.feasible
                         else f"better energy margin (score {s_cur:.2f} -> {s_new:.2f})")
        return best, dec
    return None, dec


def plan(req: dict) -> dict:
    """Plan from req['x0'] (3-DOF state) to the A/L interface.

    req keys: x0, u0, t0, wind=(speed, dir_from), warm (optional previous TAEM reference
    from now: dict(rwy, cfg, hac, t, X, U, reachable)), prefer (runway name or None),
    reselect (bool: with ``warm``, re-evaluate runway end / HAC side and switch if clearly better)."""
    t_start = time.perf_counter()
    init_worker()
    ctx = _CTX
    pm = ctx.model
    pm.set_wind(*req.get("wind", (0.0, 0.0)))
    x0 = np.asarray(req["x0"], float)
    u0 = np.asarray(req["u0"], float)
    warm = req.get("warm")
    note = ""
    fin = _final_approach(ctx, pm, x0)
    if fin is not None:
        fin["t0"] = req.get("t0", 0.0)
        fin["episode"] = req.get("episode")
        fin["wind"] = req.get("wind", (0.0, 0.0))
        return fin
    kind = None
    if warm is not None and warm["rwy"] in ctx.ends and warm["t"][-1] - warm["t"][0] > 25.0:
        rwy = ctx.ends[warm["rwy"]]
        cfg = ApproachConfig(**warm["cfg"])
        hac = warm.get("hac")
        reachable = bool(warm.get("reachable", True))
        tw = np.asarray(warm["t"], float)
        N = _nodes_for(tw[-1] - tw[0])
        Xg, Ug, tfg = _resample(tw, np.asarray(warm["X"]), np.asarray(warm["U"]), N)
        Xg[0] = x0
        Xg[0, 5] = x0[5]
        kind = "warm"
    decision = None
    switch = None
    if kind == "warm" and req.get("reselect"):
        switch, decision = _reselect(pm, ctx, x0, Xg, rwy, cfg, hac)
    energy_cls = None              # which way the energy is off (select_plan), for the degraded modes
    if req.get("fault_scp"):
        return _fault_result(req, pm, ctx, x0, t_start)
    if kind != "warm":
        cand, reachable, note = select_plan(pm, x0, ctx.ends, ApproachConfig(), prefer=req.get("prefer"))
        energy_cls = cand.energy
        rwy, cfg, hac = cand.rwy, cand.cfg, cand.hac()
        Xg0, Ug0, tf0, _ = initial_guess(pm, x0, cand, 40)
        N = _nodes_for(tf0)
        if N != 40:
            Xg, Ug, tfg = _resample(np.linspace(0, tf0, 40), Xg0, Ug0, N)
        else:
            Xg, Ug, tfg = Xg0, Ug0, tf0
        kind = "cold"
    def attempt(rwy, cfg, Xg, Ug, tfg, iters):
        al = build_reference(pm, rwy, cfg)
        xf = al.X[0].copy()
        xf[5] += round((Xg[-1, 5] - xf[5]) / TWO_PI) * TWO_PI
        Xg = Xg.copy()
        Xg[-1, 5] = xf[5]
        res = ctx.solver.solve(x0, u0, xf, Xg, Ug, tfg, max_iter=iters)
        te = res.term_err
        good = (np.all(np.isfinite(res.X)) and res.defect < 0.02 and abs(te[0]) < 300 and abs(te[1]) < 300
                and abs(te[2]) < 150 and abs(te[3]) < 8 and abs(te[4]) < math.radians(3)
                and abs(te[5]) < math.radians(8))
        consistent = bool(np.all(np.isfinite(res.X)) and np.isfinite(res.defect) and res.defect < 0.02)
        return res, good, consistent, al, Xg

    ms_total = 0.0
    it_total = 0
    if switch is not None:
        # different runway end / HAC side: cold SCP from its geometric guess; adopt it only if it
        # reaches the ALI, otherwise stay on the plan being flown
        Xs0, Us0, tfs0, _ = initial_guess(pm, x0, switch, 40)
        Ns = _nodes_for(tfs0)
        Xs, Us, tfs = _resample(np.linspace(0, tfs0, 40), Xs0, Us0, Ns) if Ns != 40 else (Xs0, Us0, tfs0)
        rs, gs_, cs_, als, Xs = attempt(switch.rwy, switch.cfg, Xs, Us, tfs, 12)
        ms_total += rs.ms
        it_total += rs.iterations
        if gs_:
            prev = (rwy.name, int(hac["dir"]) if hac else 0)
            rwy, cfg, hac, reachable, kind = switch.rwy, switch.cfg, switch.hac(), True, "switch"
            Xg, Ug, tfg = Xs, Us, tfs
            decision.update(switch=True, frm=_side_name(*prev), to=_side_name(rwy.name, switch.side))
            res, good, consistent, al = rs, gs_, cs_, als
        else:
            decision.update(rejected="SCP did not reach the ALI on the new approach")
            switch = None
    if switch is None:
        res, good, consistent, al, Xg = attempt(rwy, cfg, Xg, Ug, tfg, 12 if kind == "cold" else 6)
        ms_total += res.ms
        it_total += res.iterations
    if not good:
        # Recovery: the SCP did not reach the ALI.  Its end state says which way the energy is off:
        #   too high / fast at the ALI -> a longer track (larger HAC, extra loop, S-turn / orbit)
        #                                 aimed at a band position below the one just flown
        #   too low / slow             -> a shortened approach (lower ALI, shorter prefinal, tighter HAC)
        #   no usable end state        -> a fresh geometric guess from the runway / HAC selection
        # (before, only the low-energy direction was handled: a high miss gave up at once.)
        lvl0 = 0
        band_t = BAND_TARGET
        tried = set()
        last, last_rwy, last_cfg, last_hac = res, rwy, cfg, hac
        gnote = ""
        if kind == "cold":
            tried.add("guess")           # the first attempt already was the selection's geometric guess
        for _ in range(3):
            te = last.term_err
            usable = bool(np.all(np.isfinite(te)) and np.all(np.isfinite(last.X)) and np.isfinite(last.defect)
                          and last.defect < 0.05)
            v_ali = float(last.X[-1, 3]) if usable else 0.0
            e_exc = float(te[2] + v_ali * te[3] / G0) if usable else 0.0    # energy-height excess at the ALI
            mode = "guess"
            cand = None
            if usable and (te[2] > 150 or te[3] > 8) and e_exc > 0.0:
                mode = "stretch"
                try:
                    cur = rate_track(pm, x0, last.X, last_rwy, last_cfg, last_hac)
                    span = max(cur.dE_max - cur.dE_min, 1.0)
                    band_t = float(np.clip(min(band_t, cur.band_pos) - e_exc / span - 0.05, 0.02, BAND_TARGET))
                except Exception:
                    band_t = max(band_t - 0.15, 0.02)
                cand = stretched_pick(pm, x0, ctx.ends, ApproachConfig(), band_t, prefer=last_rwy.name)
            elif usable and (te[2] < -150 or te[3] < -8):
                mode = "shorten"
                lvl0 += 1
                if lvl0 > 2:
                    break
                cs = [c for c in candidates(pm, x0, ctx.ends, ApproachConfig(), lvl0) if c.rwy.name == last_rwy.name]
                cs = [c for c in cs if c.dE_avail >= c.dE_min * 1.04] or cs
                if cs:
                    cand = min(cs, key=lambda c: abs(c.band_pos - BAND_TARGET))
                    cand.energy = "low"
            if cand is None:
                if "guess" in tried:
                    break
                tried.add("guess")
                mode = "guess"
                cand, _, gnote = select_plan(pm, x0, ctx.ends, ApproachConfig(), prefer=rwy.name)
            key = _cand_key(cand)
            if key in tried:
                if mode == "stretch":
                    band_t = max(band_t - 0.1, 0.02)     # same route again: aim lower next time
                    continue
                break
            tried.add(key)
            energy_cls = cand.energy if energy_cls in (None, "ok") else energy_cls
            Xg0, Ug0, tf0, _ = initial_guess(pm, x0, cand, 40)
            N2 = _nodes_for(tf0)
            Xg2, Ug2, tfg2 = _resample(np.linspace(0, tf0, 40), Xg0, Ug0, N2) if N2 != 40 else (Xg0, Ug0, tf0)
            r2, g2, c2, al2, Xg2 = attempt(cand.rwy, cand.cfg, Xg2, Ug2, tfg2, 12)
            ms_total += r2.ms
            it_total += r2.iterations
            if g2 or (c2 and not consistent):
                res, good, consistent, al, Xg, Ug, tfg = r2, g2, c2, al2, Xg2, Ug2, tfg2
                rwy, cfg, hac, kind = cand.rwy, cand.cfg, cand.hac(), "cold"
                reachable = True
                note = {"stretch": "SCP high at the ALI: re-planned on a longer track",
                        "shorten": f"SCP low at the ALI: shortened approach (level {lvl0})",
                        "guess": gnote}[mode]
                if mode == "stretch" and getattr(cand, "relaxed", False):
                    from .geometry import _stretch_note
                    note += f" ({_stretch_note(cand)})"
            if good:
                break
            last, last_rwy, last_cfg, last_hac = r2, cand.rwy, cand.cfg, cand.hac()
    te = res.term_err
    if good or consistent:
        X, U, tf = res.X, res.U, res.tf      # best-effort SCP trajectory even if the ALI is missed
    else:
        X, U, tf = Xg, Ug, tfg
    if good and not reachable and "headwind" not in note:
        # the geometric energy estimate said no, but the SCP (the planning dynamics) reached the ALI:
        # the estimate is a heuristic - the plan is flyable
        reachable = True
        note = (note + " - " if note else "") + "but the SCP reached the ALI: planned as reachable"
    if not good:
        reachable = False
        note = (note + "; " if note else "") + (
            f"ALI not reachable (miss: dh {te[2]:+.0f} m, dV {te[3]:+.1f} m/s) - best-effort trajectory"
            if consistent else "SCP failed - geometric reference")
    res.ms, res.iterations = ms_total, it_total
    if good:
        energy = "ok"
    elif consistent and np.all(np.isfinite(te)):
        e_exc = float(te[2] + float(X[-1, 3]) * te[3] / G0)
        energy = "high" if e_exc > 0.0 else "low"
    else:
        if energy_cls is None:
            energy_cls = select_plan(pm, x0, ctx.ends, ApproachConfig())[0].energy
        energy = energy_cls
    return dict(t0=req.get("t0", 0.0), rwy=rwy.name, cfg=asdict(cfg), hac=hac, X=X, U=U, tf=tf, kind=kind,
                energy=energy,
                converged=bool(res.converged), ok=bool(good), fallback=not (good or consistent), iterations=res.iterations,
                ms=res.ms, wall_ms=(time.perf_counter() - t_start) * 1e3, cost=res.cost, defect=res.defect,
                term_err=te, reachable=bool(reachable) and bool(good), note=note, al=_al_dict(al),
                wind=req.get("wind", (0.0, 0.0)), episode=req.get("episode"), decision=decision)


def _cand_key(c):
    """Identity of a geometric route, so the recovery does not retry the same one."""
    p = c.path
    return (c.rwy.name, int(c.side), int(getattr(p, "loops", 0)), round(float(getattr(p, "r2", 0.0)) / 250.0),
            int(c.level), type(p).__name__, round(float(getattr(p, "a", 0.0)) / 500.0),
            round(float(getattr(p, "r1", getattr(p, "r0", 0.0))) / 500.0), int(getattr(p, "s1", getattr(p, "s0", 0))))


def _fault_result(req, pm, ctx, x0, t_start):
    """Fault injection for tests (req['fault_scp']): no SCP at all - the geometric reference of the
    selected route is returned as a failed plan, so the guidance's degraded modes are exercised."""
    cand, _, note = select_plan(pm, x0, ctx.ends, ApproachConfig(), prefer=req.get("prefer"))
    X, U, tf, _ = initial_guess(pm, x0, cand, 40)
    al = build_reference(pm, cand.rwy, cand.cfg)
    return dict(t0=req.get("t0", 0.0), rwy=cand.rwy.name, cfg=asdict(cand.cfg), hac=cand.hac(), X=X, U=U, tf=tf,
                kind="cold", energy=cand.energy, converged=False, ok=False, fallback=True, iterations=0, ms=0.0,
                wall_ms=(time.perf_counter() - t_start) * 1e3, cost=float("nan"), defect=float("nan"),
                term_err=np.full(6, np.nan), reachable=False,
                note="fault injection: SCP disabled" + (f" ({note})" if note else ""), al=_al_dict(al),
                wind=req.get("wind", (0.0, 0.0)), episode=req.get("episode"), decision=None)


def _final_approach(ctx, pm, x0):
    """Already established on (or near) a final approach: skip TAEM, track the A/L reference."""
    cfg = ApproachConfig()
    for rwy in ctx.ends.values():
        x, y = rwy.to_rwy(x0[0], x0[1])
        herr = abs((x0[5] - rwy.heading + math.pi) % TWO_PI - math.pi)
        if not (cfg.x_ali - 1500.0 < x < cfg.x_td and abs(y) < 1200.0 and herr < math.radians(30.0)
                and x0[2] - rwy.elev < cfg.h_ali + 600.0):
            continue
        al = build_reference(pm, rwy, cfg)
        i0 = int(np.clip(np.searchsorted(al.x, x), 0, len(al.x) - 3))
        X = np.vstack([x0, al.X[i0 + 1]])
        X[1, 5] = x0[5] + ((al.X[i0 + 1, 5] - x0[5] + math.pi) % TWO_PI - math.pi)
        U = np.vstack([al.U[i0], al.U[i0 + 1]])
        tf = max(float(al.t[i0 + 1] - al.t[i0]), 0.2)
        alx = ALRef(rwy=rwy, cfg=al.cfg, t=al.t[i0 + 1:] - al.t[i0 + 1], X=al.X[i0 + 1:], U=al.U[i0 + 1:],
                    gear=al.gear[i0 + 1:], x=al.x[i0 + 1:], ph=al.ph[i0 + 1:], keys=al.keys, sb_pf=al.sb_pf)
        return dict(t0=0.0, rwy=rwy.name, cfg=asdict(al.cfg), hac=None, X=X, U=U, tf=tf, kind="final",
                    converged=True, ok=True, fallback=False, iterations=0, ms=0.0, wall_ms=0.0, cost=0.0,
                    defect=0.0, term_err=np.zeros(6), reachable=True, note="established on final approach",
                    al=_al_dict(alx), wind=None)
    return None


class SyncPlanner:
    mode = "sync"

    def __init__(self, ctx=None):
        if ctx is not None:
            set_context(ctx)
        self._res = None
        self.busy = False

    def submit(self, req):
        try:
            self._res = plan(req)
        except Exception as e:          # like the async planner: report, guidance keeps the old reference
            self._res = {"error": repr(e), "episode": req.get("episode")}
        return True

    def poll(self):
        r, self._res = self._res, None
        return r

    def close(self):
        pass


class AsyncPlanner:
    mode = "async"

    def __init__(self):
        mpc_ctx = mp.get_context("spawn")
        self.ex = ProcessPoolExecutor(max_workers=1, mp_context=mpc_ctx, initializer=init_worker)
        self.fut = None
        self.ex.submit(int, 0)       # start the worker now (loads tables / terrain)

    @property
    def busy(self):
        return self.fut is not None

    def submit(self, req):
        if self.fut is not None:
            return False
        self.fut = self.ex.submit(plan, req)
        return True

    def poll(self):
        if self.fut is not None and self.fut.done():
            f, self.fut = self.fut, None
            try:
                return f.result()
            except Exception as e:      # worker crash -> report, guidance keeps the old reference
                return {"error": repr(e)}
        return None

    def cancel(self):
        if self.fut is not None:
            self.fut.cancel()
            self.fut = None

    def close(self):
        try:
            self.ex.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
