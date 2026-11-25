"""Render the README figures (docs/figures/*.svg) from the simulator itself.

    pip install matplotlib                       # only needed for this script
    python tools/readme_figures.py               # data (cached) + all figures
    python tools/readme_figures.py --only scp_iterates approach_profile
    python tools/readme_figures.py --refresh     # re-run the flight / SCP instead of the cache

Data figures come from the code, not from hand-drawn curves:
  * the trim tables (sim/trim.py) and the planning model (sim/gnc/model.py),
  * the A/L vertical profile and the final-flare law (sim/gnc/approach.py),
  * the worked SCP example of lesson 2.5 / 3.6 (planner.plan, every SCP iterate recorded),
  * one complete autonomous flight on the 6-DOF plant (seed 1000, calm air).
The flight and SCP runs take a few minutes; their results are cached in
``.cache/readme_figures.pkl`` (git-ignored scratch, delete or pass --refresh to redo).
"""
from __future__ import annotations

import argparse
import math
import os
import pickle
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

R2D = 180.0 / math.pi
FT = 0.3048
KT = 1.0 / 0.514444
CACHE = os.path.join(ROOT, ".cache", "readme_figures.pkl")
OUT = os.path.join(ROOT, "docs", "figures")


# ====================================================================== data collection
def collect_scp_example():
    """README lesson 2.5: spawn (E,N)=(-30,-25) km, 15 km, 270 m/s, gamma -6 deg, heading 045."""
    from sim.gnc import planner, scp
    from sim.gnc.geometry import ApproachConfig, initial_guess, select_plan
    iterates = []
    orig = scp.ScpSolver.defects

    def rec(self, X, U, tf):
        iterates.append((X.copy(), U.copy(), float(tf)))
        return orig(self, X, U, tf)

    hists = []
    orig_solve = scp.ScpSolver.solve

    def rec_solve(self, *a, **k):
        res = orig_solve(self, *a, **k)
        hists.append(res.history)
        return res

    scp.ScpSolver.defects = rec
    scp.ScpSolver.solve = rec_solve
    try:
        planner.init_worker()
        ctx = planner._CTX
        x0 = np.array([-30000.0, -25000.0, 15000.0, 270.0, math.radians(-6.0), math.radians(45.0)])
        cand, _, _ = select_plan(ctx.model, x0, ctx.ends, ApproachConfig())
        Xg0, Ug0, tf0, _ = initial_guess(ctx.model, x0, cand, 40)
        guess_path = cand.path.sample(100.0)
        r = planner.plan(dict(x0=x0.tolist(), u0=[6.0, 0.0, 0.0], t0=0.0))
    finally:
        scp.ScpSolver.defects = orig
        scp.ScpSolver.solve = orig_solve
    return dict(x0=x0, guess=(Xg0, Ug0, tf0), guess_path=np.array(guess_path[:2]).T,
                cand=dict(rwy=cand.rwy.name, side=int(cand.side), band=float(cand.band_pos),
                          dE=(float(cand.dE_min), float(cand.dE_avail), float(cand.dE_max))),
                plan=dict(X=r["X"], U=r["U"], tf=r["tf"], hac=r["hac"], rwy=r["rwy"], al=r["al"],
                          iterations=r["iterations"], ms=r["ms"]),
                iterates=iterates[:-1], hist=hists[0] if hists else [])


def collect_flight(seed=1000):
    """One complete autonomous flight (6-DOF plant, SCP+MPC guidance, calm air)."""
    from sim.simulator import Simulator
    sim = Simulator(seed=seed, verbose=False)
    sim.wind.set(speed=0.0, dir_from=0.0, intensity=0.0, auto=False)
    sim.drain_messages()
    sim.reset(seed=seed)
    log, plans, mpcs, events = [], [], [], []
    k = 0
    t_wall = time.perf_counter()
    while sim.t < 1500.0:
        sim.step()
        k += 1
        if k % 5 == 0:                       # 20 Hz log
            s = sim.snapshot()
            g = sim.guidance
            c = sim.gcmd
            log.append((s.t, *s.pos, s.tas, s.eas, s.mach, s.qbar, s.gamma, s.alpha, s.bank, s.vs,
                        s.agl_gear, s.rwy_along, s.rwy_cross, s.ground_speed, s.theta,
                        getattr(c, "alpha_deg", np.nan), getattr(c, "bank_deg", np.nan),
                        getattr(c, "speedbrake_deg", np.nan), float(sim.gear.pos),
                        ["PLAN", "TAEM", "A/L", "TOUCHDOWN", "ROLLOUT", "STOPPED"].index(g.phase)
                        if g.phase in ("PLAN", "TAEM", "A/L", "TOUCHDOWN", "ROLLOUT", "STOPPED") else -1,
                        g.sub))
        for m in sim.drain_messages():
            if m.get("type") == "plan":
                plans.append(m)
            elif m.get("type") == "mpc":
                mpcs.append(m)
            elif m.get("type") == "event":
                events.append((sim.t, m["level"], m["text"]))
        if sim.status in ("STOPPED", "CRASHED"):
            break
    cols = ["t", "E", "N", "h", "V", "eas", "mach", "qbar", "gamma", "alpha", "bank", "vs", "agl_gear",
            "along", "cross", "gs", "theta", "alpha_cmd", "bank_cmd", "sb_cmd", "gear", "phase"]
    arr = np.array([r[:-1] for r in log], float)
    data = {c: arr[:, i] for i, c in enumerate(cols)}
    data["sub"] = [r[-1] for r in log]
    from sim.terrain import Terrain
    ter = Terrain(sim.geo.world, verbose=False)
    ext = 60000.0
    g = np.linspace(-ext, ext, 241)
    EE, NN = np.meshgrid(g, g)
    H = np.vectorize(lambda e, n: ter.height(float(e), float(n)))(EE, NN)
    return dict(log=data, plans=plans, mpcs=mpcs, events=events, status=sim.status, touchdown=sim.touchdown,
                stop=sim.stop_info, world=sim.geo.world, terrain=(g, H), wall=time.perf_counter() - t_wall)


def collect_tables():
    from sim.gnc import planner
    planner.init_worker()
    pm = planner._CTX.model
    M = np.linspace(0.25, 2.8, 120)
    out = dict(M=M, ld_max=np.array([float(pm.ld_best(m)) for m in M]),
               ld_min=np.array([float(pm.ld_min(m)) for m in M]),
               a_max=np.array([float(pm.alpha_max(m)) for m in M]),
               a_best=np.array([float(pm.alpha_best(m)) for m in M]),
               bank_rate=np.array([float(pm.bank_rate(m)) for m in M]))
    al = np.linspace(-2, 30, 161)
    polar = {}
    for m in (0.4, 0.9, 1.5, 2.5):
        for sb in (0.0, 98.6):
            cl, cd = pm.coeffs(al, np.full_like(al, m), np.full_like(al, sb))[:2]
            polar[(m, sb)] = (al, np.asarray(cl), np.asarray(cd))
    out["polar"] = polar
    return out


SECTIONS = ("tables", "scp", "flight")


def load_data(refresh=()):
    """Cached collection; ``refresh``: section names to recompute ("all" = everything)."""
    d = {}
    if os.path.exists(CACHE):
        with open(CACHE, "rb") as f:
            d = pickle.load(f)
    todo = [k for k in SECTIONS if k not in d or k in refresh or "all" in refresh
            or (k == "scp" and "hist" not in d[k])]
    if not todo:
        return d
    fn = dict(tables=collect_tables, scp=collect_scp_example, flight=collect_flight)
    for k in todo:
        t0 = time.perf_counter()
        d[k] = fn[k]()
        print(f"collected {k:<8} {time.perf_counter() - t0:6.1f} s", flush=True)
    os.makedirs(os.path.dirname(CACHE), exist_ok=True)
    with open(CACHE, "wb") as f:
        pickle.dump(d, f)
    return d


# ====================================================================== style
import matplotlib  # noqa: E402

matplotlib.use("Agg")
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon  # noqa: E402
from matplotlib.path import Path  # noqa: E402

CJK_FONTS = ["/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
             "/usr/share/fonts/opentype/noto/NotoSerifCJK-Medium.ttc",
             "/System/Library/Fonts/Supplemental/Songti.ttc", "C:/Windows/Fonts/simsun.ttc"]
for _p in CJK_FONTS:
    if os.path.exists(_p):
        try:
            fm.fontManager.addfont(_p)
        except Exception:
            pass
_CJK = next((n for n in ("Noto Serif CJK SC", "Songti SC", "SimSun")
             if any(f.name == n for f in fm.fontManager.ttflist)), "DejaVu Serif")

plt.rcParams.update({
    "font.family": [_CJK], "font.size": 8.0,
    "mathtext.fontset": "custom", "mathtext.rm": _CJK, "mathtext.it": "STIXGeneral:italic",
    "mathtext.bf": "STIXGeneral:bold", "mathtext.sf": "STIXGeneral", "mathtext.fallback": "stix",
    "axes.linewidth": 0.6, "axes.labelsize": 8.0, "axes.titlesize": 8.5, "axes.edgecolor": "#222222",
    "xtick.direction": "in", "ytick.direction": "in", "xtick.top": True, "ytick.right": True,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.minor.width": 0.4, "ytick.minor.width": 0.4,
    "xtick.major.size": 3.0, "ytick.major.size": 3.0, "xtick.minor.size": 1.6, "ytick.minor.size": 1.6,
    "xtick.labelsize": 7.2, "ytick.labelsize": 7.2, "legend.fontsize": 7.0, "legend.frameon": False,
    "legend.handlelength": 2.0, "lines.linewidth": 1.0, "grid.linewidth": 0.4, "grid.color": "#d9d9d9",
    "figure.facecolor": "white", "savefig.facecolor": "white", "svg.fonttype": "path", "svg.hashsalt": "planelands",
    "axes.unicode_minus": True,
})

C = dict(ink="#1a1a1a", blue="#1f4e79", lblue="#5b8db8", red="#b03a2e", orange="#c87f0a", green="#1e7d46",
         purple="#6c3483", teal="#117a65", gray="#7f8c8d", lgray="#bfc5c9")
PH_COL = {"ACQ": "#dbe6f1", "HAC": "#c4d6ea", "PRF": "#aec6e2", "OGS": "#f6e3c4", "PREFLARE": "#f3d0c9",
          "IGS": "#d5ecd9", "FLARE": "#e4d6ee", "DEROTATION": "#e6e6e6", "": "#eeeeee"}
PH_LINE = {"OGS": C["orange"], "PREFLARE": C["red"], "IGS": C["green"], "FLARE": C["purple"],
           "ROLLOUT": C["gray"], "TAEM": C["blue"]}
MM = 1.0 / 25.4
FIG_W = 180 * MM


def panel(ax, s, x=-0.13, y=1.02):
    ax.text(x, y, s, transform=ax.transAxes, fontsize=8.5, fontweight="bold", va="bottom", ha="left")


def ipanel(ax, s):
    """Panel letter inside the axes (stacked time histories)."""
    ax.text(0.006, 0.95, s, transform=ax.transAxes, fontsize=8.0, fontweight="bold", va="top", ha="left",
            zorder=10, bbox=dict(fc="white", ec="none", pad=0.8, alpha=0.85))


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name + ".svg")
    fig.savefig(p, bbox_inches="tight", pad_inches=0.04, metadata={"Date": None})
    if os.environ.get("FIG_PREVIEW"):            # PNG previews for checking the layout
        os.makedirs(os.environ["FIG_PREVIEW"], exist_ok=True)
        fig.savefig(os.path.join(os.environ["FIG_PREVIEW"], name + ".png"), dpi=170, bbox_inches="tight",
                    pad_inches=0.04)
    plt.close(fig)
    print("wrote", os.path.relpath(p, ROOT))


# ---------------------------------------------------------------- flowchart primitives (mm canvas)
FC = dict(proc=("#eef3f8", "#1f4e79"), dec=("#fdf3e1", "#a86a05"), term=("#e7f3ea", "#1e7d46"),
          fail=("#fbecea", "#a93226"), io=("#f4f4f4", "#555555"), sum=("white", "#1a1a1a"))


def canvas(w_mm, h_mm, y0=0.0):
    fig = plt.figure(figsize=(w_mm * MM, h_mm * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w_mm)
    ax.set_ylim(y0, y0 + h_mm)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


class Node:
    def __init__(self, x, y, w, h, kind):
        self.x, self.y, self.w, self.h, self.kind = x, y, w, h, kind

    def p(self, side, off=0.0):
        """Port: 'l','r','t','b' (+ offset along the side)."""
        x, y, w, h = self.x, self.y, self.w / 2, self.h / 2
        return {"l": (x - w, y + off), "r": (x + w, y + off), "t": (x + off, y + h), "b": (x + off, y - h)}[side]


def node(ax, x, y, text, w=30, h=9, kind="proc", fs=7.6, bold=False, sub=None):
    fc, ec = FC[kind]
    if kind == "dec":
        ax.add_patch(Polygon([(x - w / 2, y), (x, y + h / 2), (x + w / 2, y), (x, y - h / 2)], closed=True,
                             fc=fc, ec=ec, lw=0.8, zorder=2))
    elif kind == "sum":
        ax.add_patch(Circle((x, y), w / 2, fc=fc, ec=ec, lw=0.8, zorder=2))
    else:
        r = h / 2 if kind == "term" else 1.2
        ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                    fc=fc, ec=ec, lw=0.8, zorder=2))
    if sub:
        nl = sub.count("\n") + 1
        lh = (fs - 1.2) * 1.3 / 72 * 25.4          # sub line height (mm)
        top = y + 0.5 * (3.4 + 0.5 + nl * lh)
        ax.text(x, top - 1.7, text, ha="center", va="center", fontsize=fs, zorder=3,
                fontweight="bold" if bold else "normal")
        ax.text(x, top - 3.9 - 0.5 * nl * lh, sub, ha="center", va="center", fontsize=fs - 1.2,
                color="#444444", zorder=3, linespacing=1.3)
    else:
        ax.text(x, y, text, ha="center", va="center", fontsize=fs, zorder=3, linespacing=1.25,
                fontweight="bold" if bold else "normal")
    return Node(x, y, w, h, kind)


def edge(ax, pts, label=None, at=None, lpos="c", ls="-", color="#222222", arrow=True, fs=6.8, lw=0.75):
    verts = [tuple(map(float, p)) for p in pts]
    path = Path(verts, [Path.MOVETO] + [Path.LINETO] * (len(verts) - 1))
    ax.add_patch(FancyArrowPatch(path=path, arrowstyle="-|>,head_length=2.4,head_width=1.2" if arrow else "-",
                                 lw=lw, color=color, ls=ls, shrinkA=0, shrinkB=0, zorder=1.5,
                                 mutation_scale=1.0, joinstyle="miter"))
    if label:
        if at is None:
            (x0, y0), (x1, y1) = verts[0], verts[1]
            at = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
        ha = {"c": "center", "l": "right", "r": "left"}[lpos]
        ax.text(at[0], at[1], label, ha=ha, va="center", fontsize=fs, color=color, zorder=3,
                bbox=dict(fc="white", ec="none", pad=0.6))


def lane(ax, x, y, w, h, title, note=None, fc="#f6f7f9", note_x=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.5", fc=fc,
                                ec="#9aa3ab", lw=0.6, ls=(0, (3, 2)), zorder=0.5))
    ax.text(x + 2.0, y + h - 2.2, title, ha="left", va="top", fontsize=7.8, fontweight="bold", color="#333333")
    if note:
        ax.text(note_x if note_x is not None else x + w - 2.0, y + h - 2.35, note,
                ha="left" if note_x is not None else "right", va="top", fontsize=6.8, color="#555555")


# ====================================================================== flowcharts / block diagrams
def fig_gnc_architecture(d=None):
    fig, ax = canvas(180, 134)
    # lanes (top to bottom)
    lane(ax, 10, 99, 168, 33, "规划层  SCP 轨迹规划", "每 20 s 或事件触发 · 单次 0.2–3 s · 独立进程 / Web Worker",
         note_x=50)
    lane(ax, 10, 66, 168, 30, "跟踪层  模型预测控制", "MPC 2.5 Hz · 制导输出 10 Hz", note_x=50)
    lane(ax, 10, 33, 168, 30, "内环  电传飞控 FCS", "50 Hz", note_x=46)
    lane(ax, 10, 2, 168, 28, "被控对象", "100 Hz（近地 400 Hz）", note_x=30)
    # planning
    a = node(ax, 32, 111, "跑道端 / HAC 侧选择", 34, 11, sub="能量带位置 b → 0.35")
    b = node(ax, 72, 111, "Dubins CSC 初值", 30, 11, sub=r"$(X,\ U,\ t_f)$，N 个节点")
    c = node(ax, 112, 111, "自由终端时间 SCP", 32, 11, sub="PTR · CLARABEL / ECOS")
    e = node(ax, 155, 111, "A/L 几何剖面", 34, 11, sub="OGS → 预拉平 → IGS → 拉平")
    edge(ax, [a.p("r"), b.p("l")])
    edge(ax, [b.p("r"), c.p("l")])
    edge(ax, [c.p("t"), (112, 125), (32, 125), a.p("t")], ls=(0, (3, 1.5)), color=C["red"],
         label="未到达 ALI：拉长 / 缩短 / 重选", at=(72, 125))
    edge(ax, [c.p("r"), e.p("l")], label="ALI", at=(133.5, 111))
    # tracking
    f = node(ax, 40, 79, "空间重索引", 34, 11, sub=r"$i^\star=\arg\min_i\,\|W(p_i-p)\|$")
    g = node(ax, 92, 79, "LTV-MPC", 40, 11, sub=r"$K=28,\ \Delta t=1$ s · 凝聚 QP（OSQP）")
    h = node(ax, 155, 79, "着陆纵向律 / 拉平律", 36, 11, sub="A/L 段接管纵向")
    edge(ax, [c.p("b"), (112, 89), (40, 89), f.p("t")], label=r"TAEM 基准 $\bar x(t),\ \bar u(t)$",
         at=(100, 89))
    edge(ax, [e.p("b"), h.p("t")], label=r"剖面 $H(x)$", at=(153, 95), lpos="l")
    edge(ax, [f.p("r"), g.p("l")], label=r"$x^r_k,\ u^r_k$", at=(65, 82.5))
    # inner loop
    i = node(ax, 32, 46, "α / β / μ 外环", 30, 11, sub="参考模型 + PI")
    j = node(ax, 70, 46, "航迹运动学", 30, 11, sub=r"→ 稳定轴 → 体轴 $p,q,r$")
    k = node(ax, 110, 46, "非线性动态逆", 34, 11, sub=r"$M_\mathrm{req}=J\dot\omega_d+\omega\times J\omega$")
    m = node(ax, 152, 46, "控制分配", 32, 11, sub="加权伪逆 · 俯仰优先")
    edge(ax, [g.p("b"), (92, 64.5), (32, 64.5), i.p("t")], label=r"$\alpha_c,\ \phi_c,\ \delta_{sb,c}$",
         at=(60, 64.5))
    edge(ax, [h.p("b"), (155, 64.5), (92, 64.5)], arrow=False, label=r"$\alpha_c$（A/L 段）", at=(126, 64.5))
    ax.add_patch(Circle((92, 64.5), 0.7, color="#222222", zorder=3))
    edge(ax, [i.p("r"), j.p("l")], label=r"$\dot\alpha,\dot\mu,\dot\beta$", at=(51, 49.5))
    edge(ax, [j.p("r"), k.p("l")], label=r"$\omega_c$", at=(89, 49.5))
    edge(ax, [k.p("r"), m.p("l")], label=r"$\Delta M$", at=(131.5, 49.5))
    # plant
    n = node(ax, 152, 14, "作动器", 32, 11, sub="一阶滞后 · 速率 / 位置限幅")
    o = node(ax, 98, 14, "6-DOF 刚体", 40, 11, sub="分片气动 · 起落架 · 减速伞")
    q = node(ax, 44, 14, "大气与风", 30, 11, kind="io", sub="平均风 · Dryden · 雷暴")
    edge(ax, [m.p("b"), n.p("t")], label="4 片升降副翼 · 方向舵 · 体襟翼", at=(149, 31.5), lpos="l")
    edge(ax, [n.p("l"), o.p("r")], label=r"$\delta$", at=(125, 16.5))
    edge(ax, [q.p("r"), o.p("l")], label="阵风", at=(66.5, 16.5))
    # state feedback bus
    bus = 5.0
    edge(ax, [o.p("b"), (98, 4.2), (bus, 4.2), (bus, 111)], arrow=False, color=C["blue"], lw=0.9)
    for y_, tgt in ((111, a), (79, f), (46, i)):
        edge(ax, [(bus, y_), tgt.p("l")], color=C["blue"], lw=0.9)
        ax.add_patch(Circle((bus, y_), 0.7, color=C["blue"], zorder=3))
    ax.text(bus - 2.4, 58, "状态反馈  x", rotation=90, ha="center", va="center", fontsize=7, color=C["blue"])
    save(fig, "gnc_architecture")


def fig_flight_phases(d=None):
    fig, ax = canvas(180, 62)
    lane(ax, 22, 27, 52, 33, "TAEM", "SCP 基准 + MPC 跟踪", fc="#eef3f9")
    lane(ax, 78, 27, 80, 33, "A/L", "固定剖面 + 着陆律", fc="#fbf4ea")
    y = 41
    p0 = node(ax, 11, y, "PLAN", 14, 8, kind="term", fs=7.2)
    t1 = node(ax, 32, y, "ACQ", 14, 8, sub="截获")
    t2 = node(ax, 48, y, "HAC", 14, 8, sub="航向对准圆")
    t3 = node(ax, 64, y, "PRF", 14, 8, sub="五边前段")
    a1 = node(ax, 88, y, "OGS", 14, 8, sub="−20° 下滑")
    a2 = node(ax, 106, y, "PREFLARE", 16, 8, fs=7.0, sub="R = 8 km 拉起")
    a3 = node(ax, 126, y, "IGS", 14, 8, sub="−1.5° 下滑")
    a4 = node(ax, 146, y, "FLARE", 14, 8, sub="最终拉平")
    td = node(ax, 170, y, "TOUCH-\nDOWN", 15, 9, kind="term", fs=6.6)
    for s_, t_ in ((p0, t1), (t1, t2), (t2, t3), (t3, a1), (a1, a2), (a2, a3), (a3, a4), (a4, td)):
        edge(ax, [s_.p("r"), t_.p("l")])
    # transition conditions
    for x_, s_ in ((40.0, "切入 HAC"), (56.0, "离开 HAC"), (97.0, "切点"), (116.0, "指数段"),
                   (136.0, r"$h_\mathrm{FF}$")):
        ax.text(x_, y + 7.5, s_, ha="center", va="bottom", fontsize=6.3, color="#555555")
    for x_, s_ in ((21.0, "计划就绪"), (76.0, "ALI")):
        ax.text(x_, y - 6.5, s_, ha="center", va="top", fontsize=6.3, color="#555555")
    # ground segment
    r1 = node(ax, 170, 12, "ROLLOUT", 16, 8, sub="伞 · 刹车 · 转向")
    r2 = node(ax, 140, 12, "STOPPED", 16, 8, kind="term", fs=7.0)
    edge(ax, [td.p("b"), r1.p("t")], label="降前轮", at=(170, 24.5))
    edge(ax, [r1.p("l"), r2.p("r")])
    # degraded modes
    node(ax, 48, 12, "降级模式", 66, 9, kind="fail", sub="GEOREF / MAXRANGE / HOLD，每 5 s 冷启动重规划")
    edge(ax, [(40, y - 4), (40, 16.5)], ls=(0, (3, 1.5)), color=C["red"], label="无可用计划", at=(40, 25))
    edge(ax, [(56, 16.5), (56, y - 4)], ls=(0, (3, 1.5)), color=C["green"], label="重规划成功", at=(56, 25))
    save(fig, "flight_phases")


def fig_scp_flowchart(d=None):
    fig, ax = canvas(180, 136, y0=10)
    x0 = 62
    s = node(ax, x0, 142, r"输入：$x_0,\ u_\mathrm{prev},\ x_f$（ALI），初值 $(\bar X,\bar U,\bar s)$", 84, 8, kind="term")
    a = node(ax, x0, 129, r"$r \leftarrow 1,\ \ it \leftarrow 1$", 40, 7)
    b = node(ax, x0, 116, "沿参考轨迹线性化", 66, 10,
             sub=r"$A_k,\ B_k^\pm,\ S_k,\ z_k$ 与路径约束系数 — 中心差分，一次向量化调用")
    c = node(ax, x0, 102, "求解凸子问题（DPP 参数化）", 66, 10, sub="CLARABEL；失败时 ECOS")
    dd = node(ax, x0, 87, "子问题可行？", 34, 11, kind="dec")
    e = node(ax, x0, 72, "非线性缺陷与步长", 66, 10,
             sub=r"$d_\max=\max_k\|(\Phi(x_k,u_k,u_{k+1},s\Delta\tau)-x_{k+1})\oslash D_x\|_\infty$")
    f = node(ax, x0, 52, "收敛？", 30, 13, kind="dec")
    ax.text(x0 + 17, 60.5, r"$d_\max<5\!\times\!10^{-3}$，$\|\nu\|_1<10^{-3}$，" "\n"
            r"且 步长 $<2\!\times\!10^{-3}$ 或 $\Delta J/J<1\%$", fontsize=6.4, va="center", ha="left",
            color="#555555", linespacing=1.4)
    g = node(ax, x0, 35, "调整信赖域", 66, 10,
             sub=r"$d_\max>0.05:\ r\leftarrow\max(r/2,\,0.02)$；$d_\max<0.01:\ r\leftarrow\min(1.5r,\,2)$")
    h = node(ax, x0, 20, r"$it < it_\max$？", 34, 11, kind="dec")
    ax.text(x0, 12.3, "冷启动 12 次，热启动 6 次", fontsize=6.4, va="center", ha="center", color="#555555")
    fail = node(ax, 20, 87, r"$r \leftarrow r/2$", 20, 7, kind="fail")
    xa = 148
    acc = node(ax, xa, 52, "接受检查", 36, 13, kind="dec")
    ax.text(xa, 42.5, r"$d<0.02$；$|\Delta E|,|\Delta N|<300$ m" "\n"
            r"$|\Delta h|<150$ m，$|\Delta V|<8$ m/s" "\n" r"$|\Delta\gamma|<3^\circ$，$|\Delta\psi|<8^\circ$",
            fontsize=6.3, ha="center", va="top", color="#555555", linespacing=1.45)
    ok = node(ax, xa, 84, "输出 TAEM 基准", 40, 10, kind="term", sub="交给 MPC 跟踪")
    rec = node(ax, xa, 114, "", 48, 22, kind="fail")
    ax.text(xa, 122, "恢复（planner.plan）", ha="center", va="center", fontsize=7.6)
    ax.text(xa, 111.5, r"$e_\mathrm{exc}=\Delta h+V_\mathrm{ALI}\,\Delta V/g$" "\n"
            r"$e_\mathrm{exc}>0$：下调 $b^\star$，换更长航迹" "\n" r"$e_\mathrm{exc}<0$：缩短进近" "\n"
            "不可用：几何重选", fontsize=6.3, ha="center", va="center", color="#5b1f19", linespacing=1.4)
    edge(ax, [s.p("b"), a.p("t")])
    edge(ax, [a.p("b"), b.p("t")])
    edge(ax, [b.p("b"), c.p("t")])
    edge(ax, [c.p("b"), dd.p("t")])
    edge(ax, [dd.p("b"), e.p("t")], label="是", at=(x0 + 3, 79.5))
    edge(ax, [dd.p("l"), fail.p("r")], label="否", at=(37.5, 89.5))
    edge(ax, [e.p("b"), f.p("t")], label=r"更新 $(\bar X,\bar U,\bar s)$", at=(x0 - 3, 62.5), lpos="l")
    edge(ax, [f.p("b"), g.p("t")], label="否", at=(x0 + 3, 42.5))
    edge(ax, [g.p("b"), h.p("t")])
    edge(ax, [h.p("l"), (8, 20), (8, 116), b.p("l")], label=r"是，$it{+}{+}$", at=(8, 52))
    edge(ax, [fail.p("t"), (20, 116)], arrow=False)
    ax.add_patch(Circle((20, 116), 0.7, color="#222222", zorder=3))
    edge(ax, [f.p("r"), acc.p("l")], label="是", at=(104, 54.5))
    edge(ax, [h.p("r"), (118, 20), (118, 52)], arrow=False, label="否", at=(118, 34))
    ax.add_patch(Circle((118, 52), 0.7, color="#222222", zorder=3))
    edge(ax, [acc.p("t"), ok.p("b")], label="通过", at=(xa + 5, 68.5), lpos="r")
    edge(ax, [acc.p("r"), (174, 52), (174, 114), rec.p("r")], label="未通过", at=(174, 84))
    edge(ax, [rec.p("t"), (xa, 142), s.p("r")], ls=(0, (3, 1.5)), color=C["red"],
         label="新候选：初值 → 再解", at=(126, 142))
    save(fig, "scp_flowchart")


def fig_degraded_modes(d=None):
    fig, ax = canvas(180, 96, y0=-2)
    a = node(ax, 30, 84, "重规划请求", 36, 10, kind="term", sub="定时 / 误差 / 风变化")
    b = node(ax, 30, 65, "计划可用？", 34, 12, kind="dec")
    c = node(ax, 30, 46, "跟踪新计划", 32, 8, kind="term")
    dd = node(ax, 82, 65, "旧计划仍可飞？", 40, 12, kind="dec")
    ax.text(82, 57, "旧计划好，跟踪误差 < 4 km，\n连续失败 < 3 次", ha="center", va="top", fontsize=6.4,
            color="#555555", linespacing=1.4)
    e = node(ax, 82, 84, "继续飞旧计划", 36, 10, sub="5 s 后重试")
    f = node(ax, 138, 65, "能量方向", 34, 12, kind="dec")
    ax.text(138, 73, "energy_direction()", ha="center", fontsize=6.3, color="#555555")
    g1 = node(ax, 62, 27, "GEOREF", 40, 15, kind="fail", fs=7.4, sub="MPC 跟踪几何基准\n（第 2 课的初值轨迹）")
    g2 = node(ax, 111, 27, "MAXRANGE", 46, 15, kind="fail", fs=7.4,
              sub="以 $(L/D)_\\max$ 飞向最远可达进近点\n截获中线后沿中线飞")
    g3 = node(ax, 159, 27, "HOLD", 38, 15, kind="fail", fs=7.4, sub="绕 HAC 中心盘旋\n坡度 ≈35°，减速板 80°")
    node(ax, 135, 4.5, "地形前视", 86, 10, kind="io",
             sub="预测 8–60 s；冲突时转向最近安全航向并收减速板")
    edge(ax, [a.p("b"), b.p("t")])
    edge(ax, [b.p("b"), c.p("t")], label="是", at=(33, 54))
    edge(ax, [b.p("r"), dd.p("l")], label="否", at=(50, 67.5))
    edge(ax, [dd.p("t"), e.p("b")], label="是", at=(85, 74))
    edge(ax, [dd.p("r"), f.p("l")], label="否", at=(111.5, 67.5))
    edge(ax, [f.p("b"), (138, 44), (62, 44), g1.p("t")], label="正常", at=(74, 44))
    edge(ax, [(111, 44), g2.p("t")], label="不足 / 转弯受限", at=(111, 39.5))
    edge(ax, [(138, 44), (159, 44), g3.p("t")], label="过剩", at=(159, 39.5))
    ax.add_patch(Circle((111, 44), 0.7, color="#222222", zorder=3))
    ax.add_patch(Circle((138, 44), 0.7, color="#222222", zorder=3))
    edge(ax, [g2.p("b"), (111, 9.5)], color="#777777", ls=(0, (2, 1.5)))
    edge(ax, [g3.p("b"), (159, 9.5)], color="#777777", ls=(0, (2, 1.5)))
    edge(ax, [g1.p("l"), (8, 27), (8, 84), a.p("l")], ls=(0, (3, 1.5)), color=C["blue"],
         label="三种模式均每 5 s\n冷启动重规划", at=(8, 74.5))
    edge(ax, [e.p("l"), a.p("r")], ls=(0, (3, 1.5)), color=C["blue"])
    save(fig, "degraded_modes")


def fig_fcs_ndi(d=None):
    fig, ax = canvas(180, 76, y0=-2)
    y = 50
    i0 = node(ax, 12, y, r"$\alpha_c,\ \mu_c$" "\n" r"$\beta_c=0$", 16, 12, kind="io")
    a = node(ax, 40, y, "姿态外环", 28, 14, sub="α 参考模型 0.7 s + PI\nμ：P；β：PI")
    b = node(ax, 74, y, "航迹运动学", 28, 14, sub="升力 / 侧力 / 重力前馈\n→ 稳定轴 → 体轴")
    sm = node(ax, 96, y, "", 5, 5, kind="sum")
    ax.text(97.3, y - 3.6, "−", ha="center", va="center", fontsize=7)
    k = node(ax, 112, y, r"$K_\omega$", 12, 9)
    c = node(ax, 136, y, "动态逆", 26, 14, sub=r"$J\dot\omega_d+\omega\times J\omega$" "\n" r"$-\,M_\mathrm{gear}-M_0$")
    m = node(ax, 166, y, "控制分配", 22, 14, sub="加权伪逆\n俯仰优先")
    act = node(ax, 166, 20, "作动器", 22, 11, sub="速率 / 位置限幅")
    p = node(ax, 112, 20, "6-DOF 对象", 34, 11, sub="分片气动 · 起落架")
    bb = node(ax, 136, 33, r"在线控制效能 $B=\partial M/\partial u$", 38, 9, kind="io", fs=6.8,
              sub="每周期 6 次批量有限差分")
    edge(ax, [i0.p("r"), a.p("l")])
    edge(ax, [a.p("r"), b.p("l")], label=r"$\dot\alpha,\dot\mu,\dot\beta$", at=(57, y + 3.2))
    edge(ax, [b.p("r"), sm.p("l")], label=r"$\omega_c$", at=(91, y + 3.2))
    edge(ax, [sm.p("r"), k.p("l")])
    edge(ax, [k.p("r"), c.p("l")], label=r"$\dot\omega_d$", at=(120.5, y + 3.2))
    edge(ax, [c.p("r"), m.p("l")], label=r"$\Delta M$", at=(152, y + 3.2))
    edge(ax, [(169, y - 7), (169, 25.5)], label=r"$\Delta u$", at=(172.5, 35))
    edge(ax, [bb.p("r"), (160, 33), (160, y - 7)], color="#555555")
    edge(ax, [act.p("l"), p.p("r")], label=r"$\delta_{e,1..4},\ \delta_r,\ \delta_{bf}$", at=(143, 22.8))
    edge(ax, [p.p("l"), (70, 20), (70, 8), (96, 8), sm.p("b")], color=C["blue"], label=r"$\omega=(p,q,r)$",
         at=(84, 8))
    edge(ax, [(70, 20), (40, 20), a.p("b")], color=C["blue"], label=r"$\alpha,\ \beta,\ \mu,\ V$", at=(53, 20))
    ax.add_patch(Circle((70, 20), 0.7, color=C["blue"], zorder=3))
    for x1, x2, txt in ((26, 88, "外环：姿态 → 角速率指令"), (92, 178, "内环：角速率 → 力矩 → 舵面")):
        ax.plot([x1, x1, x2, x2], [64, 66, 66, 64], color="#888888", lw=0.6)
        ax.text(0.5 * (x1 + x2), 67.3, txt, ha="center", va="bottom", fontsize=6.8, color="#444444")
    ax.text(4, 0.5, "时间尺度分离：外环带宽 ≪ 内环带宽。饱和作动器移出后重新分配（≤ 3 次），"
            "滚转 / 偏航需求按 1 → 0.6 → 0.3 → 0 缩减，保证俯仰力矩", fontsize=6.4, color="#555555")
    save(fig, "fcs_ndi")


# ====================================================================== data figures
G0 = 9.80665
PH_NAMES = ["PLAN", "TAEM", "A/L", "TOUCHDOWN", "ROLLOUT", "STOPPED"]


def _label(ph, sub):
    ph = int(ph)
    if ph in (1, 2):
        return sub
    return {3: "DEROTATION", 4: "ROLLOUT"}.get(ph, "")


def _segments(L):
    lab = [_label(p, s) for p, s in zip(L["phase"], L["sub"])]
    out, k0 = [], 0
    for k in range(1, len(lab) + 1):
        if k == len(lab) or lab[k] != lab[k0]:
            out.append((L["t"][k0], L["t"][min(k, len(lab) - 1)], lab[k0], k0, k))
            k0 = k
    return out


def _shade(ax, segs, t_ref=0.0, labels=False, ymax=None):
    for t0, t1, lb, _, _ in segs:
        ax.axvspan(t0 - t_ref, t1 - t_ref, color=PH_COL.get(lb, "#eeeeee"), lw=0, zorder=0)
        if labels and t1 - t0 > 6:
            ax.text(0.5 * (t0 + t1) - t_ref, 1.02, lb, transform=ax.get_xaxis_transform(), ha="center",
                    va="bottom", fontsize=6.2, color="#444444")


def _rwy(d):
    from sim.gnc.geometry import runway_ends
    f = d["flight"]
    return runway_ends(f["world"])[f["touchdown"]["rwy"]], runway_ends(f["world"])


def _event_t(f, prefix):
    for t, _, txt in f["events"]:
        if txt.startswith(prefix):
            return t
    return None


def _draw_runway(ax, ends, scale=1e-3, lw=2.2):
    e33 = ends["33"]
    p0 = e33.thr * scale
    p1 = (e33.thr + e33.d * e33.length) * scale
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#222222", lw=lw, solid_capstyle="butt", zorder=6)


def fig_trim_envelope(d):
    T = d["tables"]
    fig, axs = plt.subplots(1, 3, figsize=(FIG_W, 56 * MM), gridspec_kw=dict(wspace=0.42))
    ax = axs[0]
    cm = plt.get_cmap("viridis")
    machs = (0.4, 0.9, 1.5, 2.5)
    for i, m in enumerate(machs):
        col = cm(0.08 + 0.8 * i / (len(machs) - 1))
        amax = np.interp(m, T["M"], T["a_max"])
        for sb, ls in ((0.0, "-"), (98.6, (0, (3, 1.6)))):
            al, cl, cd = T["polar"][(m, sb)]
            k = al <= amax
            ax.plot(cd[k], cl[k], color=col, ls=ls, lw=1.0, label=f"Mach {m:g}" if sb == 0 else None)
            if sb == 0:
                j = np.argmax(np.where(k & (cl > 0.05), cl / cd, 0))
                ax.plot(cd[j], cl[j], "o", ms=3, mfc="white", mec=col, mew=0.9, zorder=4)
    ax.plot([], [], color="#555555", ls=(0, (3, 1.6)), label="减速板全开")
    ax.set_xlabel(r"配平阻力系数 $C_D$")
    ax.set_ylabel(r"配平升力系数 $C_L$")
    ax.set_xlim(0, 0.62)
    ax.set_ylim(-0.1, 1.05)
    ax.legend(loc="lower right", fontsize=6.3, handlelength=1.8)
    panel(ax, "(a)", -0.24)
    ax = axs[1]
    ax.fill_between(T["M"], T["ld_min"], T["ld_max"], color=C["blue"], alpha=0.12, lw=0)
    ax.plot(T["M"], T["ld_max"], color=C["blue"], label=r"$(L/D)_\max$")
    ax.plot(T["M"], T["ld_min"], color=C["red"], ls=(0, (4, 1.6)), label=r"$(L/D)_\min$（减速板全开）")
    ax.axvline(1.0, color="#999999", lw=0.5, ls=":")
    ax.text(1.7, 3.7, "可调范围", ha="center", fontsize=7, color=C["blue"])
    ax.set_xlabel("马赫数 $M$")
    ax.set_ylabel("配平升阻比 $L/D$")
    ax.set_xlim(T["M"][0], T["M"][-1])
    ax.set_ylim(0, 5.6)
    ax.legend(loc="upper right", fontsize=6.3)
    panel(ax, "(b)", -0.2)
    ax = axs[2]
    ax.plot(T["M"], T["a_max"], color=C["ink"], label=r"可用最大迎角 $\alpha_\max$")
    ax.plot(T["M"], T["a_best"], color=C["teal"], ls=(0, (4, 1.6)), label=r"最大升阻比迎角")
    ax.axvspan(1.2, 1.6, color="#f3d0c9", alpha=0.6, lw=0)
    ax.text(1.4, 1.2, "体襟翼\n饱和区", ha="center", fontsize=6.0, color=C["red"])
    ax.set_xlabel("马赫数 $M$")
    ax.set_ylabel(r"迎角 (°)")
    ax.set_xlim(T["M"][0], T["M"][-1])
    ax.set_ylim(0, 40)
    ax.tick_params(right=False)
    a2 = ax.twinx()
    br = T["bank_rate"] * (R2D if T["bank_rate"].max() < 1.0 else 1.0)
    a2.plot(T["M"], br, color=C["orange"], lw=0.9, ls=(0, (1, 1.2)))
    a2.set_ylabel(r"倾侧速率上限 $\dot\phi_\max$ (°/s)", color=C["orange"])
    a2.set_ylim(0, 14)
    a2.tick_params(axis="y", colors=C["orange"], direction="in")
    h1, l1 = ax.get_legend_handles_labels()
    ax.legend(h1 + [plt.Line2D([], [], color=C["orange"], ls=(0, (1, 1.2)))], l1 + [r"$\dot\phi_\max$（右轴）"],
              loc="upper right", fontsize=6.3)
    panel(ax, "(c)", -0.2)
    save(fig, "trim_envelope")


def fig_scp_iterates(d):
    S = d["scp"]
    from sim.gnc.geometry import runway_ends
    ends = runway_ends(d["flight"]["world"])
    fig = plt.figure(figsize=(FIG_W, 72 * MM))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.15, 1.0], height_ratios=[1, 1], wspace=0.38, hspace=0.55)
    ax = fig.add_subplot(gs[:, 0])
    Xg, Ug, tfg = S["guess"]
    its = S["iterates"]
    cols = [plt.get_cmap("Blues")(0.3 + 0.55 * k / max(len(its) - 1, 1)) for k in range(len(its))]
    gp = S["guess_path"] * 1e-3
    ax.plot(gp[:, 0], gp[:, 1], color=C["gray"], lw=0.8, ls=(0, (1, 1.3)), label="Dubins CSC 几何路径")
    ax.plot(Xg[:, 0] * 1e-3, Xg[:, 1] * 1e-3, color=C["gray"], lw=0.9, ls=(0, (4, 1.6)), label="初值轨迹")
    for k, (X, _, _) in enumerate(its[:-1]):
        ax.plot(X[:, 0] * 1e-3, X[:, 1] * 1e-3, color=cols[k], lw=0.7,
                label=f"SCP 第 1–{len(its) - 1} 次迭代（由浅到深）" if k == 0 else None)
    Xf = S["plan"]["X"]
    ax.plot(Xf[:, 0] * 1e-3, Xf[:, 1] * 1e-3, color=C["blue"], lw=1.4, label=f"SCP 第 {len(its)} 次迭代（收敛）")
    ax.plot(Xf[:, 0] * 1e-3, Xf[:, 1] * 1e-3, "o", ms=1.6, color=C["blue"])
    al = S["plan"]["al"]
    ax.plot(al["X"][:, 0] * 1e-3, al["X"][:, 1] * 1e-3, color=C["orange"], lw=1.2, label="A/L 剖面")
    hac = S["plan"]["hac"]
    ax.add_patch(Circle(np.array(hac["center"]) * 1e-3, hac["radius"] * 1e-3, fill=False, ec=C["teal"],
                        lw=0.6, ls=(0, (2, 1.5))))
    ax.text(hac["center"][0] * 1e-3, hac["center"][1] * 1e-3, "HAC", ha="center", va="center", fontsize=6.5,
            color=C["teal"])
    _draw_runway(ax, ends)
    ax.plot(*(S["x0"][:2] * 1e-3), marker="*", ms=7, color=C["red"], zorder=7)
    ax.annotate("出生点  15 km\nMach 0.91", S["x0"][:2] * 1e-3, xytext=(-25.5, -23.0), fontsize=6.3, ha="left",
                va="top",
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ali = al["X"][0, :2] * 1e-3
    ax.plot(*ali, marker="^", ms=4.5, color=C["orange"], zorder=7)
    ax.annotate("ALI", ali, xytext=(ali[0] + 3.0, ali[1] + 2.5), fontsize=6.5,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.annotate("RWY15", ends["15"].thr * 1e-3, xytext=(-6.5, -6.5), fontsize=6.5,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.set_aspect("equal")
    ax.set_xlabel("东向 E (km)")
    ax.set_ylabel("北向 N (km)")
    ax.set_xlim(-33, 4)
    ax.set_ylim(-28, 17)
    fig.legend(*ax.get_legend_handles_labels(), loc="lower center", ncol=5, fontsize=6.4, handlelength=2.0,
               bbox_to_anchor=(0.5, -0.07), columnspacing=1.4)
    panel(ax, "(a)", -0.26)
    # altitude
    ax = fig.add_subplot(gs[0, 1])
    tg = np.linspace(0, tfg, len(Xg))
    ax.plot(tg, Xg[:, 2] * 1e-3, color=C["gray"], lw=0.9, ls=(0, (4, 1.6)))
    for k, (X, _, tf) in enumerate(its[:-1]):
        ax.plot(np.linspace(0, tf, len(X)), X[:, 2] * 1e-3, color=cols[k], lw=0.7)
    ax.plot(np.linspace(0, S["plan"]["tf"], len(Xf)), Xf[:, 2] * 1e-3, color=C["blue"], lw=1.3)
    ax.set_ylabel("高度 h (km)")
    ax.set_xlim(0, 265)
    panel(ax, "(b)", -0.2)
    ax = fig.add_subplot(gs[1, 1])
    for k, (X, U, tf) in enumerate(its[:-1]):
        ax.plot(np.linspace(0, tf, len(U)), U[:, 1] * R2D, color=cols[k], lw=0.7)
    Uf = S["plan"]["U"]
    tt = np.linspace(0, S["plan"]["tf"], len(Uf))
    ax.plot(tt, Uf[:, 1] * R2D, color=C["blue"], lw=1.3, label=r"倾侧角 $\phi$")
    ax.plot(tt, Uf[:, 0], color=C["red"], lw=1.0, label=r"迎角 $\alpha$")
    ax.plot(tt, Uf[:, 2], color=C["orange"], lw=1.0, ls=(0, (4, 1.6)), label=r"减速板 $\delta_{sb}$")
    ax.axhline(0, color="#999999", lw=0.4)
    ax.set_xlabel("时间 t (s)")
    ax.set_ylabel("控制量 (°)")
    ax.set_xlim(0, 265)
    ax.set_ylim(-35, 68)
    ax.legend(loc="upper left", fontsize=5.9, ncol=3, handlelength=1.6, columnspacing=0.8)
    panel(ax, "(c)", -0.2)
    # convergence
    ax = fig.add_subplot(gs[:, 2])
    H = [h for h in S["hist"] if "defect" in h]
    it = np.array([h["it"] for h in H])
    ax.semilogy(it, [h["defect"] for h in H], "o-", color=C["red"], ms=3.2, mfc="white", mew=0.9,
                label=r"非线性缺陷 $d_\max$")
    ax.semilogy(it, [h["step"] for h in H], "s-", color=C["blue"], ms=3.0, mfc="white", mew=0.9,
                label=r"步长 $\|\Delta\|_\infty$")
    ax.axhline(5e-3, color=C["red"], lw=0.6, ls=(0, (3, 1.5)))
    ax.text(it[-1] + 0.15, 5e-3 * 1.25, r"$5\times10^{-3}$", fontsize=6.0, color=C["red"], ha="right")
    ax.set_xlabel("SCP 迭代次数")
    ax.set_ylabel("缩放坐标下的范数")
    ax.set_xticks(it)
    ax.set_ylim(1e-4, 4)
    ax.tick_params(right=False)
    a2 = ax.twinx()
    a2.plot(it, [h["tf"] for h in H], "^--", color=C["gray"], ms=3, lw=0.7, mfc="white")
    a2.set_ylabel(r"终端时间 $t_f$ (s)", color=C["gray"])
    a2.tick_params(axis="y", colors=C["gray"], direction="in")
    h1, l1 = ax.get_legend_handles_labels()
    ax.legend(h1 + [plt.Line2D([], [], color=C["gray"], ls="--", marker="^", ms=3, mfc="white", lw=0.7)],
              l1 + [r"$t_f$（右轴）"], loc="upper right", fontsize=6.0)
    panel(ax, "(d)", -0.22)
    save(fig, "scp_iterates")


def fig_flight_history(d):
    f = d["flight"]
    L = f["log"]
    t = L["t"]
    segs = _segments(L)
    t_td = f["touchdown"]["t"]
    fig, axs = plt.subplots(5, 1, figsize=(FIG_W, 128 * MM), sharex=True,
                            gridspec_kw=dict(hspace=0.12, height_ratios=[1.5, 1, 1, 1, 0.8]))
    for ax in axs:
        _shade(ax, segs)
        ax.axvline(t_td, color="#444444", lw=0.6, ls=(0, (3, 1.5)))
        ax.grid(axis="y")
    _shade(axs[0], segs, labels=True)
    # (a) altitude, SCP references, MPC predictions
    ax = axs[0]
    for k, p in enumerate(f["plans"]):
        nd = np.array(p["nodes"])
        ax.plot(nd[:, 0], nd[:, 3] * 1e-3, color=C["lblue"], lw=0.7, ls=(0, (3, 1.5)),
                label="SCP 基准（每次重规划）" if k == 0 else None)
        ax.plot(p["t0"], np.interp(p["t0"], t, L["h"]) * 1e-3, "v", ms=3.2, color=C["blue"], zorder=5)
    ax.plot(t, L["h"] * 1e-3, color=C["ink"], lw=1.0, label="6-DOF 实际", zorder=3)
    last = -1e9
    first = True
    for m in f["mpcs"]:
        if m["t"] - last >= 45.0 and m["t"] < t_td - 25:
            pr = np.array(m["pred"])
            ax.plot(m["t"] + np.arange(len(pr)), pr[:, 2] * 1e-3, color=C["orange"], lw=3.2, alpha=0.55,
                    solid_capstyle="round", zorder=1.5, label="MPC 预测（28 s 时域）" if first else None)
            ax.plot(m["t"], pr[0, 2] * 1e-3, "o", ms=2.2, color=C["orange"])
            last, first = m["t"], False
    ax.set_ylabel("高度 h (km)")
    ax.set_ylim(-0.2, 9.4)
    ax.legend(loc="upper right", fontsize=6.3, ncol=3, handlelength=1.8)
    ax.text(0.006, 0.05, "▼ 重规划时刻", transform=ax.transAxes, fontsize=6.0, color=C["blue"])
    ipanel(ax, "(a)")
    # (b) speed
    ax = axs[1]
    ax.plot(t, L["eas"] * KT, color=C["ink"], lw=1.0, label="等效空速 KEAS")
    ax.plot(t, L["gs"] * KT, color=C["teal"], lw=0.8, ls=(0, (4, 1.6)), label="地速")
    ax.set_ylabel("速度 (kt)")
    ax.set_ylim(0, 420)
    ax.tick_params(right=False)
    a2 = ax.twinx()
    a2.plot(t, L["mach"], color=C["purple"], lw=0.8, ls=(0, (1, 1.2)))
    a2.set_ylabel("马赫数", color=C["purple"])
    a2.set_ylim(0, 1.0)
    a2.tick_params(axis="y", colors=C["purple"], direction="in")
    h1, l1 = ax.get_legend_handles_labels()
    ax.legend(h1 + [plt.Line2D([], [], color=C["purple"], ls=(0, (1, 1.2)))], l1 + ["Mach（右轴）"],
              loc="lower left", fontsize=6.3, ncol=3)
    ipanel(ax, "(b)")
    # (c) alpha
    fl = L["phase"] <= 2
    ax = axs[2]
    ax.plot(t, np.where(fl, L["alpha_cmd"], np.nan), color=C["red"], lw=0.9, ls=(0, (3, 1.3)), label="指令")
    ax.plot(t, L["alpha"], color=C["ink"], lw=0.9, label="实际")
    ax.set_ylabel(r"迎角 $\alpha$ (°)")
    ax.set_ylim(-2, 16)
    ax.legend(loc="upper left", fontsize=6.3, ncol=2, bbox_to_anchor=(0.04, 1.0))
    ipanel(ax, "(c)")
    # (d) bank
    ax = axs[3]
    ax.plot(t, np.where(fl, L["bank_cmd"], np.nan), color=C["red"], lw=0.9, ls=(0, (3, 1.3)), label="指令")
    ax.plot(t, L["bank"], color=C["ink"], lw=0.9, label="实际（航迹滚转角 μ）")
    ax.set_ylabel(r"倾侧角 $\phi$ (°)")
    ax.set_ylim(-25, 32)
    ax.legend(loc="upper left", fontsize=6.3, ncol=2, bbox_to_anchor=(0.04, 1.0))
    ipanel(ax, "(d)")
    # (e) speedbrake + gear
    ax = axs[4]
    ax.plot(t, L["sb_cmd"], color=C["orange"], lw=1.0, label=r"减速板指令 $\delta_{sb}$")
    ax.fill_between(t, 0, L["gear"] * 98.6, color=C["gray"], alpha=0.25, lw=0, step="post", label="起落架放下")
    ax.set_ylabel(r"$\delta_{sb}$ (°)")
    ax.set_ylim(0, 105)
    ax.set_xlabel("时间 t (s)")
    ax.legend(loc="upper left", fontsize=6.3, ncol=2, bbox_to_anchor=(0.04, 1.0))
    ax.set_xlim(0, t[-1])
    ipanel(ax, "(e)")
    axs[0].text(t_td + 1.5, 2.2, "主轮接地", fontsize=6.3, rotation=90, va="bottom", color="#444444")
    save(fig, "flight_history")


def fig_ground_track(d):
    from matplotlib.colors import LightSource
    f = d["flight"]
    L = f["log"]
    rwy, ends = _rwy(d)
    T = d["tables"]
    fig, axs = plt.subplots(1, 2, figsize=(FIG_W, 78 * MM), gridspec_kw=dict(width_ratios=[1.45, 1], wspace=0.2))
    ax = axs[0]
    g, Hgrid = f["terrain"]
    ls = LightSource(azdeg=315, altdeg=40)
    shade = ls.hillshade(Hgrid, vert_exag=3.0, dx=g[1] - g[0], dy=g[1] - g[0])
    rgb = np.dstack([0.55 + 0.42 * shade] * 3)
    sea = Hgrid <= 0.5
    rgb[sea] = (0.86, 0.91, 0.95)
    ext = [g[0] * 1e-3, g[-1] * 1e-3, g[0] * 1e-3, g[-1] * 1e-3]
    ax.imshow(rgb, extent=ext, origin="lower", interpolation="bilinear", zorder=0)
    cs = ax.contour(g * 1e-3, g * 1e-3, Hgrid, levels=[200, 500, 1000, 1500], colors="#6b6b6b",
                    linewidths=0.3, alpha=0.7, zorder=1)
    ax.clabel(cs, fontsize=5, fmt="%d m", inline_spacing=1)
    p0 = f["plans"][0]
    nd = np.array(p0["nodes"])
    ax.plot(nd[:, 1] * 1e-3, nd[:, 2] * 1e-3, color="white", lw=2.2, zorder=2)
    ax.plot(nd[:, 1] * 1e-3, nd[:, 2] * 1e-3, color=C["blue"], lw=0.8, ls=(0, (3, 1.5)), zorder=3,
            label="首次 SCP 基准")
    hac = f["plans"][-1]["hac"]
    ax.add_patch(Circle(np.array(hac["center"]) * 1e-3, hac["radius"] * 1e-3, fill=False, ec=C["teal"], lw=0.7,
                        ls=(0, (2, 1.5)), zorder=3))
    ax.text(hac["center"][0] * 1e-3, hac["center"][1] * 1e-3, "HAC", ha="center", va="center", fontsize=6.5,
            color=C["teal"], zorder=3)
    for t0, t1, lb, k0, k1 in _segments(L):
        col = PH_LINE.get(lb, C["blue"] if lb in ("ACQ", "HAC", "PRF") else C["gray"])
        ax.plot(L["E"][k0:k1 + 1] * 1e-3, L["N"][k0:k1 + 1] * 1e-3, color=col, lw=1.4, zorder=4,
                solid_capstyle="round")
    _draw_runway(ax, ends, lw=2.4)
    ax.plot(L["E"][0] * 1e-3, L["N"][0] * 1e-3, marker="*", ms=8, color=C["red"], zorder=7)
    ax.annotate("出生点  8.6 km, Mach 0.62", (L["E"][0] * 1e-3, L["N"][0] * 1e-3), xytext=(-33, 19),
                fontsize=6.3, arrowprops=dict(arrowstyle="-", lw=0.5, color="#333333"), zorder=8)
    ali = np.array(f["plans"][-1]["ali"]) * 1e-3
    ax.plot(ali[0], ali[1], marker="^", ms=5, color=C["orange"], zorder=7)
    ax.annotate("ALI", ali[:2], xytext=(ali[0] + 2.5, ali[1] + 2.0), fontsize=6.5,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#333333"), zorder=8)
    ax.annotate("RWY15", rwy.thr * 1e-3, xytext=(2.5, 6.5), fontsize=6.5,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#333333"), zorder=8)
    # scale bar + north arrow
    ax.plot([-1, 9], [-4.6, -4.6], color="#222222", lw=1.5, solid_capstyle="butt", zorder=8)
    ax.text(4, -3.9, "10 km", ha="center", fontsize=6.3, zorder=8)
    ax.annotate("", xy=(8.0, 18.2), xytext=(8.0, 14.6),
                arrowprops=dict(arrowstyle="-|>", lw=0.8, color="#222222", mutation_scale=7), zorder=8)
    ax.text(8.0, 18.6, "N", ha="center", va="bottom", fontsize=7, zorder=8)
    handles = [plt.Line2D([], [], color=C["blue"], lw=1.4), plt.Line2D([], [], color=C["orange"], lw=1.4),
               plt.Line2D([], [], color=C["red"], lw=1.4), plt.Line2D([], [], color=C["green"], lw=1.4),
               plt.Line2D([], [], color=C["purple"], lw=1.4),
               plt.Line2D([], [], color=C["blue"], lw=0.8, ls=(0, (3, 1.5)))]
    ax.legend(handles, ["TAEM", "OGS", "PREFLARE", "IGS", "FLARE / 滑跑", "首次 SCP 基准"], loc="lower left",
              fontsize=6.0, frameon=True, framealpha=0.9, edgecolor="none", ncol=2)
    ax.set_xlim(-36, 10)
    ax.set_ylim(-6, 22)
    ax.set_aspect("equal")
    ax.set_xlabel("东向 E (km)")
    ax.set_ylabel("北向 N (km)")
    panel(ax, "(a)", -0.17)
    # (b) energy corridor
    ax = axs[1]
    k_td = int(np.searchsorted(L["t"], f["touchdown"]["t"]))
    E, N = L["E"][:k_td + 1], L["N"][:k_td + 1]
    ds = np.hypot(np.diff(E), np.diff(N))
    s_togo = np.concatenate([np.cumsum(ds[::-1])[::-1], [0.0]])
    elev = rwy.elev
    Eh = L["h"][:k_td + 1] - elev + L["V"][:k_td + 1] ** 2 / (2 * G0)
    M = L["mach"][:k_td + 1]
    ldmax = np.interp(M, T["M"], T["ld_max"])
    ldmin = np.interp(M, T["M"], T["ld_min"])
    seg = np.concatenate([ds, [0.0]])
    dmin = np.concatenate([np.cumsum((seg / ldmax)[::-1])[::-1]])
    dmax = np.concatenate([np.cumsum((seg / ldmin)[::-1])[::-1]])
    Etd = Eh[-1]
    ax.fill_between(s_togo * 1e-3, (Etd + dmin) * 1e-3, (Etd + dmax) * 1e-3, color=C["blue"], alpha=0.10, lw=0)
    ax.plot(s_togo * 1e-3, (Etd + dmin) * 1e-3, color=C["blue"], lw=0.7, ls=(0, (4, 1.6)))
    ax.plot(s_togo * 1e-3, (Etd + dmax) * 1e-3, color=C["red"], lw=0.7, ls=(0, (4, 1.6)))
    for t0, t1, lb, k0, k1 in _segments({k: (v[:k_td + 1] if k != "sub" else v[:k_td + 1]) for k, v in L.items()}):
        col = PH_LINE.get(lb, C["blue"] if lb in ("ACQ", "HAC", "PRF") else C["gray"])
        ax.plot(s_togo[k0:k1 + 1] * 1e-3, Eh[k0:k1 + 1] * 1e-3, color=col, lw=1.4, solid_capstyle="round")
    ax.plot(s_togo * 1e-3, (L["h"][:k_td + 1] - elev) * 1e-3, color=C["ink"], lw=0.7, ls=(0, (1, 1.2)))
    def at(sv, arr):
        return (Etd + np.interp(sv * 1e3, s_togo[::-1], arr[::-1])) * 1e-3
    def ang(sv, arr):           # on-screen angle of a curve at s = sv km (reversed x axis)
        fig.canvas.draw()
        p0 = ax.transData.transform((sv + 1, at(sv + 1, arr)))
        p1 = ax.transData.transform((sv - 1, at(sv - 1, arr)))
        return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    ax.set_xlim(s_togo[0] * 1e-3 + 1, 0)
    ax.set_ylim(0, 15)
    ax.text(27, at(27, dmax) + 0.35, r"上界 $E_{h,\mathrm{td}}+\int ds/(L/D)_\min$", fontsize=6.4, color=C["red"],
            ha="center", va="bottom", rotation=ang(27, dmax), rotation_mode="anchor")
    ax.text(27, at(27, dmin) - 0.35, r"下界 $E_{h,\mathrm{td}}+\int ds/(L/D)_\max$", fontsize=6.4,
            color=C["blue"], ha="center", va="top", rotation=ang(27, dmin), rotation_mode="anchor")
    eh = lambda sv: np.interp(sv * 1e3, s_togo[::-1], Eh[::-1]) * 1e-3          # noqa: E731
    hh = lambda sv: np.interp(sv * 1e3, s_togo[::-1], (L["h"][:k_td + 1] - elev)[::-1]) * 1e-3  # noqa: E731
    ax.annotate("能量高度 $E_h$", (18, eh(18)), xytext=(19.5, 10.2), fontsize=6.6, color=C["blue"],
                arrowprops=dict(arrowstyle="-", lw=0.5, color=C["blue"]))
    ax.annotate("几何高度 $h$", (12, hh(12)), xytext=(19, 1.0), fontsize=6.6,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#333333"))
    ax.set_xlabel("沿实际航迹的剩余地面航程 (km)")
    ax.set_ylabel("相对跑道 (km)")
    ax.grid(lw=0.3)
    panel(ax, "(b)", -0.16)
    save(fig, "ground_track")


def fig_approach_profile(d):
    from sim.gnc.approach import PH_FLARE, PH_IGS, PH_OGS, PH_PREFLARE, vertical_profile
    from sim.gnc.geometry import ApproachConfig
    f = d["flight"]
    L = f["log"]
    rwy, _ = _rwy(d)
    cfg = ApproachConfig()
    H, phase, K = vertical_profile(cfg)
    xa, _ = rwy.to_rwy(L["E"], L["N"])
    ha = L["h"] - rwy.elev
    al = (L["phase"] >= 2) & (L["phase"] <= 3)
    fig, axs = plt.subplots(1, 2, figsize=(FIG_W, 64 * MM), gridspec_kw=dict(width_ratios=[1.25, 1], wspace=0.3))
    cols = {PH_OGS: C["orange"], PH_PREFLARE: C["red"], PH_IGS: C["green"], PH_FLARE: C["purple"]}
    x = np.linspace(cfg.x_ali - 300, K["x_td"] + 300, 6000)
    Hx, px = H(x), phase(x)
    for ax, xs, sc in ((axs[0], 1e-3, 1e-3), (axs[1], 1.0, 1.0)):
        for ph_, col in cols.items():
            k = px == ph_
            ax.plot(x[k] * xs, Hx[k] * sc, color=col, lw=2.6 if ax is axs[0] else 2.0, alpha=0.9,
                    solid_capstyle="butt")
        ax.plot(xa[al] * xs, ha[al] * sc, color=C["ink"], lw=0.8, ls=(0, (2.5, 1.2)))
    ax = axs[0]
    pts = [(K["x_ali"], 3000.0, f"ALI  {cfg.h_ali:.0f} m", (1.15, 0.05)),
           (K["x_pf"], K["H_pf"], f"拉起开始  {K['H_pf']:.0f} m", (1.1, 0.5))]
    for xk, hk, txt, (dx, dy) in pts:
        ax.plot(xk * 1e-3, hk * 1e-3, "o", ms=3.4, mfc="white", mec="#222222", mew=0.8, zorder=5)
        ax.annotate(txt, (xk * 1e-3, hk * 1e-3), xytext=(xk * 1e-3 + dx, hk * 1e-3 + dy), fontsize=6.4,
                    ha="left", va="center", arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.text(0.12, 0.55, "末段放大\n见 (b)", fontsize=6.2, ha="left", color="#555555")
    xo = np.array([cfg.x_ali, cfg.x_aim_ogs])
    ax.plot(xo * 1e-3, (cfg.x_aim_ogs - xo) * math.tan(math.radians(20)) * 1e-3, color="#999999", lw=0.5,
            ls=":", zorder=0)
    ax.annotate("OGS 瞄准点", (cfg.x_aim_ogs * 1e-3, 0), xytext=(-0.75, 0.26), fontsize=6.3, ha="center",
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.text(-7.2, 2.15, r"OGS  $\gamma=-20^\circ$", rotation=-28, fontsize=6.8, color=C["orange"])
    ax.axvline(0, color="#555555", lw=0.5)
    ax.text(0.08, 2.9, "跑道入口", fontsize=6.3, rotation=90, va="top")
    ax.set_xlim((cfg.x_ali - 500) * 1e-3, 1.4)
    ax.set_ylim(-0.08, 3.3)
    ax.set_xlabel("距跑道入口 x (km)")
    ax.set_ylabel("重心离跑道高度 H (km)")
    hs = [plt.Line2D([], [], color=c_, lw=2.4) for c_ in cols.values()] + \
         [plt.Line2D([], [], color=C["ink"], lw=0.8, ls=(0, (2.5, 1.2)))]
    ax.legend(hs, ["OGS", "预拉平圆弧 + 指数段", "IGS  −1.5°", "最终拉平", "6-DOF 实际（种子 1000）"],
              loc="lower left", fontsize=6.2)
    panel(ax, "(a)", -0.12)
    ax = axs[1]
    xi = np.array([-1300.0, cfg.x_aim_igs])
    ax.plot(xi, (cfg.x_aim_igs - xi) * math.tan(math.radians(1.5)), color="#999999", lw=0.6, ls=":")
    ax.text(-1180, 47.5, "IGS 延长线", fontsize=6.2, color="#777777", rotation=-5)
    ax.axhline(cfg.h_td_cg, color="#999999", lw=0.5, ls=(0, (4, 2)))
    ax.text(-1270, cfg.h_td_cg + 0.8, "接地姿态的重心高度", fontsize=6.0, ha="left", color="#777777")
    ax.axvline(0, color="#555555", lw=0.5)
    ax.axvline(f["touchdown"]["past_thr"], color=C["purple"], lw=0.5, ls=(0, (3, 1.5)))
    ax.text(f["touchdown"]["past_thr"] + 15, 40, f"实际接地 +{f['touchdown']['past_thr']:.0f} m", fontsize=6.2,
            rotation=90, color=C["purple"], va="top")
    ax.plot(K["x_flare"], K["H_flare"], "o", ms=3.4, mfc="white", mec="#222222", mew=0.8, zorder=5)
    ax.annotate(f"进入最终拉平\n{K['H_flare']:.1f} m", (K["x_flare"], K["H_flare"]), xytext=(-620, 9),
                fontsize=6.3, arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.set_xlim(-1300, 900)
    ax.set_ylim(0, 60)
    ax.set_xlabel("距跑道入口 x (m)")
    ax.set_ylabel("H (m)")
    ax.tick_params(right=False)
    a2 = ax.twinx()
    a2.set_ylim(0, 60 / FT)
    a2.set_ylabel("H (ft)")
    a2.tick_params(direction="in")
    panel(ax, "(b)", -0.16)
    save(fig, "approach_profile")


def fig_flare_law(d):
    from sim.gnc.approach import FF_HMIN, FF_HOFF, FF_K
    f = d["flight"]
    L = f["log"]
    t_ff = _event_t(f, "final flare")
    t_td = f["touchdown"]["t"]
    fig, axs = plt.subplots(1, 2, figsize=(FIG_W, 60 * MM), gridspec_kw=dict(wspace=0.3))
    ax = axs[0]
    hft = np.linspace(0, 100, 400)
    sink = FF_K * (np.maximum(hft * FT, FF_HMIN) + FF_HOFF) / FT
    ax.axhspan(30, 80, color=C["purple"], alpha=0.08, lw=0)
    ax.text(29.3, 72, "自动拉平进入高度带\n30–80 ft", fontsize=6.3, color=C["purple"], ha="right", va="center")
    k = hft <= 52
    ax.plot(sink[k], hft[k], color=C["purple"], lw=1.6, label="JSC-23266 下沉率–高度表")
    ax.plot(sink[~k], hft[~k], color=C["purple"], lw=1.0, ls=(0, (2, 1.4)), label="本仿真延伸到 80 ft 以上")
    for hh, ss in ((52, 13), (2, 3)):
        ax.plot(ss, hh, "o", ms=3.2, mfc="white", mec=C["purple"], mew=0.9, zorder=5)
        ax.annotate(f"{ss} fps @ {hh} ft", (ss, hh), xytext=(ss + 4, hh + (5 if hh > 10 else 6)), fontsize=6.3,
                    arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    m = (L["t"] >= t_ff - 6) & (L["t"] <= t_td + 0.05)
    ax.plot(-L["vs"][m] / FT, np.maximum(L["agl_gear"][m], 0) / FT, color=C["ink"], lw=1.0,
            label="6-DOF 实际（拉平前 6 s 起）")
    j = int(np.argmin(np.abs(L["t"] - t_ff)))
    ax.plot(-L["vs"][j] / FT, L["agl_gear"][j] / FT, "s", ms=3.4, color=C["ink"], zorder=6)
    ax.annotate(f"进入拉平\n{L['agl_gear'][j] / FT:.0f} ft, {-L['vs'][j] / FT:.1f} fps",
                (-L["vs"][j] / FT, L["agl_gear"][j] / FT), xytext=(18, 26), fontsize=6.3,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.set_xlim(0, 30)
    ax.set_ylim(0, 100)
    ax.set_xlabel(r"下沉率 $-\dot h$ (ft/s)")
    ax.set_ylabel("主轮离地高度 (ft)")
    ax.legend(loc="upper left", fontsize=6.1)
    panel(ax, "(a)", -0.13)
    ax = axs[1]
    h0 = float(L["agl_gear"][j])
    tt = np.linspace(0, 14, 300)
    ideal = (h0 + FF_HOFF) * np.exp(-FF_K * tt) - FF_HOFF
    ax.axhline(0, color="#222222", lw=0.6)
    ax.axhline(-FF_HOFF, color=C["red"], lw=0.7, ls=(0, (4, 1.6)))
    ax.text(13.8, -FF_HOFF + 0.25, r"渐近线 $h=-h_\mathrm{off}=-13$ ft", fontsize=6.4, color=C["red"], ha="right")
    ax.text(13.8, 0.25, "跑道面", fontsize=6.4, ha="right")
    ax.plot(tt, ideal, color=C["purple"], lw=1.2,
            label=r"理想解 $h+h_\mathrm{off}=(h_0+h_\mathrm{off})\,e^{-kt}$")
    m = (L["t"] >= t_ff) & (L["t"] <= t_td + 1.0)
    ax.plot(L["t"][m] - L["t"][j], L["agl_gear"][m], color=C["ink"], lw=1.0, label="6-DOF 实际主轮高度")
    ax.plot(t_td - L["t"][j], 0, "v", ms=4, color=C["ink"], zorder=6)
    ax.annotate(f"接地  下沉率 {f['touchdown']['sink']:.2f} m/s", (t_td - L["t"][j], 0),
                xytext=(t_td - L["t"][j] + 1.2, 4.5), fontsize=6.3,
                arrowprops=dict(arrowstyle="-", lw=0.5, color="#555555"))
    ax.set_xlim(0, 14)
    ax.set_ylim(-5.2, 14.5)
    ax.set_xlabel("进入最终拉平后的时间 (s)")
    ax.set_ylabel("主轮离地高度 (m)")
    ax.legend(loc="upper right", fontsize=6.1)
    panel(ax, "(b)", -0.13)
    save(fig, "flare_law")


def fig_landing_timeline(d):
    f = d["flight"]
    L = f["log"]
    t_td = f["touchdown"]["t"]
    tr = L["t"] - t_td
    m = (tr >= -45) & (tr <= 35)
    segs = [s_ for s_ in _segments(L) if s_[1] - t_td > -45]
    fig, axs = plt.subplots(4, 1, figsize=(FIG_W, 108 * MM), sharex=True,
                            gridspec_kw=dict(hspace=0.12, height_ratios=[1.2, 1, 1, 1]))
    evs = [("phase A/L / PREFLARE", "拉起"), ("gear down", "放起落架"), ("phase A/L / IGS", "IGS"),
           ("final flare", "拉平"), ("main gear touchdown", "主轮接地"), ("nose gear touchdown", "前轮接地"),
           ("drag chute jettison", "抛伞")]
    t_thr = L["t"][np.argmax(_rwy(d)[0].to_rwy(L["E"], L["N"])[0] >= 0)]
    evs_t = [(_event_t(f, a), b) for a, b in evs] + [(t_thr, "过入口")]
    for ax in axs:
        _shade(ax, segs, t_ref=t_td)
        for te, _ in evs_t:
            if te is not None:
                ax.axvline(te - t_td, color="#777777", lw=0.45, ls=(0, (2, 1.5)))
        ax.grid(axis="y")
    prev, row = -1e9, 0
    for te, lb in sorted((e for e in evs_t if e[0] is not None), key=lambda e: e[0]):
        row = 1 - row if te - prev < 3.5 else 0
        axs[0].text(te - t_td, 1.02 + 0.13 * row, lb, transform=axs[0].get_xaxis_transform(), fontsize=6.0,
                    ha="center", va="bottom", color="#333333")
        prev = te
    ax = axs[0]
    ax.semilogy(tr[m], np.maximum(L["agl_gear"][m], 0.03) / FT, color=C["ink"], lw=1.1)
    ax.set_ylim(0.3, 3000)
    ax.set_ylabel("主轮高度 (ft)")
    ipanel(ax, "(a)")
    ax = axs[1]
    ax.plot(tr[m], -L["vs"][m], color=C["ink"], lw=1.1)
    ax.set_ylim(-1, 58)
    ax.set_ylabel(r"下沉率 $-\dot h$ (m/s)")
    ins = ax.inset_axes([0.47, 0.38, 0.22, 0.55])
    k = (tr >= -8) & (tr <= 0.5)
    ins.plot(tr[k], -L["vs"][k], color=C["ink"], lw=0.9)
    ins.set_xlim(-8, 0.5)
    ins.set_ylim(0, 6)
    ins.tick_params(labelsize=5.5, length=2)
    ins.set_title("接地前 8 s", fontsize=6, pad=1.5)
    ipanel(ax, "(b)")
    ax = axs[2]
    ax.plot(tr[m], L["eas"][m] * KT, color=C["ink"], lw=1.0, label="KEAS")
    ax.plot(tr[m], L["gs"][m] * KT, color=C["teal"], lw=0.9, ls=(0, (4, 1.6)), label="地速")
    ax.set_ylabel("速度 (kt)")
    ax.set_ylim(0, 320)
    ax.legend(loc="upper right", fontsize=6.3, ncol=2)
    ipanel(ax, "(c)")
    ax = axs[3]
    ax.plot(tr[m], L["theta"][m], color=C["ink"], lw=1.0, label=r"俯仰角 $\theta$")
    ax.plot(tr[m], L["alpha"][m], color=C["red"], lw=0.9, ls=(0, (4, 1.6)), label=r"迎角 $\alpha$")
    ax.plot(tr[m], L["gamma"][m], color=C["blue"], lw=0.9, ls=(0, (1, 1.2)), label=r"航迹角 $\gamma$")
    ax.axhline(0, color="#999999", lw=0.4)
    ax.set_ylabel("角度 (°)")
    ax.set_ylim(-24, 16)
    ax.set_xlabel("相对主轮接地的时间 (s)")
    ax.set_xlim(-45, 35)
    ax.legend(loc="lower right", fontsize=6.3, ncol=3)
    ipanel(ax, "(d)")
    save(fig, "landing_timeline")


FIGS = dict(gnc_architecture=fig_gnc_architecture, flight_phases=fig_flight_phases, trim_envelope=fig_trim_envelope,
            scp_iterates=fig_scp_iterates, scp_flowchart=fig_scp_flowchart, flight_history=fig_flight_history,
            ground_track=fig_ground_track, approach_profile=fig_approach_profile, flare_law=fig_flare_law,
            landing_timeline=fig_landing_timeline, degraded_modes=fig_degraded_modes, fcs_ndi=fig_fcs_ndi)
NEED_DATA = {"trim_envelope", "scp_iterates", "flight_history", "ground_track", "approach_profile", "flare_law",
             "landing_timeline"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", choices=sorted(FIGS), help="render only these figures")
    ap.add_argument("--refresh", nargs="*", choices=list(SECTIONS) + ["all"],
                    help="recompute cached data sections (no names: all)")
    args = ap.parse_args()
    names = args.only or list(FIGS)
    refresh = () if args.refresh is None else (args.refresh or ["all"])
    d = load_data(refresh) if any(n in NEED_DATA for n in names) or args.refresh is not None else None
    for n in names:
        FIGS[n](d)


if __name__ == "__main__":
    main()
