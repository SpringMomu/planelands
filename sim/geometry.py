"""Orbiter geometry -> aerodynamic element arrays.

Reads ``shared/orbiter_geometry.json`` (body FRD frame, origin = CG) and builds the
structure-of-arrays used by :mod:`sim.aero`:

* lifting elements: wing strips (glove, main wing, carry-through) left/right, body-flap
  strips, vertical-tail strips.  Each strip has quarter-, half- and three-quarter-chord
  points, a unit normal ``n`` (lift-positive side), a span axis ``es`` (simple-sweep
  decomposition axis), a chordwise axis ``ect`` (perpendicular to ``es`` in the strip
  plane, pointing aft) and a coverage matrix that maps the control channels to the
  strip (fraction of the strip span covered by each flap).
* body elements: fuselage + OMS-pod slender-body segments with apparent-mass areas.
* gear, drag-chute points.

All vectors are stored as separate x/y/z component arrays for fast numpy evaluation.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEOM_PATH = os.path.join(ROOT, "shared", "orbiter_geometry.json")
WORLD_PATH = os.path.join(ROOT, "shared", "world.json")

# control channel order used everywhere in the aero / fcs code (radians)
CTRL_NAMES = ["elevon_li", "elevon_lo", "elevon_ri", "elevon_ro", "bodyflap", "rudder", "speedbrake"]
CH_LI, CH_LO, CH_RI, CH_RO, CH_BF, CH_RUD, CH_SB = range(7)

# component (group) names used for force breakdown / frontend overlay
GROUPS = ["wing_l", "wing_r", "elevon_li", "elevon_lo", "elevon_ri", "elevon_ro",
          "bodyflap", "vtail", "rudder_sb", "fuselage", "gear", "chute"]
G = {name: i for i, name in enumerate(GROUPS)}

SURF_WING, SURF_BF, SURF_VT = 0, 1, 2


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _interp_poly(pts, y):
    """piecewise-linear interpolation of x along a polyline given as [[x, y], ...] sorted by y."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return float(np.interp(y, ys, xs))


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


@dataclass
class LiftingElements:
    n: int = 0
    # points (N,) components
    qc: np.ndarray = None     # quarter chord (N,3)
    hc: np.ndarray = None     # half chord
    c34: np.ndarray = None    # three-quarter chord
    flap_pt: np.ndarray = None  # mid point of the flap portion (for overlay / hinge)
    nrm: np.ndarray = None
    es: np.ndarray = None
    ect: np.ndarray = None
    ep: np.ndarray = None     # nose-up pitch axis of the section (= n x ect)
    S: np.ndarray = None
    chord: np.ndarray = None
    tc: np.ndarray = None
    sweep: np.ndarray = None  # quarter-chord sweep (rad), geometric
    surf: np.ndarray = None   # SURF_*
    group: np.ndarray = None  # base group index
    eff: np.ndarray = None    # lift effectiveness multiplier (carry-through etc.)
    Kv: np.ndarray = None     # Polhamus vortex-lift constant
    suction: np.ndarray = None  # subsonic leading-edge suction fraction
    a_stall: np.ndarray = None  # stall / vortex-breakdown angle (rad, in the section plane)
    cf: np.ndarray = None     # flap chord fraction
    fmap: np.ndarray = None   # (N, 6) coverage of channels li, lo, ri, ro, bf, rudder
    flap_group: np.ndarray = None  # group index for the flap increment (or -1)
    rud_area: np.ndarray = None    # rudder/speedbrake panel planform area in the strip
    ground_effect: np.ndarray = None  # bool
    xnose_dist: np.ndarray = None  # streamwise distance of the strip LE from the body nose (for Re)


@dataclass
class BodyElements:
    # boundaries
    bx: np.ndarray = None
    bz: np.ndarray = None
    by: np.ndarray = None
    Az: np.ndarray = None     # apparent-mass area for vertical crossflow  (pi * (w/2)^2)
    Ay: np.ndarray = None     # apparent-mass area for lateral crossflow   (pi * (h/2)^2)
    # segments
    fore: np.ndarray = None   # index of the fore boundary
    aft: np.ndarray = None
    pos: np.ndarray = None    # (M,3) segment centre
    dx: np.ndarray = None
    width: np.ndarray = None
    height: np.ndarray = None
    perim: np.ndarray = None
    xdist: np.ndarray = None  # distance from the nose of its chain (Reynolds length)
    contract: np.ndarray = None  # Munk factor (1 expanding, <1 contracting)
    is_fus: np.ndarray = None    # 1 for fuselage segments, 0 for OMS pods
    nose_w: np.ndarray = None    # weight of forebody wave drag on this segment
    base_area: float = 0.0
    base_pos: np.ndarray = None
    pod_base_area: float = 0.0
    amax: float = 0.0


@dataclass
class Geometry:
    raw: dict
    world: dict
    S: float
    b: float
    cbar: float
    mass: float
    J: np.ndarray
    Jinv: np.ndarray
    lift: LiftingElements
    body: BodyElements
    gear_pts: dict
    gear_drag_pts: np.ndarray
    gear_cda: np.ndarray
    chute_attach: np.ndarray
    chute_cds: float
    limits: dict
    rates: dict
    elements: list = field(default_factory=list)       # frontend element list
    element_slices: dict = field(default_factory=dict)  # how aero arrays map onto `elements`


def build_geometry(path: str = GEOM_PATH) -> Geometry:
    g = load_json(path)
    world = load_json(WORLD_PATH)
    ref = g["reference"]
    ms = g["mass"]
    J = np.array([[ms["Ixx"], 0.0, -ms["Ixz"]],
                  [0.0, ms["Iyy"], 0.0],
                  [-ms["Ixz"], 0.0, ms["Izz"]]])

    lift = _build_lifting(g)
    body = _build_body(g)

    gg = g["gear"]
    gear_pts = {k: np.array([gg[k]["x"], gg[k]["y"], gg[k]["z"]]) for k in ("nose", "main_l", "main_r")}
    gear_drag_pts = np.array([[gg["nose"]["x"], 0.0, 3.5],
                              [gg["main_l"]["x"], gg["main_l"]["y"], 3.7],
                              [gg["main_r"]["x"], gg["main_r"]["y"], 3.7]])
    gear_cda = np.array([1.10, 1.60, 1.60])   # m^2 equivalent flat-plate area (dCD ~ 0.017)
    ch = g["drag_chute"]
    chute_cds = ch["CD"] * math.pi * (ch["diameter"] / 2.0) ** 2

    w = g["wing"]
    bf = g["bodyflap"]
    vt = g["vtail"]
    d2r = math.radians
    limits = {
        "elevon": (d2r(w["elevon_limits_deg"][0]), d2r(w["elevon_limits_deg"][1])),
        "bodyflap": (d2r(bf["limits_deg"][0]), d2r(bf["limits_deg"][1])),
        "rudder": (d2r(vt["rudder"]["limits_deg"][0]), d2r(vt["rudder"]["limits_deg"][1])),
        "speedbrake": (0.0, d2r(vt["speedbrake_max_deg"])),
    }
    rates = {
        "elevon": d2r(w["elevon_rate_dps"]),
        "bodyflap": d2r(bf["rate_dps"]),
        "rudder": d2r(vt["rudder"]["rate_dps"]),
        "speedbrake": d2r(vt["speedbrake_rate_dps"]),
    }
    geo = Geometry(raw=g, world=world, S=ref["S"], b=ref["b"], cbar=ref["cbar"],
                   mass=ms["m"], J=J, Jinv=np.linalg.inv(J), lift=lift, body=body,
                   gear_pts=gear_pts, gear_drag_pts=gear_drag_pts, gear_cda=gear_cda,
                   chute_attach=np.array(ch["attach"], dtype=float), chute_cds=chute_cds,
                   limits=limits, rates=rates)
    _build_element_list(geo)
    return geo


# --------------------------------------------------------------------------- lifting
def _build_lifting(g) -> LiftingElements:
    w = g["wing"]
    y_root = w["y_root"]
    z_root = w["z_root"]
    gam = math.radians(w["dihedral_deg"])
    le = sorted(w["le"], key=lambda p: p[1])
    te = sorted(w["te"], key=lambda p: p[1])
    tip = w["tip_y"]
    tcs = w["t_over_c"]
    y_kink = le[1][1]
    nose_x = g["fuselage"]["nose_x"]

    # reference trapezoid (used for the carry-through strips)
    def le_trap(y):
        return 3.19 - y

    def te_trap(y):
        return -14.32 + (2.12 / 11.9) * y

    rows = []   # list of dicts (right side), mirrored afterwards

    def add_strip(ya, yb, kind):
        ym = 0.5 * (ya + yb)
        dy = yb - ya
        if kind == "carry":
            xle, xte = le_trap(ym), te_trap(ym)
            dle = -1.0
            dte = 2.12 / 11.9
            z = z_root
            Gm = 0.0
        else:
            xle, xte = _interp_poly(le, ym), _interp_poly(te, ym)
            # local LE slope
            if ym < y_kink:
                dle = (le[1][0] - le[0][0]) / (le[1][1] - le[0][1])
            else:
                dle = (le[2][0] - le[1][0]) / (le[2][1] - le[1][1])
            dte = (te[1][0] - te[0][0]) / (te[1][1] - te[0][1])
            z = z_root - (ym - y_root) * math.tan(gam)
            Gm = gam
        c = xle - xte
        dqc = 0.75 * dle + 0.25 * dte
        sweep = math.atan(-dqc)          # positive = aft sweep
        tc = float(np.interp(ym, [p[0] for p in tcs], [p[1] for p in tcs]))
        rows.append(dict(ya=ya, yb=yb, ym=ym, dy=dy, xle=xle, c=c, z=z, gam=Gm,
                         sweep=sweep, tc=tc, kind=kind))

    # carry-through (inside fuselage) 2 strips, glove 3 strips, main wing 11 strips
    for ya, yb in ((0.0, 1.3), (1.3, y_root)):
        add_strip(ya, yb, "carry")
    gl = np.linspace(y_root, y_kink, 4)
    for i in range(3):
        add_strip(gl[i], gl[i + 1], "glove")
    mw = np.linspace(y_kink, tip, 12)
    for i in range(11):
        add_strip(mw[i], mw[i + 1], "main")

    elev = w["elevons"]   # right side ri / ro
    pts = {k: [] for k in ("qc", "hc", "c34", "flap", "n", "es", "ect")}
    arr = {k: [] for k in ("S", "chord", "tc", "sweep", "surf", "group", "eff", "Kv", "suction",
                           "a_stall", "cf", "fgroup", "rud_area", "ge", "xn")}
    fmap = []

    def push(qc, hc, c34, flap, n, es, ect, S, chord, tc, sweep, surf, group, eff, Kv, suc,
             ast, cf, fm, fgroup, rud_area, ge, xn):
        for k, v in (("qc", qc), ("hc", hc), ("c34", c34), ("flap", flap), ("n", n), ("es", es), ("ect", ect)):
            pts[k].append(np.asarray(v, dtype=float))
        for k, v in (("S", S), ("chord", chord), ("tc", tc), ("sweep", sweep), ("surf", surf),
                     ("group", group), ("eff", eff), ("Kv", Kv), ("suction", suc), ("a_stall", ast),
                     ("cf", cf), ("fgroup", fgroup), ("rud_area", rud_area), ("ge", ge), ("xn", xn)):
            arr[k].append(v)
        fmap.append(fm)

    for side in (-1, 1):   # left first, then right
        for r in rows:
            y = side * r["ym"]
            xle, c, z = r["xle"], r["c"], r["z"]
            lam_e = min(r["sweep"], math.radians(50.0))   # capped sweep for the decomposition
            sL, cL = math.sin(lam_e), math.cos(lam_e)
            sG, cG = math.sin(r["gam"]), math.cos(r["gam"])
            es = np.array([-sL, side * cL * cG, -cL * sG])
            n = np.array([0.0, -side * sG, -cG])
            ect = np.cross(es, n)
            if ect[0] > 0:
                ect = -ect
            ect = _unit(ect)
            # elevon coverage
            fm = [0.0] * 6
            cf_acc = 0.0
            cov_tot = 0.0
            fgroup = -1
            best = 0.0
            for k, e in enumerate(elev):
                ov = max(0.0, min(r["yb"], e["y1"]) - max(r["ya"], e["y0"]))
                if ov <= 0:
                    continue
                f = ov / r["dy"]
                ch = (0 if side < 0 else 2) + k   # li, lo, ri, ro
                fm[ch] = f
                cf_acc += f * e["chord"] / c
                cov_tot += f
                if f > best:
                    best = f
                    fgroup = G[("elevon_l" if side < 0 else "elevon_r") + ("i" if k == 0 else "o")]
            cf = cf_acc / cov_tot if cov_tot > 0 else 0.0
            flap_x = xle - c * (1.0 - 0.5 * cf) if cf > 0 else xle - c
            kind = r["kind"]
            eff = {"carry": 0.30, "glove": 1.0, "main": 1.0}[kind]
            Kv = {"carry": 0.4, "glove": 2.4, "main": 0.8}[kind]
            suc = {"carry": 0.5, "glove": 0.0, "main": 0.72}[kind]
            ast = math.radians({"carry": 40.0, "glove": 55.0, "main": 38.0}[kind])
            group = G["wing_l"] if side < 0 else G["wing_r"]
            push(qc=[xle - 0.25 * c, y, z], hc=[xle - 0.5 * c, y, z], c34=[xle - 0.75 * c, y, z],
                 flap=[flap_x, y, z], n=n, es=es, ect=ect, S=c * r["dy"], chord=c, tc=r["tc"],
                 sweep=r["sweep"], surf=SURF_WING, group=group, eff=eff, Kv=Kv, suc=suc, ast=ast,
                 cf=cf, fm=fm, fgroup=fgroup, rud_area=0.0, ge=(kind != "carry"),
                 xn=max(nose_x - xle, 1.0) if kind == "glove" else max(c * 0.5, 1.0))

    # ---------------- body flap: 3 strips across the span (all-moving surface)
    bf = g["bodyflap"]
    hs = bf["half_span"]
    edges = np.linspace(-hs, hs, 4)
    for i in range(3):
        ya, yb = edges[i], edges[i + 1]
        ym = 0.5 * (ya + yb)
        c = bf["chord"]
        xle = bf["hinge_x"]
        z = bf["z"]
        fm = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        push(qc=[xle - 0.25 * c, ym, z], hc=[xle - 0.5 * c, ym, z], c34=[xle - 0.75 * c, ym, z],
             flap=[xle - 0.5 * c, ym, z], n=[0.0, 0.0, -1.0], es=[0.0, 1.0, 0.0], ect=[-1.0, 0.0, 0.0],
             S=c * (yb - ya), chord=c, tc=bf["thickness"] / c, sweep=0.0, surf=SURF_BF,
             group=G["bodyflap"], eff=1.0, Kv=0.0, suc=0.3, ast=math.radians(24.0), cf=1.0, fm=fm,
             fgroup=-1, rud_area=0.0, ge=False, xn=nose_x - xle)

    # ---------------- vertical tail: 8 strips along the span
    vt = g["vtail"]
    rt, tp = vt["root"], vt["tip"]
    rud = vt["rudder"]
    zr, zt = rt["z"], tp["z"]
    nst = 8
    zs = np.linspace(zr, zt, nst + 1)
    span = abs(zt - zr)
    qroot = rt["le_x"] - 0.25 * (rt["le_x"] - rt["te_x"])
    qtip = tp["le_x"] - 0.25 * (tp["le_x"] - tp["te_x"])
    es_vt = _unit([qtip - qroot, 0.0, zt - zr])
    sweep_vt = math.atan2(abs(qtip - qroot), span)
    n_vt = np.array([0.0, 1.0, 0.0])
    ect_vt = np.cross(es_vt, n_vt)
    if ect_vt[0] > 0:
        ect_vt = -ect_vt
    ect_vt = _unit(ect_vt)
    for i in range(nst):
        za, zb = zs[i], zs[i + 1]
        zm = 0.5 * (za + zb)
        f = (zm - zr) / (zt - zr)
        xle = rt["le_x"] + f * (tp["le_x"] - rt["le_x"])
        xte = rt["te_x"] + f * (tp["te_x"] - rt["te_x"])
        c = xle - xte
        dz = abs(zb - za)
        lo, hi = min(rud["z0"], rud["z1"]), max(rud["z0"], rud["z1"])
        ov = max(0.0, min(max(za, zb), hi) - max(min(za, zb), lo))
        cov = ov / dz
        cf = rud["chord_frac"] if cov > 0 else 0.0
        fm = [0.0, 0.0, 0.0, 0.0, 0.0, cov]
        push(qc=[xle - 0.25 * c, 0.0, zm], hc=[xle - 0.5 * c, 0.0, zm], c34=[xle - 0.75 * c, 0.0, zm],
             flap=[xle - c * (1.0 - 0.5 * cf) if cf > 0 else xte, 0.0, zm], n=n_vt, es=es_vt, ect=ect_vt,
             S=c * dz, chord=c, tc=vt["t_over_c"], sweep=sweep_vt, surf=SURF_VT, group=G["vtail"],
             eff=1.0, Kv=0.5, suc=0.5, ast=math.radians(30.0), cf=cf, fm=fm,
             fgroup=G["rudder_sb"] if cov > 0 else -1, rud_area=cf * c * dz * cov, ge=False,
             xn=max(c * 0.5, 1.0))

    L = LiftingElements()
    L.n = len(arr["S"])
    L.qc = np.array(pts["qc"])
    L.hc = np.array(pts["hc"])
    L.c34 = np.array(pts["c34"])
    L.flap_pt = np.array(pts["flap"])
    L.nrm = np.array(pts["n"])
    L.es = np.array(pts["es"])
    L.ect = np.array(pts["ect"])
    L.ep = np.cross(L.nrm, L.ect)
    for k, attr, dt in (("S", "S", float), ("chord", "chord", float), ("tc", "tc", float),
                        ("sweep", "sweep", float), ("surf", "surf", int), ("group", "group", int),
                        ("eff", "eff", float), ("Kv", "Kv", float), ("suction", "suction", float),
                        ("a_stall", "a_stall", float), ("cf", "cf", float), ("fgroup", "flap_group", int),
                        ("rud_area", "rud_area", float), ("ge", "ground_effect", bool), ("xn", "xnose_dist", float)):
        setattr(L, attr, np.array(arr[k], dtype=dt))
    L.fmap = np.array(fmap, dtype=float)
    return L


# --------------------------------------------------------------------------- body
def _build_body(g) -> BodyElements:
    fus = g["fuselage"]
    st = np.array(sorted(fus["stations"], key=lambda s: -s[0]))   # nose first
    nose_x, base_x = fus["nose_x"], fus["base_x"]
    Lb = nose_x - base_x
    nseg = 20
    xi = Lb * (np.arange(nseg + 1) / nseg) ** 1.5
    bx = nose_x - xi
    sx = st[::-1, 0]   # ascending x for np.interp
    hw = np.interp(bx, sx, st[::-1, 1])
    zt = np.interp(bx, sx, st[::-1, 2])
    zb = np.interp(bx, sx, st[::-1, 3])
    W = 2.0 * hw
    H = np.maximum(zb - zt, 0.0)
    zc = 0.5 * (zt + zb)

    # OMS pods (two chains of 4 segments)
    pod = g["oms_pods"]
    px = np.array([pod["x_front"], pod["x_front"] - 1.5, pod["x_front"] - 3.0, -8.0, pod["x_aft"]])
    pw_full, ph_full = 2.2, abs(pod["z_top"] - pod["z_bottom"])
    grow = np.clip((pod["x_front"] - px) / 3.0, 0.0, 1.0) ** 0.5
    pW = pw_full * grow
    pH = ph_full * grow
    pz = 0.5 * (pod["z_top"] + pod["z_bottom"])

    bxs, bys, bzs, Ws, Hs = [bx], [np.zeros_like(bx)], [zc], [W], [H]
    fore, aft, xdist = [], [], []
    for i in range(nseg):
        fore.append(i)
        aft.append(i + 1)
        xdist.append(nose_x - 0.5 * (bx[i] + bx[i + 1]))
    off = nseg + 1
    for side in (-1, 1):
        bxs.append(px)
        bys.append(np.full_like(px, side * pod["y"]))
        bzs.append(np.full_like(px, pz))
        Ws.append(pW)
        Hs.append(pH)
        for i in range(len(px) - 1):
            fore.append(off + i)
            aft.append(off + i + 1)
            xdist.append(pod["x_front"] - 0.5 * (px[i] + px[i + 1]))
        off += len(px)

    B = BodyElements()
    B.bx = np.concatenate(bxs)
    B.by = np.concatenate(bys)
    B.bz = np.concatenate(bzs)
    Wall = np.concatenate(Ws)
    Hall = np.concatenate(Hs)
    B.Az = math.pi * (Wall / 2.0) ** 2
    B.Ay = 0.9 * math.pi * (Hall / 2.0) ** 2
    B.fore = np.array(fore)
    B.aft = np.array(aft)
    B.pos = np.stack([0.5 * (B.bx[B.fore] + B.bx[B.aft]),
                      0.5 * (B.by[B.fore] + B.by[B.aft]),
                      0.5 * (B.bz[B.fore] + B.bz[B.aft])], axis=1)
    B.dx = np.abs(B.bx[B.fore] - B.bx[B.aft])
    B.width = 0.5 * (Wall[B.fore] + Wall[B.aft])
    B.height = 0.5 * (Hall[B.fore] + Hall[B.aft])
    B.perim = 1.08 * math.pi * 0.5 * (B.width + B.height)
    B.xdist = np.maximum(np.array(xdist), 0.5)
    dA = B.Az[B.aft] - B.Az[B.fore]
    B.contract = np.where(dA < 0.0, 0.35, 1.0)
    B.is_fus = (np.arange(len(fore)) < nseg).astype(float)
    # forebody wave-drag weight: fuselage segments with expanding area
    nw = np.where((np.arange(len(fore)) < nseg) & (dA > 0), dA, 0.0)
    B.nose_w = nw / max(nw.sum(), 1e-9)
    B.amax = float(0.80 * W.max() * H.max())
    B.base_area = float(0.80 * 0.85 * W[-1] * H[-1])
    B.base_pos = np.array([base_x, 0.0, zc[-1]])
    B.pod_base_area = 2.0 * 0.5 * pw_full * ph_full
    return B


# --------------------------------------------------------------------------- elements list
def _build_element_list(geo: Geometry):
    L, B = geo.lift, geo.body
    els = []
    sl = {}
    start = 0
    for i in range(L.n):
        els.append({"name": f"{GROUPS[L.group[i]]}_{i}", "group": GROUPS[L.group[i]],
                    "pos": [round(float(v), 3) for v in L.qc[i]]})
    sl["lift"] = (start, start + L.n)
    start += L.n
    # flap increment elements (only strips with a flap): elevons + rudder/speedbrake
    idx = np.where(L.flap_group >= 0)[0]
    for i in idx:
        els.append({"name": f"{GROUPS[L.flap_group[i]]}_{i}", "group": GROUPS[L.flap_group[i]],
                    "pos": [round(float(v), 3) for v in L.flap_pt[i]]})
    sl["flap"] = (start, start + len(idx))
    geo.lift_flap_idx = idx
    start += len(idx)
    nb = len(B.fore)
    for j in range(nb):
        els.append({"name": f"fuselage_{j}", "group": "fuselage",
                    "pos": [round(float(v), 3) for v in B.pos[j]]})
    sl["body"] = (start, start + nb)
    start += nb
    els.append({"name": "fuselage_base", "group": "fuselage", "pos": [round(float(v), 3) for v in B.base_pos]})
    sl["base"] = (start, start + 1)
    start += 1
    for k, nm in enumerate(("gear_nose", "gear_main_l", "gear_main_r")):
        els.append({"name": nm, "group": "gear", "pos": [round(float(v), 3) for v in geo.gear_drag_pts[k]]})
    sl["gear"] = (start, start + 3)
    start += 3
    els.append({"name": "chute", "group": "chute", "pos": [round(float(v), 3) for v in geo.chute_attach]})
    sl["chute"] = (start, start + 1)
    geo.elements = els
    geo.element_slices = sl


if __name__ == "__main__":
    geo = build_geometry()
    L = geo.lift
    print("lifting elements:", L.n, " body segments:", len(geo.body.fore), " total overlay elements:", len(geo.elements))
    wing = L.surf == SURF_WING
    print("wing strip area (both sides, incl. carry-through): %.1f m^2" % L.S[wing].sum())
    print("exposed wing area: %.1f m^2" % L.S[wing & (L.eff > 0.5)].sum())
    print("body flap area: %.2f  vtail area: %.2f  rudder area: %.2f" % (
        L.S[L.surf == SURF_BF].sum(), L.S[L.surf == SURF_VT].sum(), L.rud_area.sum()))
    for i in range(L.n):
        if L.surf[i] == SURF_WING and L.qc[i, 1] > 0:
            print(f"  y={L.qc[i,1]:6.2f} c={L.chord[i]:6.2f} xqc={L.qc[i,0]:7.2f} sweep={math.degrees(L.sweep[i]):5.1f}"
                  f" cf={L.cf[i]:.2f} fmap={L.fmap[i]}")
