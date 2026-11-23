"""Procedural terrain for the shuttle-landing world.

Layout (ENU, origin = runway centre):
  * east  : open ocean, a barrier island with dunes/beaches and a shallow lagoon
            behind it (Florida-space-coast-like), the airport sits on the mainland plain
  * centre: coastal plain with farmland, forests, towns, two rivers and lakes
  * west  : rolling eroded hills, then a 3+ km high ridged mountain range
            (plus a north-western spur) with snow.

Outputs a float32 heightmap and an RGBA8 land-cover map:
  R forest density, G farmland, B urban density, A water depth (decimetres).

The far field outside the map is an analytic function (far_height) that the GLSL
terrain shader mirrors exactly, and the map edge is blended into it.
"""
from __future__ import annotations

import math
import time

import numpy as np
from scipy import ndimage

from .noise import F32, Perlin2, eroded_fbm, fbm, ridged, smoothstep


# ---------------------------------------------------------------- analytic far field (mirrored in GLSL)
def coast_x(y):
    return 14000.0 + 4000.0 * np.sin(y / 23000.0 + 0.4) + 2500.0 * np.sin(y / 9100.0 + 1.3)


def far_height(x, y):
    d = x - coast_x(y)
    inland = np.maximum(-d - 5000.0, 0.0)
    land = 6.0 + 0.0009 * inland
    hills = 140.0 * smoothstep(15000.0, 40000.0, inland) * (0.55 + 0.45 * np.sin(x / 7000.0) * np.sin(y / 5300.0))
    mount = smoothstep(50000.0, 80000.0, inland) * (1500.0 + 700.0 * np.sin(y / 13000.0 + 0.7) * np.sin(x / 9000.0 + 1.1))
    hl = land + hills + mount
    hs = np.maximum(-8.0 - 0.004 * np.maximum(d, 0.0), -150.0)
    return hl + (hs - hl) * smoothstep(-300.0, 600.0, d)


# ---------------------------------------------------------------- helpers
def _runway_frame(world, X, Y):
    rw = world["runway"]
    hdg = math.radians(rw["heading_deg"])
    dx, dy = math.sin(hdg), math.cos(hdg)          # along (toward RWY33 direction)
    rx, ry = math.cos(hdg), -math.sin(hdg)         # right of the landing direction
    cx, cy = rw["center_en"]
    along = (X - cx) * dx + (Y - cy) * dy
    across = (X - cx) * rx + (Y - cy) * ry
    return along, across


def _river_path(p0, p1, amps, phases, n=24000):
    t = np.linspace(0.0, 1.0, n)
    x = p0[0] + (p1[0] - p0[0]) * t
    y = p0[1] + (p1[1] - p0[1]) * t
    taper = np.maximum(np.sin(np.pi * np.clip(t, 0, 1)), 0.0) ** 0.35
    for (a, f), ph in zip(amps, phases):
        y = y + a * np.sin(2 * np.pi * f * t + ph) * taper
        x = x + 0.35 * a * np.cos(2 * np.pi * f * 0.7 * t + ph * 1.3) * taper
    return x, y


def _carve_river(h, water, path_x, path_y, res, L, hw0, hw1, depth0, depth1, log):
    N = h.shape[0]
    cols = np.floor((path_x + L / 2) / res).astype(np.int64)
    rows = np.floor((path_y + L / 2) / res).astype(np.int64)
    ok = (cols >= 0) & (cols < N) & (rows >= 0) & (rows < N)
    cols, rows = cols[ok], rows[ok]
    px, py = path_x[ok], path_y[ok]
    # truncate the path once it reaches open water (lagoon / sea)
    hp = h[rows, cols]
    wet = np.nonzero(hp < -0.4)[0]
    if len(wet):
        end = wet[0] + 60
        cols, rows, px, py, hp = cols[:end], rows[:end], px[:end], py[:end], hp[:end]
    seg = np.hypot(np.diff(px), np.diff(py))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    S = s[-1]
    hs = ndimage.gaussian_filter1d(hp.astype(np.float64), 25)
    z = np.minimum.accumulate(hs - 3.5)
    z = np.maximum(z, 0.15 + 0.0002 * (S - s))
    frac = s / max(S, 1.0)
    hw = hw0 + (hw1 - hw0) * frac ** 1.2
    dep = depth0 + (depth1 - depth0) * frac

    zgrid = np.full(h.shape, np.inf, dtype=np.float64)
    np.minimum.at(zgrid, (rows, cols), z)
    hwgrid = np.zeros(h.shape, dtype=np.float64)
    np.maximum.at(hwgrid, (rows, cols), hw)
    dgrid = np.zeros(h.shape, dtype=np.float64)
    np.maximum.at(dgrid, (rows, cols), dep)
    mask = np.isfinite(zgrid)
    dist, (ri, ci) = ndimage.distance_transform_edt(~mask, sampling=res, return_indices=True)
    near = dist < 14000.0
    zn = zgrid[ri, ci]
    hwn = hwgrid[ri, ci]
    dn = dgrid[ri, ci]
    bed = zn - dn * (1.0 - np.clip(dist / hwn, 0, 1) ** 2)
    e = np.maximum(dist - hwn, 0.0)
    bank = zn + 0.6 + 0.012 * e + 0.08 * np.maximum(e - 1500.0, 0.0) + 0.17 * np.maximum(e - 5000.0, 0.0)
    target = np.where(dist < hwn, bed, bank)
    hn = np.where(near, np.minimum(h, target), h).astype(F32)
    wd = np.where(dist < hwn * 1.1, np.maximum(zn - hn, 0.0), 0.0)
    water = np.maximum(water, wd.astype(F32))
    log(f"    river: length {S/1000:.1f} km, source surface {z[0]:.0f} m")
    return hn, water, dist.astype(F32)


def generate(world: dict, seed: int = 7, log=print):
    t0 = time.time()
    N = int(world["map"]["size"])
    L = float(world["map"]["extent"])
    res = L / N
    c = (-L / 2 + (np.arange(N) + 0.5) * res).astype(np.float64)
    X, Y = np.meshgrid(c, c)          # X[row, col] = east(col), Y[row, col] = north(row)
    P = [Perlin2(seed * 101 + i) for i in range(32)]
    log(f"  terrain {N}x{N} @ {res:.0f} m/texel")

    # ---- coastline (1-D noise along north, evaluated on the column vector and broadcast)
    cy = c[:, None]
    pert1d = 2600.0 * fbm(P[0], cy / 16000.0, 0.37, 4)
    islandW = (1300.0 + 500.0 * fbm(P[2], cy / 7000.0, 1.7, 3)).astype(F32)
    lagoonW = (3300.0 + 1500.0 * fbm(P[3], cy / 11000.0, 2.9, 3)).astype(F32)
    inlet = smoothstep(0.50, 0.62, fbm(P[4], cy / 2600.0, 4.4, 2)).astype(F32)
    sea_shore = coast_x(Y) + pert1d + 900.0 * fbm(P[1], X / 5000.0, Y / 5000.0, 4)
    d = (X - sea_shore).astype(F32)                   # > 0 : open sea
    dm = (-(d + islandW + lagoonW)).astype(F32)        # > 0 : inland from the lagoon shore
    warp = 7000.0 * fbm(P[5], X / 30000.0, Y / 30000.0, 3)
    dmw = (dm + warp).astype(F32)
    wx = X + warp
    wy = Y + 7000.0 * fbm(P[6], X / 30000.0 + 3.0, Y / 30000.0, 3)
    log(f"  coast done {time.time()-t0:.1f}s")

    # ---- mainland
    plain = (1.5 + 0.0011 * np.maximum(dm, 0.0)
             + 14.0 * fbm(P[7], X / 9000.0, Y / 9000.0, 5) * smoothstep(500.0, 6000.0, dm)
             + 3.0 * fbm(P[8], X / 1500.0, Y / 1500.0, 3) * smoothstep(0.0, 3000.0, dm)).astype(F32)
    h_main = plain.copy()

    hill_amp = (460.0 * smoothstep(9000.0, 34000.0, dmw)).astype(F32)
    sel = hill_amp > 0.5
    ef = eroded_fbm(P[9], X[sel] / 14000.0, Y[sel] / 14000.0, 8)
    h_main[sel] += hill_amp[sel] * np.clip(ef * 0.9 + 0.45, 0.0, None) ** 1.25
    log(f"  hills done {time.time()-t0:.1f}s")

    m_amp = smoothstep(34000.0, 64000.0, dmw)
    m_amp2 = smoothstep(22000.0, 42000.0, dmw) * smoothstep(48000.0, 70000.0, Y)
    m_amp = np.maximum(m_amp, 0.8 * m_amp2).astype(F32)
    sel = m_amp > 0.002
    rg = ridged(P[10], wx[sel] / 26000.0, wy[sel] / 26000.0, 9)
    ef2 = np.clip(eroded_fbm(P[11], X[sel] / 20000.0, Y[sel] / 20000.0, 9) + 0.4, 0.0, None)
    h_main[sel] += m_amp[sel] * (350.0 + 3100.0 * (0.62 * rg + 0.38 * ef2))
    log(f"  mountains done {time.time()-t0:.1f}s")

    # ---- barrier island, lagoon, sea
    t_lag = np.clip(-(d + islandW) / lagoonW, 0.0, 1.0)
    bowl = 4.0 * t_lag * (1.0 - t_lag)
    n_lag = fbm(P[12], X / 2500.0, Y / 2500.0, 3)
    h_lagoon = (-0.7 - 2.4 * bowl + 2.2 * np.maximum(n_lag - 0.35, 0.0) * 3.0).astype(F32)
    t_isl = np.clip(-d / islandW, 0.0, 1.0)
    bump = np.maximum(np.sin(np.pi * t_isl), 0.0) ** 0.7
    h_island = (0.6 + 6.0 * bump * (0.6 + 0.4 * fbm(P[13], X / 1200.0, Y / 1200.0, 3))
                - 0.8 * (1.0 - smoothstep(0.0, 0.12, t_isl))).astype(F32)
    h_island = h_island * (1.0 - inlet) + (-2.5) * inlet
    dpos = np.maximum(d, 0.0)
    h_sea = np.maximum(-1.0 - 28.0 * (1.0 - np.exp(-dpos / 2500.0)) - 0.0022 * dpos
                       + 2.5 * fbm(P[14], X / 6000.0, Y / 6000.0, 3), -160.0).astype(F32)

    w_sea = smoothstep(-60.0, 140.0, d)
    w_lag = smoothstep(-islandW + 90.0, -islandW - 140.0, d)
    w_main = smoothstep(-islandW - lagoonW + 160.0, -islandW - lagoonW - 260.0, d)
    h = h_sea * w_sea + (1.0 - w_sea) * (h_island * (1.0 - w_lag) + w_lag * (h_lagoon * (1.0 - w_main) + w_main * h_main))
    h = h.astype(F32)
    log(f"  coast blend done {time.time()-t0:.1f}s")

    # ---- rivers
    water = np.zeros_like(h)
    rng = np.random.default_rng(seed)
    rivers = []
    river_dist = np.full(h.shape, 1e9, dtype=F32)
    specs = [
        ((-88000.0, 16000.0), (30000.0, 10500.0), [(5200, 1.6), (1300, 6.8), (380, 19.0)], 70.0, 230.0, 3.0, 7.0),
        ((-64000.0, -44000.0), (30000.0, -29000.0), [(3500, 1.3), (1100, 7.7), (300, 21.0)], 55.0, 170.0, 2.5, 5.5),
        ((-58000.0, 47000.0), (-36000.0, 19000.0), [(2200, 1.1), (700, 6.1), (220, 17.0)], 45.0, 110.0, 2.0, 4.0),
    ]
    for (p0, p1, amps, hw0, hw1, d0, d1) in specs:
        ph = rng.uniform(0, 2 * np.pi, len(amps))
        rx, ry = _river_path(p0, p1, amps, ph)
        h, water, dist = _carve_river(h, water, rx, ry, res, L, hw0, hw1, d0, d1, log)
        river_dist = np.minimum(river_dist, dist)
        step = max(1, len(rx) // 400)
        rivers.append([[float(a), float(b)] for a, b in zip(rx[::step], ry[::step])])
    log(f"  rivers done {time.time()-t0:.1f}s")

    # ---- lakes
    lakes = [(-15500.0, -12500.0, 1900.0, 9.0), (-33000.0, 21000.0, 2600.0, 14.0),
             (9000.0, -45000.0, 2400.0, 8.0), (-52000.0, -12000.0, 1500.0, 12.0)]
    nl = fbm(P[15], X / 1700.0, Y / 1700.0, 3)
    for (lx, ly, R, dep) in lakes:
        r = np.hypot(X - lx, Y - ly)
        ang = np.arctan2(Y - ly, X - lx)
        rr = r / (R * (1.0 + 0.35 * np.sin(2.0 * ang + lx * 0.001) + 0.2 * np.sin(3.0 * ang + 1.0))) + 0.30 * nl
        inside = rr < 1.0
        if not inside.any():
            continue
        level = float(np.percentile(h[inside], 12)) + 0.8
        basin = level - dep * np.clip(1.0 - rr * rr, 0.0, 1.0) ** 0.8
        shore = level + 0.5 + np.maximum(rr - 1.0, 0.0) * R * 0.02
        target = np.where(rr < 1.0, basin, shore)
        zone = rr < 1.7
        h = np.where(zone, np.minimum(h, target), h).astype(F32)
        wd = np.where(rr < 1.05, np.maximum(level - h, 0.0), 0.0)
        water = np.maximum(water, wd.astype(F32))
    log(f"  lakes done {time.time()-t0:.1f}s")

    # ---- airport flattening
    along, across = _runway_frame(world, X, Y)
    fz = world["airport_flat_zone"]
    elev = float(world["runway"]["elevation"])
    ax = np.maximum(np.abs(along) - fz["half_along"], 0.0)
    cx_ = np.maximum(np.abs(across) - fz["half_across"], 0.0)
    dd = np.hypot(ax, cx_)
    amask = (1.0 - smoothstep(0.0, fz["blend"], dd)).astype(F32)
    h = (h * (1.0 - amask) + elev * amask).astype(F32)
    water[amask > 0.02] = 0.0
    ap_zone = (np.hypot(np.maximum(np.abs(along) - fz["half_along"] - 300, 0), np.maximum(np.abs(across) - fz["half_across"] - 200, 0)) < 1.0)

    # ---- blend into the analytic far field at the map border
    edge = np.maximum(np.abs(X), np.abs(Y))
    b = smoothstep(L / 2 - 14000.0, L / 2 - 1500.0, edge).astype(F32)
    h = (h * (1.0 - b) + far_height(X, Y) * b).astype(F32)
    water *= (1.0 - smoothstep(L / 2 - 14000.0, L / 2 - 8000.0, edge)).astype(F32)
    log(f"  airport + border done {time.time()-t0:.1f}s")

    # ---- land cover
    gy_, gx_ = np.gradient(h, res)
    slope = np.hypot(gx_, gy_).astype(F32)
    fn = fbm(P[20], X / 5000.0, Y / 5000.0, 5) + 0.25 * fbm(P[21], X / 1200.0, Y / 1200.0, 3)
    hillF = smoothstep(8000.0, 30000.0, dmw)
    riparian = 0.5 * (1.0 - np.clip(river_dist / 700.0, 0.0, 1.0))
    f_raw = fn + 0.30 * hillF + 0.45 * smoothstep(0.10, 0.40, slope) - 0.30 + riparian
    forest = smoothstep(0.0, 0.35, f_raw)
    forest *= 1.0 - smoothstep(1800.0 + 250.0 * fn, 2300.0 + 250.0 * fn, h)
    forest *= 1.0 - smoothstep(0.9, 1.3, slope)
    island_zone = (d < 0) & (d > -islandW)
    forest = np.where(island_zone, 0.45 * smoothstep(0.25, 0.6, t_isl) * (0.6 + 0.4 * fn), forest)

    # towns
    towns = [(5000.0, -36000.0, 5200.0), (-21000.0, 11500.0, 2400.0), (-46000.0, 14500.0, 1700.0),
             (-9000.0, -21000.0, 3000.0), (2500.0, 37000.0, 3800.0), (-30000.0, -7000.0, 1800.0),
             (-62000.0, -31000.0, 1500.0), (-15000.0, 53000.0, 2200.0), (8000.0, 16000.0, 1500.0),
             (-4000.0, -9000.0, 900.0)]
    # a beach town on the barrier island
    row = int((-14000.0 + L / 2) / res)
    drow = d[row]
    isl = np.nonzero((drow < -0.3 * islandW[row, 0]) & (drow > -0.7 * islandW[row, 0]))[0]
    if len(isl):
        towns.append((float(c[isl[len(isl) // 2]]), -14000.0, 1800.0))
    nu = fbm(P[23], X / 900.0, Y / 900.0, 3)
    urban = np.zeros_like(h)
    for (tx, ty, R) in towns:
        r = np.hypot(X - tx, Y - ty) / R
        u = np.clip(1.15 - r ** 1.4 + 0.38 * nu, 0.0, 1.0)
        urban = np.maximum(urban, u.astype(F32))
    urban *= (1.0 - smoothstep(0.12, 0.3, slope)).astype(F32)

    farm_raw = (0.45 - forest * 1.0 + 0.35 * fbm(P[22], X / 14000.0, Y / 14000.0, 3)
                - 0.6 * smoothstep(0.14, 0.30, slope) - smoothstep(600.0, 900.0, h))
    farm = smoothstep(0.0, 0.15, farm_raw) * smoothstep(300.0, 1500.0, dm)

    wet = water > 0.05
    sea = h < 0.0
    for arr in (forest, farm, urban):
        arr[wet | sea] = 0.0
        arr[ap_zone] = 0.0
    forest *= (1.0 - 0.8 * urban)
    farm *= (1.0 - urban)
    # fade land cover where the far field takes over (the shader does its own there)
    lc_fade = 1.0 - b
    lc = np.zeros(h.shape + (4,), dtype=np.uint8)
    lc[..., 0] = np.clip(forest * lc_fade * 255.0, 0, 255).astype(np.uint8)
    lc[..., 1] = np.clip(farm * lc_fade * 255.0, 0, 255).astype(np.uint8)
    lc[..., 2] = np.clip(urban * lc_fade * 255.0, 0, 255).astype(np.uint8)
    lc[..., 3] = np.clip(np.round(water * 10.0), 0, 255).astype(np.uint8)
    log(f"  landcover done {time.time()-t0:.1f}s, h range {h.min():.0f}..{h.max():.0f} m")

    meta = {
        "size": N,
        "extent": L,
        "texel": res,
        "seed": seed,
        "h_min": float(h.min()),
        "h_max": float(h.max()),
        "runway_elevation": elev,
        "towns": [[t[0], t[1], t[2]] for t in towns],
        "rivers": rivers,
        "lakes": [[l[0], l[1], l[2]] for l in lakes],
        "landcover_channels": "R forest, G farmland, B urban, A water depth [dm]",
        "far_field": "see worldgen/terrain.py far_height(); mirrored in web/js/terrain.js",
        "build_seconds": round(time.time() - t0, 1),
    }
    return h, lc, meta
