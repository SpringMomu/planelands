"""Tileable noise volumes for the volumetric cloud renderer (Nubis / Schneider style).

shape  128^3 RGBA8 : R = Perlin-Worley (base billows), G/B/A = Worley fBm at 3 increasing frequencies
detail  32^3 RGBA8 : R/G/B = Worley fBm at 3 increasing frequencies (edge erosion), A = 255
weather 512^2 RGBA8: R = coverage, G = secondary coverage / breakup, B = cloud type, A = height variation
"""
from __future__ import annotations

import time

import numpy as np

from .noise import F32, perlin2_tileable, perlin3_tileable, remap, worley2_tileable, worley3_tileable


def _u8(a):
    return np.clip(np.round(a * 255.0), 0, 255).astype(np.uint8)


def _wfbm(ws, i):
    return ws[i] * 0.625 + ws[i + 1] * 0.25 + ws[i + 2] * 0.125


def shape_volume(res=128, seed=11, log=print):
    t0 = time.time()
    perlin = (perlin3_tileable(res, 4, seed) * 0.5 + 0.5 * perlin3_tileable(res, 8, seed + 1) * 0.5
              + 0.25 * perlin3_tileable(res, 16, seed + 2) * 0.5)
    perlin = np.clip(perlin / 1.4 * 0.5 + 0.5, 0.0, 1.0)
    freqs = [4, 8, 16, 32, 64]
    ws = [worley3_tileable(res, f, seed + 10 + i) for i, f in enumerate(freqs)]
    log(f"    worley done {time.time()-t0:.1f}s")
    w_low = _wfbm(ws, 0)
    pw = np.clip(remap(perlin, 0.0, 1.0, w_low, 1.0), 0.0, 1.0)
    # normalise contrast
    pw = np.clip((pw - pw.min()) / (pw.max() - pw.min() + 1e-6), 0, 1)
    g = _wfbm(ws, 1)
    b = _wfbm(ws, 2)
    a = ws[3] * 0.625 + ws[4] * 0.375
    vol = np.stack([pw, g, b, a], -1)
    for k in range(4):
        ch = vol[..., k]
        vol[..., k] = (ch - ch.min()) / (ch.max() - ch.min() + 1e-6)
    log(f"    shape volume {time.time()-t0:.1f}s")
    return _u8(vol)


def detail_volume(res=32, seed=23):
    freqs = [2, 4, 8, 16, 32]
    ws = [worley3_tileable(res, f, seed + i) for i, f in enumerate(freqs)]
    r = _wfbm(ws, 0)
    g = _wfbm(ws, 1)
    b = _wfbm(ws, 2)
    vol = np.stack([r, g, b, np.ones_like(r)], -1)
    for k in range(3):
        ch = vol[..., k]
        vol[..., k] = (ch - ch.min()) / (ch.max() - ch.min() + 1e-6)
    return _u8(vol)


def weather_map(res=512, seed=37):
    def pfbm(f0, s, octs=4):
        acc = np.zeros((res, res), F32)
        amp, norm = 1.0, 0.0
        for o in range(octs):
            acc += amp * perlin2_tileable(res, f0 * 2 ** o, s + o)
            norm += amp
            amp *= 0.5
        return acc / norm

    def wfbm(f0, s):
        return (worley2_tileable(res, f0, s) * 0.625 + worley2_tileable(res, f0 * 2, s + 1) * 0.25
                + worley2_tileable(res, f0 * 4, s + 2) * 0.125)

    p = pfbm(4, seed) * 0.5 + 0.5
    w = wfbm(6, seed + 10)
    cov = np.clip(remap(p, 0.0, 1.0, w * 0.9, 1.0), 0, 1)
    cov = (cov - cov.min()) / (cov.max() - cov.min())
    p2 = pfbm(8, seed + 20, 3) * 0.5 + 0.5
    w2 = wfbm(12, seed + 30)
    cov2 = np.clip(remap(p2, 0.0, 1.0, w2, 1.0), 0, 1)
    cov2 = (cov2 - cov2.min()) / (cov2.max() - cov2.min())
    ctype = pfbm(2, seed + 40, 3) * 0.5 + 0.5
    ctype = (ctype - ctype.min()) / (ctype.max() - ctype.min())
    hvar = pfbm(3, seed + 50, 3) * 0.5 + 0.5
    hvar = (hvar - hvar.min()) / (hvar.max() - hvar.min())
    return _u8(np.stack([cov, cov2, ctype, hvar], -1))
