"""Weather map for the volumetric cloud renderer (Nubis / Schneider style).

weather 512^2 RGBA8: R = coverage, G = secondary coverage / breakup, B = cloud type, A = height variation

The tileable shape (128^3) and detail (32^3) noise volumes are generated in the browser
(web/js/render/cloud_noise.js) instead of being downloaded; only the weather map is a file, because
the CPU side (storm forcing of the simulator's wind) reads it and it must be identical everywhere.
"""
from __future__ import annotations

import numpy as np

from .noise import F32, perlin2_tileable, remap, worley2_tileable


def _u8(a):
    return np.clip(np.round(a * 255.0), 0, 255).astype(np.uint8)


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
