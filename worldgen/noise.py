"""Vectorised procedural noise used by the world generator.

All functions operate on numpy float32 arrays.  Two families:
  * Perlin2   - infinite (hash based) 2-D gradient noise with analytic derivatives,
                used for the terrain heightmap.
  * tileable  - periodic 2-D Perlin and Worley noise for the cloud weather map (a texture that
                wraps seamlessly on the GPU; the 3-D cloud volumes: web/js/render/cloud_noise.js).
"""
from __future__ import annotations

import numpy as np

F32 = np.float32


def _fade(t):
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def _dfade(t):
    return 30.0 * t * t * (t * (t - 2.0) + 1.0)


class Perlin2:
    """2-D gradient noise, ~[-0.7, 0.7], with optional analytic derivatives."""

    def __init__(self, seed: int):
        rng = np.random.default_rng(seed)
        p = rng.permutation(256).astype(np.int32)
        self.perm = np.concatenate([p, p, p[:2]]).astype(np.int32)
        ang = rng.uniform(0.0, 2.0 * np.pi, 256)
        self.gx = np.cos(ang).astype(F32)
        self.gy = np.sin(ang).astype(F32)

    def __call__(self, x, y, deriv: bool = False):
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        fx = np.floor(x)
        fy = np.floor(y)
        xf = (x - fx).astype(F32)
        yf = (y - fy).astype(F32)
        xi = (fx.astype(np.int64) & 255).astype(np.int32)
        yi = (fy.astype(np.int64) & 255).astype(np.int32)
        perm = self.perm
        px0 = perm[xi]
        px1 = perm[xi + 1]
        h00 = perm[px0 + yi]
        h10 = perm[px1 + yi]
        h01 = perm[px0 + yi + 1]
        h11 = perm[px1 + yi + 1]
        gx, gy = self.gx, self.gy
        gx00, gy00 = gx[h00], gy[h00]
        gx10, gy10 = gx[h10], gy[h10]
        gx01, gy01 = gx[h01], gy[h01]
        gx11, gy11 = gx[h11], gy[h11]
        xm = xf - 1.0
        ym = yf - 1.0
        n00 = gx00 * xf + gy00 * yf
        n10 = gx10 * xm + gy10 * yf
        n01 = gx01 * xf + gy01 * ym
        n11 = gx11 * xm + gy11 * ym
        u = _fade(xf)
        v = _fade(yf)
        k1 = n10 - n00
        k2 = n01 - n00
        k3 = n00 - n10 - n01 + n11
        n = n00 + u * k1 + v * k2 + u * v * k3
        if not deriv:
            return n
        du = _dfade(xf)
        dv = _dfade(yf)
        dnx = (gx00 + u * (gx10 - gx00) + v * (gx01 - gx00) + u * v * (gx00 - gx10 - gx01 + gx11)
               + du * (k1 + k3 * v))
        dny = (gy00 + u * (gy10 - gy00) + v * (gy01 - gy00) + u * v * (gy00 - gy10 - gy01 + gy11)
               + dv * (k2 + k3 * u))
        return n, dnx, dny


_ROT = np.array([[0.80, -0.60], [0.60, 0.80]], dtype=np.float64)


def fbm(noise: Perlin2, x, y, octaves=5, lacunarity=2.0, gain=0.5):
    """Plain fBm, roughly in [-1, 1]."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    total = np.zeros(np.broadcast(x, y).shape, dtype=F32)
    amp = 1.0
    norm = 0.0
    for i in range(octaves):
        total += F32(amp) * noise(x, y)
        norm += amp
        amp *= gain
        x, y = (_ROT[0, 0] * x + _ROT[0, 1] * y) * lacunarity + 17.3, (_ROT[1, 0] * x + _ROT[1, 1] * y) * lacunarity - 5.1
    return total * F32(1.4 / norm)


def eroded_fbm(noise: Perlin2, x, y, octaves=9, gain=0.5, warp=1.0):
    """iq-style 'derivative damped' fBm: high octaves are suppressed on steep slopes,
    which gives a convincing eroded / drainage look.  Returns roughly [-0.6, 0.8]."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    shape = np.broadcast(x, y).shape
    a = np.zeros(shape, dtype=F32)
    dx = np.zeros(shape, dtype=F32)
    dy = np.zeros(shape, dtype=F32)
    b = 1.0
    for i in range(octaves):
        n, nx, ny = noise(x, y, deriv=True)
        dx += nx * F32(warp)
        dy += ny * F32(warp)
        a += F32(b) * n / (1.0 + dx * dx + dy * dy)
        b *= gain
        x, y = (_ROT[0, 0] * x + _ROT[0, 1] * y) * 2.0 + 3.1, (_ROT[1, 0] * x + _ROT[1, 1] * y) * 2.0 - 7.7
    return a


def ridged(noise: Perlin2, x, y, octaves=8, gain=0.5, offset=1.0):
    """Ridged multifractal (Musgrave), roughly [0, 1]."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    shape = np.broadcast(x, y).shape
    total = np.zeros(shape, dtype=F32)
    weight = np.ones(shape, dtype=F32)
    amp = 0.5
    norm = 0.0
    for i in range(octaves):
        n = noise(x, y)
        s = offset - np.abs(n) * 1.45
        s = s * s
        s *= weight
        weight = np.clip(s * 2.0, 0.0, 1.0).astype(F32)
        total += s * F32(amp)
        norm += amp
        amp *= gain
        x, y = (_ROT[0, 0] * x + _ROT[0, 1] * y) * 2.03 + 11.0, (_ROT[1, 0] * x + _ROT[1, 1] * y) * 2.03 + 2.0
    return total / F32(norm)


# ----------------------------------------------------------------------------- tileable noise
def perlin2_tileable(res: int, freq: int, seed: int):
    rng = np.random.default_rng(seed)
    ang = rng.uniform(0, 2 * np.pi, (freq, freq))
    g = np.stack([np.cos(ang), np.sin(ang)], -1).astype(F32)
    c = (np.arange(res, dtype=F32) + 0.5) / F32(res)
    yc, xc = np.meshgrid(c, c, indexing="ij")
    px, py = xc * freq, yc * freq
    ix, iy = np.floor(px).astype(np.int32), np.floor(py).astype(np.int32)
    fx, fy = px - ix, py - iy
    u, v = _fade(fx), _fade(fy)
    out = np.zeros(px.shape, dtype=F32)
    for dy in (0, 1):
        wy = v if dy else 1.0 - v
        for dx in (0, 1):
            wx = u if dx else 1.0 - u
            gi = g[(iy + dy) % freq, (ix + dx) % freq]
            out += wx * wy * (gi[..., 0] * (fx - dx) + gi[..., 1] * (fy - dy))
    return out * F32(1.45)


def worley2_tileable(res: int, freq: int, seed: int):
    rng = np.random.default_rng(seed)
    pts = rng.random((freq, freq, 2)).astype(F32)
    c = (np.arange(res, dtype=F32) + 0.5) / F32(res)
    yc, xc = np.meshgrid(c, c, indexing="ij")
    px, py = xc * freq, yc * freq
    ix, iy = np.floor(px).astype(np.int32), np.floor(py).astype(np.int32)
    best = np.full(px.shape, 1e9, dtype=F32)
    for oy in (-1, 0, 1):
        for ox in (-1, 0, 1):
            cx, cy = ix + ox, iy + oy
            f = pts[cy % freq, cx % freq]
            ddx = cx + f[..., 0] - px
            ddy = cy + f[..., 1] - py
            np.minimum(best, ddx * ddx + ddy * ddy, out=best)
    return np.clip(1.0 - np.sqrt(best), 0.0, 1.0).astype(F32)


def remap(v, lo, hi, nlo, nhi):
    return nlo + (v - lo) * (nhi - nlo) / (hi - lo)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)
