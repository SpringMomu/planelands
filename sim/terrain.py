"""Terrain height queries for ground contact (heightmap written by worldgen)."""
from __future__ import annotations

import json
import math
import os

import numpy as np

from .geometry import ROOT, WORLD_PATH, load_json


class Terrain:
    def __init__(self, world: dict | None = None, verbose=True):
        self.world = world or load_json(WORLD_PATH)
        rw = self.world["runway"]
        self.rwy_elev = float(rw["elevation"])
        h = math.radians(rw["heading_deg"])
        self.rwy_dir = np.array([math.sin(h), math.cos(h)])          # along (toward RWY33 direction)
        self.rwy_right = np.array([math.cos(h), -math.sin(h)])       # right of the RWY33 direction
        self.rwy_c = np.array(rw["center_en"], dtype=float)
        self.rwy_half_len = 0.5 * rw["length"]
        self.rwy_half_w = 0.5 * rw["width"]
        self.rwy_overrun = rw["overrun"]
        self.rwy_shoulder = rw["shoulder"]
        self._cx, self._cy = float(self.rwy_c[0]), float(self.rwy_c[1])
        self._dx, self._dy = float(self.rwy_dir[0]), float(self.rwy_dir[1])
        self._rx, self._ry = float(self.rwy_right[0]), float(self.rwy_right[1])
        self._lim_a = self.rwy_half_len + self.rwy_overrun
        self._lim_c = self.rwy_half_w + self.rwy_shoulder
        self.h = None
        self.size = 0
        self.extent = 1.0
        self.loaded = False
        self.reload(verbose)

    def reload(self, verbose=True):
        m = self.world["map"]
        meta_path = os.path.join(ROOT, m["files"]["meta"])
        h_path = os.path.join(ROOT, m["files"]["height"])
        size, extent = int(m["size"]), float(m["extent"])
        if os.path.exists(meta_path):
            try:
                meta = load_json(meta_path)
                size = int(meta.get("size", size))
                extent = float(meta.get("extent", extent))
            except Exception:
                pass
        if os.path.exists(h_path) and os.path.getsize(h_path) == size * size * 4:
            self.h = np.fromfile(h_path, dtype="<f4").reshape(size, size).astype(np.float32)
            self.size, self.extent = size, extent
            self.loaded = True
            if verbose:
                print(f"[sim] terrain loaded: {size}x{size}, {extent/1000:.1f} km, "
                      f"h {self.h.min():.0f}..{self.h.max():.0f} m")
        else:
            self.h = None
            self.loaded = False
            if verbose:
                print("[sim] WARNING: terrain heightmap not found - using flat ground at 0 m "
                      "(runway at its elevation)")

    # ------------------------------------------------------------------ runway frame
    def runway_frame(self, e, n):
        de, dn = e - self._cx, n - self._cy
        return de * self._dx + dn * self._dy, de * self._rx + dn * self._ry

    def on_runway(self, e, n, margin=0.0):
        de, dn = e - self._cx, n - self._cy
        return (abs(de * self._dx + dn * self._dy) <= self._lim_a + margin and
                abs(de * self._rx + dn * self._ry) <= self._lim_c + margin)

    # ------------------------------------------------------------------ heights
    def map_height(self, e, n):
        if not self.loaded:
            return 0.0
        s, L = self.size, self.extent
        fx = (e + 0.5 * L) / L * s - 0.5
        fy = (n + 0.5 * L) / L * s - 0.5
        fx = min(max(fx, 0.0), s - 1.001)
        fy = min(max(fy, 0.0), s - 1.001)
        i, j = int(fy), int(fx)
        ty, tx = fy - i, fx - j
        h = self.h
        h00, h01 = h[i, j], h[i, j + 1]
        h10, h11 = h[i + 1, j], h[i + 1, j + 1]
        return float((h00 * (1 - tx) + h01 * tx) * (1 - ty) + (h10 * (1 - tx) + h11 * tx) * ty)

    def height(self, e, n):
        """Ground (or water surface) height at (E, N): runway exact, else heightmap (>= sea level)."""
        if self.on_runway(e, n):
            return self.rwy_elev
        h = self.map_height(e, n)
        if not self.loaded:
            # flat world: blend runway elevation in its vicinity, 0 elsewhere
            a, c = self.runway_frame(e, n)
            if abs(a) < self.rwy_half_len + 1500 and abs(c) < 1000:
                return self.rwy_elev
            return 0.0
        return max(h, 0.0)
