"""Builds (and caches) every generated asset the web client and the simulator need.

    python -m worldgen.build [--force]

Files written to web/data/:
  terrain_height.f32    float32 LE heightmap (size x size, row 0 = south)
  terrain_landcover.u8  RGBA8 land cover (same grid)
  terrain_meta.json     metadata
  cloud_weather.u8      512^2 RGBA8 weather map (the 3-D cloud noise is generated in the browser)
  assets.json           manifest (versions / sizes) read by the web client
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "web" / "data"
WORLD = ROOT / "shared" / "world.json"
ASSET_VERSION = 3


def _load_world():
    with open(WORLD, "r", encoding="utf-8") as f:
        return json.load(f)


def _manifest_ok():
    m = DATA / "assets.json"
    if not m.exists():
        return False
    try:
        man = json.loads(m.read_text(encoding="utf-8"))
    except Exception:
        return False
    if man.get("version") != ASSET_VERSION:
        return False
    if man.get("world_mtime") != int(WORLD.stat().st_mtime):
        return False
    for name in man.get("files", {}):
        if not (DATA / name).exists():
            return False
    return True


def _copy_shared():
    """The web client reads the shared JSON definitions from web/data/."""
    DATA.mkdir(parents=True, exist_ok=True)
    for name in ("world.json", "orbiter_geometry.json"):
        src = ROOT / "shared" / name
        if src.exists():
            (DATA / name).write_bytes(src.read_bytes())


def ensure_assets(force: bool = False, log=print) -> Path:
    """Generate the assets if missing/outdated. Returns the data directory."""
    _copy_shared()
    if not force and _manifest_ok():
        return DATA
    from . import clouds, terrain

    DATA.mkdir(parents=True, exist_ok=True)
    world = _load_world()
    t0 = time.time()
    log("[worldgen] generating terrain ...")
    h, lc, meta = terrain.generate(world, log=log)
    (DATA / "terrain_height.f32").write_bytes(np.ascontiguousarray(h, dtype="<f4").tobytes())
    (DATA / "terrain_landcover.u8").write_bytes(np.ascontiguousarray(lc).tobytes())
    (DATA / "terrain_meta.json").write_text(json.dumps(meta), encoding="utf-8")
    log("[worldgen] generating the cloud weather map ...")
    weather = clouds.weather_map(512)
    (DATA / "cloud_weather.u8").write_bytes(np.ascontiguousarray(weather).tobytes())
    man = {
        "version": ASSET_VERSION,
        "world_mtime": int(WORLD.stat().st_mtime),
        "files": {
            "terrain_height.f32": {"type": "f32", "size": [meta["size"], meta["size"]]},
            "terrain_landcover.u8": {"type": "rgba8", "size": [meta["size"], meta["size"]]},
            "terrain_meta.json": {"type": "json"},
            "cloud_weather.u8": {"type": "rgba8", "size": [512, 512]},
        },
        "build_seconds": round(time.time() - t0, 1),
    }
    (DATA / "assets.json").write_text(json.dumps(man, indent=1), encoding="utf-8")
    log(f"[worldgen] done in {time.time()-t0:.1f}s -> {DATA}")
    return DATA


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    ensure_assets(force=a.force)
