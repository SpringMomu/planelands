#!/usr/bin/env python
"""Build the static browser release: everything runs in the visitor's browser (rendering in WebGL,
the Python simulator + planner in Pyodide Web Workers); the server only hands out files.

    python tools/build_web.py [--out dist] [--serve 8090]

Layout of --out (deploy it as the web root; tools/deploy_web.sh does that for the jp server):

    index.html, version.json      small, revalidated on every visit (Cache-Control: no-cache)
    v/<ver>/...                   this release: js, css, vendor, small JSON data, py/sim.zip  (immutable)
    d/<sha>/<name>                large data files, content-addressed: unchanged data is never
                                  downloaded again across releases; the heightmap losslessly
                                  coded (terrain_height.plh, see encode_heights)            (immutable)
    pyodide/<version>/...         self-hosted Pyodide runtime + the wheels the simulator needs (immutable)

<ver> is a hash of the release contents, so building twice gives the same folder and a deploy only
uploads what changed. Older v/ folders stay until pruned, so pages opened before an update keep working.
Compressible files get a .gz sibling for nginx gzip_static.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import os
import re
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WEB = ROOT / "web"
sys.path.insert(0, str(ROOT))

PYODIDE_VERSION = "314.0.8"
PYODIDE_CDN = f"https://cdn.jsdelivr.net/pyodide/v{PYODIDE_VERSION}/full/"
PYODIDE_CORE = ["pyodide.mjs", "pyodide.asm.mjs", "pyodide.asm.wasm", "python_stdlib.zip", "pyodide-lock.json"]
PY_PACKAGES = ["numpy", "scipy", "cvxpy-base", "clarabel"]          # keep in sync with web/js/sim/pyboot.js
CACHE = ROOT / ".cache" / "pyodide" / PYODIDE_VERSION

BIG_DATA = ["terrain_height.f32", "terrain_landcover.u8", "cloud_weather.u8"]   # (cloud shape/detail: generated in the page)
SMALL_DATA = ["world.json", "orbiter_geometry.json", "terrain_meta.json", "assets.json"]
LAYOUT = 2              # bump when the generated index.html changes shape (it is part of <ver>)
GZ_EXT = {".html", ".js", ".mjs", ".css", ".json", ".wasm", ".f32", ".plh", ".u8", ".glsl", ".svg", ".txt", ".zip"}


def sha(data: bytes, n=12) -> str:
    return hashlib.sha256(data).hexdigest()[:n]


def log(*a):
    print("[build]", *a, flush=True)


# --------------------------------------------------------------------------- terrain coding
def encode_heights(raw: bytes, size: int) -> bytes:
    """Lossless PLH1 coding of the float32 heightmap (decoder: web/js/core/terrain_codec.js): uint32 bit
    patterns, planar prediction mod 2^32, zigzag, byte planes. ~56 % of the raw grid after gzip."""
    import numpy as np
    u = np.frombuffer(raw, dtype="<u4").reshape(size, size)
    pred = np.zeros_like(u)
    pred[0, 1:] = u[0, :-1]
    pred[1:, 0] = u[:-1, 0]
    pred[1:, 1:] = u[1:, :-1] + u[:-1, 1:] - u[:-1, :-1]          # uint32: wraps mod 2^32
    s = (u - pred).view(np.int32)
    z = ((s << 1) ^ (s >> 31)).view(np.uint32).ravel()
    planes = np.ascontiguousarray(z.view(np.uint8).reshape(-1, 4).T).tobytes()
    out = b"PLH1" + np.array([size, size, 0], dtype="<u4").tobytes() + planes
    if decode_heights(out) != raw:
        raise SystemExit("terrain coding is not lossless")
    return out


def decode_heights(buf: bytes) -> bytes:
    import numpy as np
    W, H, _ = np.frombuffer(buf[4:16], dtype="<u4")
    N = int(W) * int(H)
    z = np.frombuffer(buf[16:], dtype=np.uint8).reshape(4, N).T.copy().view("<u4").ravel()
    s = (z >> 1) ^ (np.uint32(0) - (z & 1))
    # planar predictor inverse = 2-D prefix sum of the residuals (mod 2^32)
    u = np.cumsum(np.cumsum(s.reshape(int(H), int(W)), axis=0, dtype=np.uint32), axis=1, dtype=np.uint32)
    return u.astype("<u4").tobytes()


# --------------------------------------------------------------------------- python bundle
def bundle_files():
    """(archive path, bytes) of everything the simulator needs inside Pyodide (not the terrain grid,
    which the worker takes from the same URL as the renderer)."""
    out = []
    for p in sorted((ROOT / "sim").rglob("*.py")):
        if "__pycache__" not in p.parts:
            out.append((p.relative_to(ROOT).as_posix(), p.read_bytes()))
    for rel in ["server.py", "shared/world.json", "shared/orbiter_geometry.json",
                "sim/data/trim_tables.npz", "web/data/terrain_meta.json"]:
        out.append((rel, (ROOT / rel).read_bytes()))
    return out


def make_bundle() -> bytes:
    """Deterministic zip (fixed timestamps) of bundle_files()."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for rel, data in bundle_files():
            zi = zipfile.ZipInfo(rel, date_time=(2020, 1, 1, 0, 0, 0))
            zi.external_attr = 0o644 << 16
            zi.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(zi, data)
    return buf.getvalue()


def ensure_generated():
    """World data and trim tables up to date (the bundle ships the tables the sources expect)."""
    from worldgen.build import ensure_assets
    ensure_assets()
    from sim.aero import AeroModel
    from sim.geometry import build_geometry
    from sim.trim import load_or_build_trim_tables
    load_or_build_trim_tables(AeroModel(build_geometry()), verbose=True)


# --------------------------------------------------------------------------- pyodide
def _download(url, dst: Path):
    tmp = dst.with_suffix(dst.suffix + ".part")
    log(f"download {url}")
    with urllib.request.urlopen(url, timeout=120) as r, open(tmp, "wb") as f:
        shutil.copyfileobj(r, f)
    tmp.replace(dst)


def pyodide_files():
    """Core runtime + wheels (with dependencies) of PY_PACKAGES, cached in .cache/pyodide/."""
    CACHE.mkdir(parents=True, exist_ok=True)
    for f in PYODIDE_CORE:
        if not (CACHE / f).exists():
            _download(PYODIDE_CDN + f, CACHE / f)
    lock = json.loads((CACHE / "pyodide-lock.json").read_text())
    pk = lock["packages"]
    need, todo = set(), list(PY_PACKAGES)
    while todo:
        n = todo.pop()
        if n not in need:
            need.add(n)
            todo += pk[n].get("depends", [])
    files = list(PYODIDE_CORE)
    for n in sorted(need):
        fn, want = pk[n]["file_name"], pk[n]["sha256"]
        p = CACHE / fn
        if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest() != want:
            _download(PYODIDE_CDN + fn, p)
            if hashlib.sha256(p.read_bytes()).hexdigest() != want:
                raise SystemExit(f"checksum mismatch for {fn}")
        files.append(fn)
    return files


# --------------------------------------------------------------------------- output
def write(path: Path, data: bytes):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_bytes() == data:
        return
    path.write_bytes(data)
    gz = path.with_name(path.name + ".gz")
    if path.suffix in GZ_EXT and len(data) > 1024:
        z = gzip.compress(data, compresslevel=9, mtime=0)
        if len(z) < 0.9 * len(data):
            gz.write_bytes(z)
            return
    if gz.exists():
        gz.unlink()


def build(out: Path):
    ensure_generated()
    out.mkdir(parents=True, exist_ok=True)

    # 1. big data, content-addressed
    files, sizes, enc = {}, {}, {}
    for name in BIG_DATA:
        data = (WEB / "data" / name).read_bytes()
        sizes[name] = len(data)
        pub = name
        if name == "terrain_height.f32":
            meta = json.loads((WEB / "data" / "terrain_meta.json").read_text())
            data, pub, enc[name] = encode_heights(data, int(meta["size"])), "terrain_height.plh", "plh1"
        rel = f"d/{sha(data)}/{pub}"
        write(out / rel, data)
        files[name] = rel

    # 2. pyodide
    pyo_rel = f"pyodide/{PYODIDE_VERSION}/"
    for fn in pyodide_files():
        write(out / pyo_rel / fn, (CACHE / fn).read_bytes())

    # 3. the release: hash everything that goes into v/<ver>/ (plus the data / pyodide it points to)
    rel_files = {}
    for sub in ("js", "css", "vendor"):
        for p in sorted((WEB / sub).rglob("*")):
            if p.is_file() and not p.name.startswith("."):
                rel_files[p.relative_to(WEB).as_posix()] = p.read_bytes()
    rel_files["orbiter.html"] = (WEB / "orbiter.html").read_bytes()
    for name in SMALL_DATA:
        rel_files[f"data/{name}"] = (WEB / "data" / name).read_bytes()
    rel_files["py/sim.zip"] = make_bundle()
    h = hashlib.sha256()
    for k in sorted(rel_files):
        h.update(k.encode() + b"\0" + hashlib.sha256(rel_files[k]).digest())
    h.update(json.dumps([files, pyo_rel, LAYOUT], sort_keys=True).encode())
    h.update((WEB / "index.html").read_bytes())
    ver = h.hexdigest()[:10]
    vdir = f"v/{ver}/"
    for k, data in rel_files.items():
        write(out / vdir / k, data)

    cfg = {
        "ver": ver, "sim": "browser",
        "data": vdir + "data/", "files": files, "sizes": sizes, "enc": enc,
        "bundle": vdir + "py/sim.zip", "pyodide": pyo_rel,
        "versionURL": "version.json",
    }
    # what this release references (deploy_web.sh prunes d/ against the releases it keeps)
    write(out / vdir / "release.json", json.dumps({**cfg, "refs": sorted(files.values())}, indent=1).encode())

    # 4. index.html: point at the release folder, inject the config, preload the module graph
    html = (WEB / "index.html").read_text(encoding="utf-8")
    html = html.replace('href="./css/', f'href="./{vdir}css/')
    html = html.replace('"./vendor/', f'"./{vdir}vendor/')
    html = html.replace('src="./js/main.js"', f'src="./{vdir}js/main.js"')
    if f"{vdir}js/main.js" not in html or f"{vdir}vendor/" not in html:
        raise SystemExit("index.html: could not rewrite the asset paths")
    preload = "".join(f'<link rel="modulepreload" href="./{vdir}{k}" />\n' for k in sorted(rel_files)
                      if k.startswith("js/") and k.endswith(".js") and not k.startswith(("js/sim/", "js/tools/")))
    preload += f'<link rel="modulepreload" href="./{vdir}vendor/three/build/three.module.js" />\n'
    # config before the import map; module preloads only after it (a module fetched before the import
    # map is seen disables the map: "three" would no longer resolve)
    inject = f"<script>window.PLANELANDS = {json.dumps(cfg, separators=(',', ':'))};</script>\n"
    html, n1 = re.subn(r'(<script type="importmap">)', lambda m: inject + m.group(1), html, count=1)
    html, n2 = re.subn(r'(<script type="importmap">.*?</script>\n)', lambda m: m.group(1) + preload, html,
                       count=1, flags=re.S)
    if n1 != 1 or n2 != 1:
        raise SystemExit("index.html: import map not found")
    write(out / "index.html", html.encode())
    write(out / "version.json", json.dumps({"ver": ver}).encode())
    total = sum(len(v) for v in rel_files.values())
    log(f"release {ver}: {len(rel_files)} files ({total / 1e6:.1f} MB) -> {out}")
    return ver


def serve(out: Path, port: int):
    """Local preview with roughly the production cache headers (and gzip_static)."""
    from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

    class H(SimpleHTTPRequestHandler):
        extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".js": "text/javascript", ".mjs": "text/javascript",
                          ".wasm": "application/wasm", ".json": "application/json", ".f32": "application/octet-stream",
                          ".u8": "application/octet-stream", ".whl": "application/zip"}

        def end_headers(self):
            p = self.path.split("?")[0]
            immutable = p.startswith(("/v/", "/d/", "/pyodide/"))
            self.send_header("Cache-Control", "public, max-age=31536000, immutable" if immutable else "no-cache")
            super().end_headers()

        def log_message(self, *a):
            pass

    os.chdir(out)
    log(f"serving {out} on http://localhost:{port}")
    ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(ROOT / "dist"))
    ap.add_argument("--serve", type=int, default=0, help="serve the result on this port afterwards")
    args = ap.parse_args()
    out = Path(args.out).resolve()
    build(out)
    if args.serve:
        serve(out, args.serve)


if __name__ == "__main__":
    main()
