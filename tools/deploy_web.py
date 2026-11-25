"""Build the browser release and publish it to the web server.  Linux, macOS and Windows.

    python tools/deploy_web.py [ssh-host] [--keep N] [--remote DIR] [--no-build] [--dry-run]
    tools/deploy_web.sh  [ssh-host] ...        (Linux / macOS / Git Bash)
    tools\\deploy_web.cmd [ssh-host] ...        (Windows cmd / PowerShell)

Needs only Python 3 and an ``ssh`` client (OpenSSH; built into Windows 10+) that can log in to the
host, e.g. an entry ``Host jp`` in ~/.ssh/config.  The server needs ``tar`` and ``python3``.

Steps, as the release layout requires (tools/build_web.py):
  1. upload the content-addressed files the server does not have yet (``v/<ver>/``, ``d/<sha>/``,
     ``pyodide/<version>/``) plus every other changed file except index.html / version.json, as one
     tar stream over ssh;
  2. switch: index.html, then version.json, each written to a temporary name and renamed into place,
     so visitors never see a half-uploaded release; open pages notice version.json and offer a reload;
  3. prune all but the newest N releases (default 5) and the data / Pyodide directories no kept
     release references.
"""
from __future__ import annotations

import argparse
import functools
import os
import posixpath
import shlex
import shutil
import subprocess
import sys
import tarfile
import time
from pathlib import Path

print = functools.partial(print, flush=True)   # keep our lines in order with ssh output
ROOT = Path(__file__).resolve().parent.parent
SWITCH = ("index.html", "version.json")          # uploaded last, in this order

PRUNE = r'''
import json, os, shutil, sys
root, cur, keep = sys.argv[1], sys.argv[2], int(sys.argv[3])
vdir = os.path.join(root, "v")
vers = sorted(os.listdir(vdir), key=lambda v: os.path.getmtime(os.path.join(vdir, v)), reverse=True)
kept = [cur] + [v for v in vers if v != cur][:keep - 1]
for v in vers:
    if v not in kept:
        shutil.rmtree(os.path.join(vdir, v)); print("pruned release", v)
refs, pyo = set(), set()
for v in kept:
    try:
        r = json.load(open(os.path.join(vdir, v, "release.json")))
    except OSError:
        continue
    refs |= {p.split("/")[1] for p in r.get("refs", [])}
    pyo.add(r.get("pyodide", "").strip("/").split("/")[-1])
for sub, keepset, what in (("d", refs, "data"), ("pyodide", pyo, "pyodide")):
    d = os.path.join(root, sub)
    for h in (os.listdir(d) if os.path.isdir(d) else []):
        if h not in keepset:
            shutil.rmtree(os.path.join(d, h)); print("pruned", what, h)
print(f"live release {cur}; kept {len(kept)}")
'''


def ssh(host, cmd, **kw):
    """Run a remote shell command; the command is a POSIX shell string executed on the server."""
    return subprocess.run(["ssh", host, cmd], **kw)


def remote_listing(host, remote):
    q = shlex.quote(remote)
    r = ssh(host, f"mkdir -p {q} && cd {q} && find . -type f -printf '%P\\t%s\\n'",
            capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"ssh {host}: cannot list {remote}: {r.stderr.strip()}")
    out = {}
    for ln in r.stdout.splitlines():
        p, _, s = ln.rpartition("\t")
        if p:
            out[p] = int(s)
    return out


def local_listing(dist):
    out = {}
    for p in dist.rglob("*"):
        if p.is_file():
            out[p.relative_to(dist).as_posix()] = p.stat().st_size
    return out


def upload_tar(host, remote, dist, paths):
    """Stream the files as an uncompressed tar into `tar -x` on the server (most are compressed)."""
    q = shlex.quote(remote)
    proc = subprocess.Popen(["ssh", host, f"mkdir -p {q} && tar -xf - -C {q}"], stdin=subprocess.PIPE)
    with tarfile.open(fileobj=proc.stdin, mode="w|", format=tarfile.PAX_FORMAT) as tf:
        for rel in paths:
            ti = tf.gettarinfo(str(dist / rel), arcname=rel)
            ti.uid = ti.gid = 0; ti.uname = ti.gname = ""
            ti.mode = 0o644
            with open(dist / rel, "rb") as f:
                tf.addfile(ti, f)
    proc.stdin.close()
    if proc.wait() != 0:
        sys.exit("upload failed")


def put_atomic(host, remote, dist, rel):
    dst = posixpath.join(remote, rel)
    tmp = posixpath.join(posixpath.dirname(dst), f".{posixpath.basename(dst)}.upload")
    with open(dist / rel, "rb") as f:
        r = ssh(host, f"cat > {shlex.quote(tmp)} && chmod 644 {shlex.quote(tmp)} && mv -f {shlex.quote(tmp)} {shlex.quote(dst)}", stdin=f)
    if r.returncode != 0:
        sys.exit(f"switching {rel} failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("host", nargs="?", default="jp", help="ssh host (default: jp)")
    ap.add_argument("--keep", type=int, default=5, help="releases to keep on the server (default 5)")
    ap.add_argument("--remote", default="/var/www/planelands", help="web root on the server")
    ap.add_argument("--dist", default=str(ROOT / "dist"), help="build output directory")
    ap.add_argument("--no-build", action="store_true", help="publish the existing dist/ as it is")
    ap.add_argument("--dry-run", action="store_true", help="show what would be uploaded, change nothing")
    a = ap.parse_args()
    if a.keep < 1:
        ap.error("--keep must be at least 1")
    if not shutil.which("ssh"):
        sys.exit("no ssh client found (Windows: Settings > Optional features > OpenSSH Client)")
    dist = Path(a.dist).resolve()
    t0 = time.time()

    if not a.no_build:
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "build_web.py"), "--out", str(dist)])
        if r.returncode != 0:
            sys.exit("build failed")
    import json
    ver = json.loads((dist / "version.json").read_text(encoding="utf-8"))["ver"]

    loc, rem = local_listing(dist), remote_listing(a.host, a.remote)
    # content-addressed paths never change once published: upload what is missing; anything else
    # (e.g. index.html.gz) whenever its size differs or it is not a hashed path
    hashed = ("v/", "d/", "pyodide/")
    todo = sorted(p for p, s in loc.items() if p not in SWITCH and
                  (p not in rem or rem[p] != s or not p.startswith(hashed)))
    size = sum(loc[p] for p in todo)
    print(f"[deploy] release {ver} -> {a.host}:{a.remote}: {len(todo)} files to upload ({size / 1e6:.1f} MB)")
    if a.dry_run:
        for p in todo:
            print("  ", p)
        print("[deploy] dry run: nothing changed")
        return
    if todo:
        upload_tar(a.host, a.remote, dist, todo)
    for rel in SWITCH:
        put_atomic(a.host, a.remote, dist, rel)
    print(f"[deploy] switched to {ver}")
    r = ssh(a.host, f"python3 - {shlex.quote(a.remote)} {shlex.quote(ver)} {a.keep}", input=PRUNE.encode())      # bytes: no CRLF translation on Windows
    if r.returncode != 0:
        print("[deploy] warning: pruning failed (the new release is live)", file=sys.stderr)
    print(f"[deploy] done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
