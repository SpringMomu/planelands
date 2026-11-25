#!/usr/bin/env bash
# Build the browser release and publish it to the web server (default: ssh host "jp").
#   tools/deploy_web.sh [ssh-host] [--keep N]
# Uploads only what changed (releases and data are content-addressed), switches index.html last so
# visitors never see a half-uploaded release, then prunes all but the newest N releases (default 5)
# and the data files no kept release references. Open pages notice the new version.json and offer
# a reload.
set -euo pipefail
HOST="${1:-jp}"; KEEP=5
[[ "${2:-}" == "--keep" ]] && KEEP="${3:?}"
REMOTE=/var/www/planelands
cd "$(dirname "$0")/.."

python3 tools/build_web.py --out dist
VER=$(python3 -c "import json; print(json.load(open('dist/version.json'))['ver'])")

ssh "$HOST" "mkdir -p $REMOTE"
# 1. immutable content first (new paths only; index.html.gz may run a moment ahead of index.html,
#    harmless: both name releases that exist)
rsync -a --info=stats1 --exclude=/index.html --exclude=/version.json dist/ "$HOST:$REMOTE/"
# 2. switch: index.html, then version.json (rsync renames into place atomically)
rsync -a dist/index.html "$HOST:$REMOTE/index.html"
rsync -a dist/version.json "$HOST:$REMOTE/version.json"

# 3. prune old releases and unreferenced data
ssh "$HOST" python3 - "$REMOTE" "$VER" "$KEEP" <<'PY'
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
for h in os.listdir(os.path.join(root, "d")):
    if h not in refs:
        shutil.rmtree(os.path.join(root, "d", h)); print("pruned data", h)
for pv in os.listdir(os.path.join(root, "pyodide")):
    if pv not in pyo:
        shutil.rmtree(os.path.join(root, "pyodide", pv)); print("pruned pyodide", pv)
print(f"live release {cur}; kept {len(kept)}")
PY
