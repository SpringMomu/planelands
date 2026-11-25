#!/usr/bin/env sh
# Build the browser release and publish it to the web server (default ssh host "jp").
#   tools/deploy_web.sh [ssh-host] [--keep N] [--no-build] [--dry-run]
# Linux / macOS / Git Bash launcher for tools/deploy_web.py (needs Python 3 and ssh, no rsync).
dir=$(dirname "$0")
for py in python3 python py; do
  # skip the Windows Store "python3" stub, which exists but does not run
  if command -v "$py" >/dev/null 2>&1 && "$py" -c 'import sys; sys.exit(sys.version_info < (3, 8))' >/dev/null 2>&1; then
    exec "$py" "$dir/deploy_web.py" "$@"
  fi
done
echo "deploy_web: Python 3.8+ not found" >&2; exit 1
