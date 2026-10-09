#!/usr/bin/env bash
# Passive G1 premap relocalization.  This process never sends cmd_vel.
set -euo pipefail

cd "$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"
unset PYTHONPATH PYTHONHOME
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"

ROBOPLAN_GOMP="$(find "$HOME/dimos/.venv/lib/python3.12/site-packages/roboplan.libs" -maxdepth 1 -name 'libgomp-*.so*' -print -quit)"
if [[ -z "$ROBOPLAN_GOMP" ]]; then
  echo "roboplan libgomp not found" >&2
  exit 1
fi
export LD_PRELOAD="$ROBOPLAN_GOMP:/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"

export DIMOS_POINTLIO_HOST_IP="${DIMOS_POINTLIO_HOST_IP:-192.168.123.164}"
export DIMOS_POINTLIO_LIDAR_IP="${DIMOS_POINTLIO_LIDAR_IP:-192.168.123.161}"

# Pass --map-file=<stem-or-path>, e.g. --map-file=g1_map.
exec uv run --no-sync dimos \
  --transport=lcm --rerun-host=0.0.0.0 --rerun-open=none \
  run unitree-g1-relocalization "$@"
