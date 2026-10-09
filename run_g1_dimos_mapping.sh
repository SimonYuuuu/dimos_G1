#!/usr/bin/env bash
# Passive G1 map collection: Point-LIO + recording + Rerun, no cmd_vel output.
set -euo pipefail

cd "$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"
unset PYTHONPATH PYTHONHOME
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"

# Open3D / RoboPlan can load separate libgomp copies on Jetson.  Preload the
# bundled copy before the system copy so Open3D does not fail with
# "cannot allocate memory in static TLS block" during coordinator startup.
ROBOPLAN_GOMP="$(find "$HOME/dimos/.venv/lib/python3.12/site-packages/roboplan.libs" -maxdepth 1 -name 'libgomp-*.so*' -print -quit)"
if [[ -z "$ROBOPLAN_GOMP" ]]; then
  echo "roboplan libgomp not found" >&2
  exit 1
fi
export LD_PRELOAD="$ROBOPLAN_GOMP:/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"

# These must describe the Orin NIC and the physical MID-360, respectively.
export DIMOS_POINTLIO_HOST_IP="${DIMOS_POINTLIO_HOST_IP:-192.168.123.164}"
export DIMOS_POINTLIO_LIDAR_IP="${DIMOS_POINTLIO_LIDAR_IP:-192.168.123.120}"

exec uv run --no-sync dimos \
  --transport=lcm --rerun-host=0.0.0.0 --rerun-open=none \
  run unitree-g1-map-record "$@"


# Map creation dimos map global memory.db --export