#!/usr/bin/env bash
# set -euo pipefail

# cd "$HOME/dimos"
# export PATH="$HOME/.local/bin:$PATH"
# export LD_PRELOAD="/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
# export DIMOS_POINTLIO_HOST_IP="${DIMOS_POINTLIO_HOST_IP:-192.168.123.164}"
# export DIMOS_POINTLIO_LIDAR_IP="${DIMOS_POINTLIO_LIDAR_IP:-192.168.123.161}"

# exec uv run dimos run unitree-g1-teleop --transport=lcm --rerun-host 0.0.0.0 --rerun-open none --network-interface eth0 "$@"

# #!/usr/bin/env bash
# set -euo pipefail

# cd "$HOME/dimos"
# export PATH="$HOME/.local/bin:$PATH"
# export LD_PRELOAD="/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
# export DIMOS_POINTLIO_HOST_IP="${DIMOS_POINTLIO_HOST_IP:-192.168.123.164}"
# export DIMOS_POINTLIO_LIDAR_IP="${DIMOS_POINTLIO_LIDAR_IP:-192.168.123.161}"

# exec uv run dimos run unitree-g1-teleop --network-interface eth0 "$@"

#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"
unset PYTHONPATH PYTHONHOME
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$HOME/cyclonedds/install}"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"

ROBOPLAN_GOMP="$(find "$HOME/dimos/.venv/lib/python3.12/site-packages/roboplan.libs" -maxdepth 1 -name 'libgomp-*.so*' -print -quit)"
if [[ -z "${ROBOPLAN_GOMP}" ]]; then
  echo "roboplan libgomp not found" >&2
  exit 1
fi
export LD_PRELOAD="${ROBOPLAN_GOMP}:/lib/aarch64-linux-gnu/libGLdispatch.so.0:/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
export LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.123.164}"
export LIDAR_IP="${LIDAR_IP:-192.168.123.120}"

send_g1_stop() {
  uv run --no-sync python -c 'from unitree_sdk2py.core.channel import ChannelFactoryInitialize; from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient; ChannelFactoryInitialize(0, "eth0"); c=LocoClient(); c.SetTimeout(2.0); c.Init(); [c.StopMove() for _ in range(3)]'
}

case "${1:-}" in
  status)
    exec uv run --no-sync dimos --transport=lcm status
    ;;
  logs|log)
    shift
    exec uv run --no-sync dimos --transport=lcm log --follow "$@"
    ;;
  stop)
    shift
    # Stop physical motion first; coordinator cleanup can take several seconds.
    send_g1_stop || true
    # Request an orderly shutdown first so G1HighLevelDdsSdk sends StopMove.
    uv run --no-sync dimos --transport=lcm stop "$@" || true
    # A second, independent stop command protects against a coordinator that
    # had to be force-killed before module cleanup completed.
    send_g1_stop || true
    exit 0
    ;;
  help|-h|--help)
    cat <<'EOF'
Usage: ./run_g1_dimos_nav.sh [start options]
       ./run_g1_dimos_nav.sh status
       ./run_g1_dimos_nav.sh logs [-n LINES]
       ./run_g1_dimos_nav.sh stop [--force]

Run without --daemon to see live logs and stop with Ctrl-C.
Run with --daemon, then use "logs" and "stop" above.
EOF
    exit 0
    ;;
esac

# Starting a second coordinator on the same LCM bus is unsafe: deployment gets
# far enough to create modules before the coordinator collision is detected,
# and its cleanup can stop native processes belonging to the first run.
status_output="$(uv run --no-sync dimos --transport=lcm status 2>&1 || true)"
if [[ "${status_output}" != *"No running DimOS instance"* ]]; then
  echo "DimOS is already running; leaving the existing instance unchanged."
  printf '%s\n' "${status_output}"
  exit 0
fi

# exec uv run --no-sync dimos run unitree-g1-teleop --network-interface eth0 "$@"
exec uv run --no-sync dimos --transport=lcm --rerun-host 0.0.0.0 --rerun-open none \
  run unitree-g1-voice-agentic --network-interface eth0 "$@"

  # --transport=lcm --rerun-host 0.0.0.0 --rerun-open none
