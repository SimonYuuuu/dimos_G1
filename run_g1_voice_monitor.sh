#!/usr/bin/env bash
# Read-only G1 voice transcript monitor. It can run on either the host or Orin.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${SCRIPT_DIR}/voice-monitor.pid"
LOG_FILE="${SCRIPT_DIR}/voice-monitor.log"
G1_ADDRESS="${G1_SSH_TARGET:-unitree@192.168.123.164}"

on_orin() {
  ip -o -4 address show 2>/dev/null | grep -q '192\.168\.123\.164/'
}

stream_local_logs() {
  cd "${HOME}/dimos"
  local current_file=""
  local follower_pid=""

  cleanup_follower() {
    if [[ -n "${follower_pid}" ]]; then
      kill "${follower_pid}" 2>/dev/null || true
      wait "${follower_pid}" 2>/dev/null || true
    fi
  }
  trap cleanup_follower EXIT INT TERM

  while true; do
    local latest_file=""
    latest_file="$(ls -t logs/*unitree-g1-voice-control*/main.jsonl 2>/dev/null | head -1 || true)"
    if [[ -n "${latest_file}" && "${latest_file}" != "${current_file}" ]]; then
      cleanup_follower
      current_file="${latest_file}"
      echo "Following ${current_file}" >&2
      tail -n 100 -F "${current_file}" |
        python3 -u "${SCRIPT_DIR}/voice_log_formatter.py" &
      follower_pid=$!
    fi
    sleep 2
  done
}

run_remote_monitor() {
  ssh "${G1_ADDRESS}" 'cd ~/dimos && ./run_g1_voice_monitor.sh __stream'
}

run_monitor() {
  if on_orin; then
    stream_local_logs
  else
    while true; do
      run_remote_monitor || true
      echo "Voice log stream unavailable; retrying in 3 seconds..." >&2
      sleep 3
    done
  fi
}

running_pid() {
  [[ -f "${PID_FILE}" ]] || return 1
  local pid
  pid="$(<"${PID_FILE}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  printf '%s\n' "${pid}"
}

case "${1:-}" in
  --daemon)
    if pid="$(running_pid)"; then
      echo "Voice monitor is already running (PID ${pid})."
      echo "Log: ${LOG_FILE}"
      exit 0
    fi
    : >"${LOG_FILE}"
    nohup setsid "$0" __run </dev/null >>"${LOG_FILE}" 2>&1 &
    pid=$!
    printf '%s\n' "${pid}" >"${PID_FILE}"
    sleep 0.3
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "Voice monitor failed to start. See ${LOG_FILE}" >&2
      exit 1
    fi
    echo "Voice monitor started in background (PID ${pid})."
    echo "View: $0 logs"
    echo "Stop: $0 stop"
    ;;
  __run)
    run_monitor
    ;;
  __stream)
    stream_local_logs
    ;;
  status)
    if pid="$(running_pid)"; then
      echo "Voice monitor is running (PID ${pid})."
      echo "Log: ${LOG_FILE}"
    else
      echo "Voice monitor is not running."
      exit 1
    fi
    ;;
  logs|log)
    touch "${LOG_FILE}"
    exec tail -n 100 -f "${LOG_FILE}"
    ;;
  stop)
    if ! pid="$(running_pid)"; then
      rm -f "${PID_FILE}"
      echo "Voice monitor is not running."
      exit 0
    fi
    # The daemon has its own process group, so this also closes ssh/grep.
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    rm -f "${PID_FILE}"
    echo "Voice monitor stopped."
    ;;
  help|-h|--help)
    cat <<EOF
Usage: $0 [--daemon|status|logs|stop]

With no argument, follow voice events in the current terminal.
The script runs locally on the Orin and connects to ${G1_ADDRESS} from the host.
Set G1_SSH_TARGET to override the SSH destination.
EOF
    ;;
  '')
    run_monitor
    ;;
  *)
    echo "Unknown option: $1" >&2
    echo "Run '$0 --help' for usage." >&2
    exit 2
    ;;
esac
