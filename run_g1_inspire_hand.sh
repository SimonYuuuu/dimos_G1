#!/usr/bin/env bash
# Run on the Orin (~/dimos). Inspire RH56DFTP right hand via DimOS (Modbus TCP).
#   ./run_g1_inspire_hand.sh [start]   start DimOS blueprint unitree-g1-inspire-hand
#   ./run_g1_inspire_hand.sh status | logs [-n N] | stop
#   ./run_g1_inspire_hand.sh --demo    short wiggle via Modbus client (no DimOS stack)
set -euo pipefail
cd "$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"
unset PYTHONPATH PYTHONHOME
export INSPIRE_HAND_IP="${INSPIRE_HAND_IP:-192.168.123.211}"

case "${1:-}" in
  status) exec uv run --no-sync dimos --transport=lcm status ;;
  logs|log) shift; exec uv run --no-sync dimos --transport=lcm log --follow "$@" ;;
  stop) shift; exec uv run --no-sync dimos --transport=lcm stop "$@" ;;
  --demo)
    exec uv run --no-sync python - <<'PY'
import os, time, importlib.util, sys
p = os.path.expanduser("~/dimos/dimos/robot/unitree/g1/effectors/inspire_hand/modbus.py")
spec = importlib.util.spec_from_file_location("inspire_modbus", p)
m = importlib.util.module_from_spec(spec); sys.modules["inspire_modbus"] = m; spec.loader.exec_module(m)
c = m.InspireModbusClient(os.environ["INSPIRE_HAND_IP"]); c.connect()
c.set_speed([200]*6); c.set_force([200]*6)
print("angles:", c.get_angles(), "forces:", c.get_forces())
try:
    c.set_angles([1000]*6); time.sleep(1.0)
    for i in (3, 2, 1, 0):  # index, middle, ring, pinky
        a = [1000]*6; a[i] = 850; c.set_angles(a); time.sleep(0.6)
    c.set_angles([1000]*6); time.sleep(0.8)
finally:
    c.set_angles([1000]*6); c.close()
print("demo done")
PY
    ;;
  help|-h|--help) sed -n 2,6p "$0"; exit 0 ;;
  start) shift ;;
esac

status_output="$(uv run --no-sync dimos --transport=lcm status 2>&1 || true)"
if [[ "${status_output}" != *"No running DimOS instance"* ]]; then
  echo "DimOS is already running; leaving the existing instance unchanged."
  printf '%s\n' "${status_output}"
  exit 0
fi
exec uv run --no-sync dimos --transport=lcm run unitree-g1-inspire-hand "$@"
