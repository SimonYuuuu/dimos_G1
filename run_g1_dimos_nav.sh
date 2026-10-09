#!/usr/bin/env bash
# Stable G1 nav-simple launcher.  The legacy teleop-named script is retained
# for compatibility with existing shortcuts.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_g1_dimos_teleop.sh" "$@"
