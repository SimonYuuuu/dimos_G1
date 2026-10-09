#!/usr/bin/env bash
set -euo pipefail

# exec ssh -N \
#   -o ExitOnForwardFailure=yes \
#   -L 13030:127.0.0.1:3030 \
#   -L 19877:127.0.0.1:9877 \
#   g1-dimos

VIEWER_BIN="${DIMOS_VIEWER_BIN:-$HOME/.local/bin/dimos-viewer}"

if [[ ! -x "${VIEWER_BIN}" ]]; then
  echo "dimos-viewer not found at ${VIEWER_BIN}" >&2
  exit 1
fi

exec "${VIEWER_BIN}" \
  --newest-first \
  --connect rerun+http://127.0.0.1:19877/proxy \
  --ws-url ws://127.0.0.1:13030/ws
