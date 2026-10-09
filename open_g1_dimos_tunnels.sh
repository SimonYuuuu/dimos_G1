#!/usr/bin/env bash
set -euo pipefail

exec ssh -N \
  -o ExitOnForwardFailure=yes \
  -L 13030:127.0.0.1:3030 \
  -L 19877:127.0.0.1:9877 \
  g1-dimos
