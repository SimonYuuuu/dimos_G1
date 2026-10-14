#!/usr/bin/env bash
# 在 Orin 上运行：如果 DimOS 手模块没在跑，就先启动，再重复比划手势，结束后停止（仅停止本脚本启动的）。
# 重复次数等参数在下面改，或用环境变量覆盖：  REPEAT=3 ./run_g1_inspire_hand_gestures.sh
set -euo pipefail
cd "$HOME/dimos"
export PATH="$HOME/.local/bin:$PATH"
unset PYTHONPATH PYTHONHOME
export DIMOS_TRANSPORT=lcm

# ===== 在这里改参数 =====
export REPEAT="${REPEAT:-2}"             # 整套动作重复次数
export HAND_CLOSE="${HAND_CLOSE:-0.85}"  # 单指弯曲幅度，越接近 1 幅度越小
export HAND_FIST="${HAND_FIST:-0.3}"     # yeah/点赞时收拢手指位置
# ========================

STARTED=0
if [[ "$(uv run --no-sync dimos --transport=lcm status 2>&1 || true)" == *"No running DimOS instance"* ]]; then
  nohup ./run_g1_inspire_hand.sh start > /tmp/inspire_dimos.log 2>&1 &
  STARTED=1
  for _ in $(seq 1 30); do
    sleep 1
    grep -q "InspireHandModule connected" /tmp/inspire_dimos.log 2>/dev/null && break
  done
  sleep 2
fi

cleanup() {
  if [[ "$STARTED" == 1 ]]; then
    ./run_g1_inspire_hand.sh stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

uv run --no-sync python inspire_hand_gestures.py
