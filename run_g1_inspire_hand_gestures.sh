#!/usr/bin/env bash
# 在 Orin 上运行：如果 unitree-g1-inspire-hand 没在跑，就先启动，再重复比划手势；
# 结束后只停止本脚本自己启动的实例。若有别的 DimOS 实例（其它 blueprint）在跑，则报错退出。
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
START_TIMEOUT="${START_TIMEOUT:-30}"     # 等待手模块连上的秒数
BLUEPRINT="unitree-g1-inspire-hand"
LOG=/tmp/inspire_dimos.log
# ========================

dimos_status() { uv run --no-sync dimos --transport=lcm status 2>&1 || true; }

STARTED_PID=""
cleanup() {
  if [[ -n "$STARTED_PID" ]]; then
    # 只停本脚本启动的那个实例（按 PID 核对，避免误停别人的）
    if dimos_status | grep -Eq "PID:[[:space:]]+${STARTED_PID}\b"; then
      uv run --no-sync dimos --transport=lcm stop >/dev/null 2>&1 || true
    elif kill -0 "$STARTED_PID" 2>/dev/null; then
      kill -INT "$STARTED_PID" 2>/dev/null || true
    fi
  fi
}
trap cleanup EXIT

status="$(dimos_status)"
if [[ "$status" == *"No running DimOS instance"* ]]; then
  : > "$LOG"
  nohup uv run --no-sync dimos --transport=lcm run "$BLUEPRINT" > "$LOG" 2>&1 &
  LAUNCH_PID=$!
  STARTED_PID=$LAUNCH_PID
  ok=0
  for _ in $(seq 1 "$START_TIMEOUT"); do
    sleep 1
    if grep -q "InspireHandModule connected" "$LOG" 2>/dev/null; then ok=1; break; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then break; fi
  done
  # 记录实际的 DimOS PID（可能与 nohup 启动的 PID 不同）
  pid_now="$(dimos_status | sed -nE 's/^[[:space:]]*PID:[[:space:]]+([0-9]+).*/\1/p' | head -n1)"
  [[ -n "$pid_now" ]] && STARTED_PID="$pid_now"
  if [[ "$ok" != 1 ]]; then
    echo "错误：${START_TIMEOUT}s 内手模块未连上（见 $LOG）：" >&2
    tail -n 20 "$LOG" >&2 || true
    exit 1
  fi
  sleep 2
elif [[ "$status" == *"Blueprint: ${BLUEPRINT}"* ]]; then
  echo "复用已在运行的 ${BLUEPRINT} 实例（结束后不会停止它）。"
else
  echo "错误：已有其它 DimOS 实例在运行（非 ${BLUEPRINT}），不会启动或停止它：" >&2
  printf '%s\n' "$status" >&2
  exit 1
fi

uv run --no-sync python -m dimos.robot.unitree.g1.effectors.inspire_hand.gestures
