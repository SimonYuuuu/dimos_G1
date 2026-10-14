#!/usr/bin/env bash
# Inspire 右手手指简单测试（Modbus TCP，无需额外依赖，只需 python3）。
# 用法:
#   ./run_g1_inspire_right_hand_test.sh                 # 在本机直接运行（需能访问手的 IP）
#   ./run_g1_inspire_right_hand_test.sh --remote        # 通过 SSH 在 Orin 上运行
#   ./run_g1_inspire_right_hand_test.sh --open          # 只张开手
# 环境变量:
#   INSPIRE_HAND_IP   默认 192.168.123.211（已在 Orin 上探测到 6000 端口开放）
#   INSPIRE_HAND_PORT 默认 6000
#   INSPIRE_HAND_ID   默认 1
#   G1_SSH_HOST       默认 unitree@192.168.123.164（--remote 时使用）
#   REPEAT 重复次数；HAND_CLOSE 弯曲幅度；HAND_FIST 收拢位置；HAND_SPEED/HAND_FORCE 速度/力度（改脚本顶部默认值即可）
#   HAND_SPEED        速度 0-1000，默认 200；HAND_FORCE 力阈值 0-1000，默认 200
#   HAND_CLOSE        单指弯曲位置 0-1000，默认 850
set -euo pipefail

# ===== 在这里改参数 =====
REPEAT="${REPEAT:-2}"        # 整套动作（手指运动 + yeah + 点赞）重复次数
export HAND_CLOSE="${HAND_CLOSE:-850}"   # 手指弯曲幅度 0-1000（越大越小幅）
export HAND_SPEED="${HAND_SPEED:-200}"
export HAND_FORCE="${HAND_FORCE:-200}"
export HAND_FIST="${HAND_FIST:-300}"     # yeah/点赞时收拢手指的位置（0=握紧，1000=张开）
# ========================

MODE="wiggle"
REMOTE=0
for a in "$@"; do
  case "$a" in
    --remote) REMOTE=1 ;;
    --open) MODE="open" ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "未知参数: $a" >&2; exit 2 ;;
  esac
done

PY=$(cat <<'PYEOF'
import os, socket, struct, sys, time

IP    = os.environ.get("INSPIRE_HAND_IP", "192.168.123.211")
PORT  = int(os.environ.get("INSPIRE_HAND_PORT", "6000"))
UID   = int(os.environ.get("INSPIRE_HAND_ID", "1"))
FIST  = max(0, min(1000, int(os.environ.get("HAND_FIST", "300"))))
REPEAT = max(1, int(os.environ.get("HAND_REPEAT", "2")))
CLOSE = max(0, min(1000, int(os.environ.get("HAND_CLOSE", "850"))))
SPEED = max(0, min(1000, int(os.environ.get("HAND_SPEED", "200"))))
FORCE = max(0, min(1000, int(os.environ.get("HAND_FORCE", "200"))))
MODE  = os.environ.get("HAND_MODE", "wiggle")

REG_ANGLE_SET = 1486   # 6 个寄存器: 小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
OPEN = [1000] * 6
THUMB_IN_BEND = int(os.environ.get("THUMB_IN_BEND", "400"))  # yeah 时拇指收拢程度，方向不对就改这两个值
THUMB_IN_ROT  = int(os.environ.get("THUMB_IN_ROT", "300"))

s = socket.create_connection((IP, PORT), timeout=3)
tid = 0
def write_regs(addr, vals):
    global tid
    tid = (tid + 1) & 0xFFFF
    body = struct.pack(">BBHHB", UID, 0x10, addr, len(vals), len(vals) * 2)
    body += b"".join(struct.pack(">H", v) for v in vals)
    s.sendall(struct.pack(">HHH", tid, 0, len(body)) + body)
    hdr = recv_exact(6)  # MBAP: tid, proto, length
    rtid, _, length = struct.unpack(">HHH", hdr)
    r = hdr + recv_exact(length)
    if rtid != tid or len(r) < 8 or r[7] & 0x80:
        raise RuntimeError(f"Modbus 写入失败 addr={addr}: {r.hex()}")

def recv_exact(n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("灵巧手断开了连接")
        buf += chunk
    return buf

def pose(v, wait=0.8):
    write_regs(REG_ANGLE_SET, v)
    time.sleep(wait)

try:
    print(f"连接 {IP}:{PORT} (id={UID})")
    write_regs(REG_SPEED_SET, [SPEED] * 6)
    write_regs(REG_FORCE_SET, [FORCE] * 6)
    pose(OPEN, 1.0)
    if MODE == "wiggle":
        for n in range(REPEAT):
            print(f"第 {n+1}/{REPEAT} 轮")
            # 1) 四指依次弯曲再张开
            for i in range(4):
                p = OPEN[:]; p[i] = CLOSE
                pose(p, 0.6)
                pose(OPEN, 0.6)
            # 2) 比 yeah：食指、中指伸直，无名指、小指收起，拇指收向掌心
            #    顺序: 小指, 无名指, 中指, 食指, 拇指弯曲, 拇指旋转
            yeah = [FIST, FIST, 1000, 1000, THUMB_IN_BEND, THUMB_IN_ROT]
            pose(yeah, 1.2); pose(OPEN, 0.8)
            # 3) 点赞：四指收起，拇指伸直，上下点两下（靠拇指弯曲值）
            like = [FIST, FIST, FIST, FIST, 1000, 1000]
            pose(like, 1.2)
            for _ in range(2):
                pose([FIST]*4 + [700, 1000], 0.5)
                pose(like, 0.5)
            pose(OPEN, 1.0)
    print("完成，手已张开。")
except KeyboardInterrupt:
    pass
finally:
    try: write_regs(REG_ANGLE_SET, OPEN)
    except Exception: pass
    s.close()
PYEOF
)

export HAND_MODE="$MODE" HAND_REPEAT="$REPEAT"
if [[ "$REMOTE" == 1 ]]; then
  REMOTE_HOST="${G1_SSH_HOST:-unitree@192.168.123.164}"
  ENVS=("HAND_MODE=$MODE" "HAND_REPEAT=$REPEAT" "HAND_CLOSE=$HAND_CLOSE" "HAND_SPEED=$HAND_SPEED" "HAND_FORCE=$HAND_FORCE" "HAND_FIST=$HAND_FIST")
  for v in INSPIRE_HAND_IP INSPIRE_HAND_PORT INSPIRE_HAND_ID; do
    if [[ -n "${!v:-}" ]]; then ENVS+=("$v=${!v}"); fi
  done
  printf '%s\n' "$PY" | ssh "$REMOTE_HOST" "env $(printf '%q ' "${ENVS[@]}")python3 -"
else
  printf '%s\n' "$PY" | python3 -
fi
