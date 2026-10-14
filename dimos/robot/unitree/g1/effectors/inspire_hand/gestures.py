#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ruff: noqa: RUF001, RUF002
"""通过 DimOS 的 InspireHandModule（RPC）让右手重复：手指逐个弯曲、比 yeah、点赞。

运行：python -m dimos.robot.unitree.g1.effectors.inspire_hand.gestures
前提：DimOS 已用 ./run_g1_inspire_hand.sh 启动（或由 run_g1_inspire_hand_gestures.sh 自动启动）。
归一化数值 0..1，1=张开；关节顺序：小指、无名指、中指、食指、拇指弯曲、拇指旋转。
"""

import os
import sys
import time

from dimos.porcelain.dimos import Dimos

REPEAT = int(os.environ.get("REPEAT", "2"))  # 重复次数
CURL = float(os.environ.get("HAND_CLOSE", "0.85"))  # 单指弯曲幅度（越大越小幅）
FIST = float(os.environ.get("HAND_FIST", "0.3"))  # yeah/点赞时收拢手指的位置
THUMB_IN = (
    float(os.environ.get("THUMB_IN_BEND", "0.4")),
    float(os.environ.get("THUMB_IN_ROT", "0.3")),
)  # yeah 时拇指收拢

OPEN = [1.0] * 6


def main() -> int:
    # Dimos.connect() 只连接已运行的实例；app.stop() 只关闭本脚本的 RPC 连接，不会停掉 DimOS 进程。
    # 实例的启动/停止由 run_g1_inspire_hand_gestures.sh 负责（只停它自己启动的）。
    app = Dimos.connect(timeout=10.0)
    try:
        hand = app.get_module("InspireHandModule")
        print("当前状态:", hand.get_state())

        def pose(p, wait=0.8):
            hand.set_normalized(list(p))  # RPC，全量程（不受 agent skill 安全下限限制）
            time.sleep(wait)

        try:
            pose(OPEN, 1.0)
            for n in range(REPEAT):
                print(f"第 {n + 1}/{REPEAT} 轮")
                for i in (3, 2, 1, 0):  # 食指、中指、无名指、小指
                    p = OPEN[:]
                    p[i] = CURL
                    pose(p, 0.6)
                    pose(OPEN, 0.6)
                # yeah：食指、中指伸直，无名指、小指收起，拇指收向掌心
                pose([FIST, FIST, 1.0, 1.0, THUMB_IN[0], THUMB_IN[1]], 1.2)
                pose(OPEN, 0.8)
                # 点赞：四指收起，拇指伸直，点两下
                like = [FIST, FIST, FIST, FIST, 1.0, 1.0]
                pose(like, 1.2)
                for _ in range(2):
                    pose([FIST] * 4 + [0.7, 1.0], 0.5)
                    pose(like, 0.5)
                pose(OPEN, 1.0)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                hand.open_hand()
            except Exception as e:
                print(f"警告：张开手失败: {e}", file=sys.stderr)
    finally:
        app.stop()
    print("完成，手已张开。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
