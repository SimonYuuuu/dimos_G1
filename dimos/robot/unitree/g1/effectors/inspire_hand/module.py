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

"""DimOS Module for the Inspire RH56DFTP hand (Modbus TCP).

Streams:
    hand_command (In[JointState]):  positions normalized 0..1 (1 = open/extended;
        thumb_rotation 1 = across palm). Names must match JOINT_NAMES.
    hand_state (Out[JointState]):   same names; position = normalized angles,
        effort = normalized |force| (force_act / 1000).

Joint order: pinky, ring, middle, index, thumb_bend, thumb_rotation.
"""

from __future__ import annotations

import threading
from threading import Thread
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.effectors.inspire_hand.modbus import (
    DEFAULT_IP,
    DEFAULT_PORT,
    NUM_DOF,
    InspireModbusClient,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_FINGERS = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")


def joint_names(side: str = "r") -> list[str]:
    prefix = "left_hand" if side == "l" else "right_hand"
    return [f"{prefix}/{f}" for f in _FINGERS]


JOINT_NAMES: list[str] = joint_names("r")


def _to_raw(norm: float) -> int:
    return max(0, min(1000, int(round(float(norm) * 1000))))


class InspireHandModuleConfig(ModuleConfig):
    ip: str = DEFAULT_IP
    port: int = DEFAULT_PORT
    device_id: int = 1
    side: str = "r"
    control_rate_hz: float = 20.0
    default_speed: int = 200
    default_force: int = 200
    open_on_stop: bool = True
    frame_id: str = "inspire_hand"


class InspireHandModule(Module):
    """Bridges DimOS JointState commands to an Inspire RH56DFTP hand."""

    config: InspireHandModuleConfig

    hand_command: In[JointState]
    hand_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: InspireModbusClient | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Thread | None = None
        self._names = joint_names(self.config.side)

    # -- lifecycle --------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        client = InspireModbusClient(cfg.ip, cfg.port, cfg.device_id)
        client.connect()
        client.set_speed([cfg.default_speed] * NUM_DOF)
        client.set_force([cfg.default_force] * NUM_DOF)
        self._client = client
        logger.info("InspireHandModule connected", ip=cfg.ip, port=cfg.port)

        self.register_disposable(Disposable(self.hand_command.subscribe(self._on_command)))

        self._stop_event.clear()
        self._thread = Thread(target=self._publish_loop, daemon=True, name="inspire-hand-state")
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._thread = None
        if self._client is not None:
            if self.config.open_on_stop:
                try:
                    self._client.set_angles([1000] * NUM_DOF)
                except Exception as e:
                    logger.warning(f"Failed to open hand on stop: {e}")
            self._client.close()
            self._client = None
        super().stop()

    # -- stream handlers --------------------------------------------------
    def _on_command(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        if names:
            if len(names) != len(positions):
                logger.warning("hand_command name/position length mismatch; ignoring")
                return
            by_name = dict(zip(names, positions, strict=True))
            if not all(n in by_name for n in self._names):
                logger.warning(f"hand_command must contain {self._names}; ignoring")
                return
            positions = [by_name[n] for n in self._names]
        if len(positions) != NUM_DOF:
            logger.warning(f"hand_command expected {NUM_DOF} positions, got {len(positions)}")
            return
        try:
            self.set_normalized(positions)
        except Exception as e:
            logger.warning(f"hand_command failed: {e}")

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.control_rate_hz)
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            try:
                angles, forces = self._read()
                self.hand_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=self.config.frame_id,
                        name=self._names,
                        position=[a / 1000.0 for a in angles],
                        effort=[abs(f) / 1000.0 for f in forces],
                    )
                )
            except Exception as e:
                logger.warning(f"Inspire hand read failed: {e}")
                self._stop_event.wait(0.5)
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                self._stop_event.wait(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _read(self) -> tuple[list[int], list[int]]:
        with self._lock:
            if self._client is None:
                raise ConnectionError("not connected")
            return self._client.get_angles(), self._client.get_forces()

    # -- RPC / skills -----------------------------------------------------
    @rpc
    def set_raw(self, angles: list[int]) -> None:
        """Set 6 target angles, ints 0..1000 (1000 = open)."""
        if len(angles) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} angles")
        with self._lock:
            if self._client is None:
                raise ConnectionError("not connected")
            self._client.set_angles([max(0, min(1000, int(a))) for a in angles])

    @rpc
    @skill
    def set_normalized(self, positions: list[float]) -> str:
        """Set 6 hand joints normalized 0..1 (1=open): pinky, ring, middle, index, thumb_bend, thumb_rotation."""
        if len(positions) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} positions")
        self.set_raw([_to_raw(p) for p in positions])
        return "ok"

    @rpc
    @skill
    def open_hand(self) -> str:
        """Fully open the hand."""
        self.set_raw([1000] * NUM_DOF)
        return "Hand opened"

    @rpc
    @skill
    def close_hand(self, level: float = 0.3) -> str:
        """Close the hand to a normalized level (0=fully closed, 1=open); thumb rotates across palm."""
        r = _to_raw(level)
        self.set_raw([r, r, r, r, r, 1000 - r])
        return f"Hand closed to level {level}"

    @rpc
    def get_state(self) -> dict[str, Any]:
        angles, forces = self._read()
        return {
            "names": list(self._names),
            "angles_raw": angles,
            "angles_norm": [a / 1000.0 for a in angles],
            "forces_raw": forces,
        }
