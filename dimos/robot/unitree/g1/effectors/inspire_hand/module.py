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
        thumb_rotation 1 = across palm). Names must match JOINT_NAMES. Full range.
    hand_state (Out[JointState]):   same names; position = normalized angles,
        effort = normalized |force| (force_act / 1000).
    hand_status (Out[str]):         "connected" / "disconnected: <reason>" on link changes.

Joint order: pinky, ring, middle, index, thumb_bend, thumb_rotation.

Safety: agent-facing @skill methods are clamped to ``skill_min_opening`` (no full
clench). Full-range control is only available via RPC (``set_normalized``,
``set_raw``) and the ``hand_command`` stream. Speed/force are capped by
``max_speed`` / ``max_force``.

Connection: IP/port come from config, defaulting to ``INSPIRE_HAND_IP`` /
``INSPIRE_HAND_PORT``. If the link drops the module reconnects with exponential
backoff and publishes ``hand_status``.
"""

from __future__ import annotations

import threading
from threading import Thread
import time
from typing import Any

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.effectors.inspire_hand.modbus import (
    NUM_DOF,
    InspireModbusClient,
    env_ip,
    env_port,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_FINGERS = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")
_SIDES = {"r": "right_hand", "l": "left_hand"}


def _check_side(side: str) -> str:
    if side not in _SIDES:
        raise ValueError(f"side must be 'r' or 'l', got {side!r}")
    return side


def joint_names(side: str = "r") -> list[str]:
    prefix = _SIDES[_check_side(side)]
    return [f"{prefix}/{f}" for f in _FINGERS]


JOINT_NAMES: list[str] = joint_names("r")


def _to_raw(norm: float) -> int:
    return max(0, min(1000, round(float(norm) * 1000)))


class InspireHandModuleConfig(ModuleConfig):
    ip: str = Field(default_factory=env_ip)
    port: int = Field(default_factory=env_port)
    device_id: int = 1
    side: str = "r"
    control_rate_hz: float = 20.0
    default_speed: int = 200
    default_force: int = 200
    # Hard caps applied to any speed/force written to the hand.
    max_speed: int = 500
    max_force: int = 500
    # Agent skills cannot close any finger below this normalized opening
    # (0 = full clench, 1 = open). Thumb rotation is not limited.
    skill_min_opening: float = 0.3
    # Reconnect backoff (seconds).
    reconnect_initial_s: float = 0.5
    reconnect_max_s: float = 5.0
    open_on_stop: bool = True
    frame_id: str = "inspire_hand"

    @field_validator("side")
    @classmethod
    def _validate_side(cls, v: str) -> str:
        return _check_side(v)

    @field_validator("skill_min_opening")
    @classmethod
    def _validate_floor(cls, v: float) -> float:
        if not 0.0 <= v <= 1.0:
            raise ValueError("skill_min_opening must be in [0, 1]")
        return v


class InspireHandModule(Module):
    """Bridges DimOS JointState commands to an Inspire RH56DFTP hand."""

    config: InspireHandModuleConfig

    hand_command: In[JointState]
    hand_state: Out[JointState]
    hand_status: Out[str]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: InspireModbusClient | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Thread | None = None
        self._names = joint_names(self.config.side)
        self._connected = False
        self._backoff = self.config.reconnect_initial_s
        self._next_retry = 0.0

    # -- connection -------------------------------------------------------
    def _connect_locked(self) -> None:
        """(Re)connect and apply speed/force. Caller holds self._lock."""
        cfg = self.config
        client = InspireModbusClient(cfg.ip, cfg.port, cfg.device_id)
        client.connect()
        try:
            client.set_speed([min(cfg.default_speed, cfg.max_speed)] * NUM_DOF)
            client.set_force([min(cfg.default_force, cfg.max_force)] * NUM_DOF)
        except Exception:
            client.close()
            raise
        self._client = client
        self._connected = True
        self._backoff = cfg.reconnect_initial_s
        logger.info("InspireHandModule connected", ip=cfg.ip, port=cfg.port)
        self._publish_status("connected")

    def _mark_disconnected_locked(self, reason: str) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None
        if self._connected:
            self._connected = False
            logger.warning(f"Inspire hand disconnected: {reason}")
            self._publish_status(f"disconnected: {reason}")
        self._next_retry = time.monotonic() + self._backoff
        self._backoff = min(self._backoff * 2, self.config.reconnect_max_s)

    def _ensure_client_locked(self) -> InspireModbusClient:
        if self._client is not None and self._client.connected:
            return self._client
        if self._client is not None:  # socket invalidated by an I/O error
            self._mark_disconnected_locked("socket error")
        if time.monotonic() < self._next_retry:
            raise ConnectionError("Inspire hand not connected (waiting to reconnect)")
        try:
            self._connect_locked()
        except Exception as e:
            self._mark_disconnected_locked(f"reconnect failed: {e}")
            raise ConnectionError(f"Inspire hand reconnect failed: {e}") from e
        assert self._client is not None
        return self._client

    def _call(self, fn: Any) -> Any:
        """Run fn(client) under the lock; on I/O failure drop the link for reconnect."""
        with self._lock:
            client = self._ensure_client_locked()
            try:
                return fn(client)
            except (OSError, ConnectionError) as e:
                self._mark_disconnected_locked(str(e))
                raise

    def _publish_status(self, status: str) -> None:
        try:
            self.hand_status.publish(status)
        except Exception as e:
            logger.debug(f"hand_status publish failed: {e}")

    # -- lifecycle --------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        with self._lock:
            self._connect_locked()  # fail fast on first connect

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
        with self._lock:
            if self._client is not None:
                if self.config.open_on_stop:
                    try:
                        self._client.set_angles([1000] * NUM_DOF)
                    except Exception as e:
                        logger.warning(f"Failed to open hand on stop: {e}")
                self._client.close()
                self._client = None
            self._connected = False
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
                logger.debug(f"Inspire hand read failed: {e}")
                self._stop_event.wait(0.2)
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                self._stop_event.wait(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _read(self) -> tuple[list[int], list[int]]:
        return self._call(lambda c: (c.get_angles(), c.get_forces()))  # type: ignore[no-any-return]

    # -- RPC (full range, not exposed to agents) ---------------------------
    @rpc
    def set_raw(self, angles: list[int]) -> None:
        """Set 6 target angles, ints 0..1000 (1000 = open). Full range."""
        if len(angles) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} angles")
        raw = [max(0, min(1000, int(a))) for a in angles]
        self._call(lambda c: c.set_angles(raw))

    @rpc
    def set_normalized(self, positions: list[float]) -> str:
        """Set 6 joints normalized 0..1 (1=open). Full range; RPC only."""
        if len(positions) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} positions")
        self.set_raw([_to_raw(p) for p in positions])
        return "ok"

    @rpc
    def set_speed_force(self, speed: int, force: int) -> str:
        """Set speed and force (0..1000) for all DoF, capped by max_speed/max_force."""
        sp = max(0, min(int(speed), self.config.max_speed))
        fo = max(0, min(int(force), self.config.max_force))
        self._call(lambda c: (c.set_speed([sp] * NUM_DOF), c.set_force([fo] * NUM_DOF)))
        return f"speed={sp} force={fo}"

    # -- agent skills (safety-limited) ------------------------------------
    def _floor(self, positions: list[float]) -> list[float]:
        lo = self.config.skill_min_opening
        out = [max(lo, min(1.0, float(p))) for p in positions[:5]]
        out.append(max(0.0, min(1.0, float(positions[5]))))  # thumb rotation: not a clench
        return out

    @rpc
    @skill
    def set_hand_pose(self, positions: list[float]) -> str:
        """Set 6 hand joints normalized 0..1 (1=open): pinky, ring, middle, index, thumb_bend, thumb_rotation. Finger closing is limited by a safety floor."""
        if len(positions) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} positions")
        safe = self._floor(positions)
        self.set_raw([_to_raw(p) for p in safe])
        return f"ok (applied {[round(p, 3) for p in safe]})"

    @rpc
    @skill
    def open_hand(self) -> str:
        """Fully open the hand."""
        self.set_raw([1000] * NUM_DOF)
        return "Hand opened"

    @rpc
    @skill
    def close_hand(self, level: float = 0.5) -> str:
        """Partially close the hand to a normalized level (1=open); limited by a safety floor so it never fully clenches."""
        lvl = max(self.config.skill_min_opening, min(1.0, float(level)))
        r = _to_raw(lvl)
        self.set_raw([r, r, r, r, r, 1000 - r])
        return f"Hand closed to level {lvl}"

    @rpc
    def get_state(self) -> dict[str, Any]:
        angles, forces = self._read()
        return {
            "names": list(self._names),
            "connected": self._connected,
            "angles_raw": angles,
            "angles_norm": [a / 1000.0 for a in angles],
            "forces_raw": forces,
        }
