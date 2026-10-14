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

"""Stdlib-only Modbus TCP client for the Inspire RH56DFTP hand.

DOF order (all registers): pinky, ring, middle, index, thumb_bend, thumb_rotation.
Values are ints 0..1000 (1000 = open/extended for fingers and thumb_bend).
"""

from __future__ import annotations

import socket
import struct
import threading

DEFAULT_IP = "192.168.123.211"
DEFAULT_PORT = 6000
DEFAULT_UNIT_ID = 1

NUM_DOF = 6
DOF_NAMES = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")

REG_ANGLE_SET = 1486
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582

_FC_READ_HOLDING = 0x03
_FC_WRITE_MULTIPLE = 0x10


def _clamp(v: int) -> int:
    return max(0, min(1000, int(v)))


class InspireModbusClient:
    """Minimal Modbus TCP client. Thread-safe (one request in flight at a time)."""

    def __init__(
        self,
        ip: str = DEFAULT_IP,
        port: int = DEFAULT_PORT,
        unit_id: int = DEFAULT_UNIT_ID,
        timeout: float = 1.0,
    ) -> None:
        self.ip = ip
        self.port = port
        self.unit_id = unit_id
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._tx = 0
        self._lock = threading.Lock()

    # -- connection -------------------------------------------------------
    def connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.timeout)
        self._sock = sock

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # -- low level --------------------------------------------------------
    def _drain(self) -> None:
        """Discard any stale bytes (unread write ACKs) before a new request."""
        assert self._sock is not None
        self._sock.setblocking(False)
        try:
            while True:
                try:
                    if not self._sock.recv(4096):
                        break
                except (BlockingIOError, InterruptedError):
                    break
        finally:
            self._sock.settimeout(self.timeout)

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Inspire hand closed the connection")
            buf += chunk
        return buf

    def _transact(self, pdu: bytes) -> bytes:
        """Send one PDU, return the response PDU. TX-ID is NOT validated (firmware quirk)."""
        if self._sock is None:
            raise ConnectionError("Inspire hand not connected")
        self._tx = (self._tx + 1) & 0xFFFF
        frame = struct.pack(">HHHB", self._tx, 0, len(pdu) + 1, self.unit_id) + pdu
        self._drain()
        self._sock.sendall(frame)
        header = self._recv_exact(7)
        length = struct.unpack(">H", header[4:6])[0]
        body = self._recv_exact(max(length - 1, 0))
        if body and body[0] & 0x80:
            raise IOError(f"Modbus exception 0x{body[1] if len(body) > 1 else 0:02x}")
        return body

    # -- register access --------------------------------------------------
    def write_regs(self, addr: int, values: list[int]) -> None:
        data = b"".join(struct.pack(">H", _clamp(v)) for v in values)
        pdu = struct.pack(">BHHB", _FC_WRITE_MULTIPLE, addr, len(values), len(data)) + data
        with self._lock:
            self._transact(pdu)  # reads the ACK
            self._drain()  # and drain any extras so they don't desync later reads

    def read_holding(self, addr: int, count: int) -> list[int]:
        pdu = struct.pack(">BHH", _FC_READ_HOLDING, addr, count)
        with self._lock:
            body = self._transact(pdu)
        if len(body) < 2 + 2 * count:
            raise IOError(f"Short Modbus read response ({len(body)} bytes)")
        return list(struct.unpack(f">{count}H", body[2 : 2 + 2 * count]))

    # -- convenience ------------------------------------------------------
    @staticmethod
    def _check6(values: list[int]) -> list[int]:
        if len(values) != NUM_DOF:
            raise ValueError(f"expected {NUM_DOF} values, got {len(values)}")
        return [_clamp(v) for v in values]

    def set_angles(self, angles: list[int]) -> None:
        self.write_regs(REG_ANGLE_SET, self._check6(angles))

    def set_speed(self, speeds: list[int]) -> None:
        self.write_regs(REG_SPEED_SET, self._check6(speeds))

    def set_force(self, forces: list[int]) -> None:
        self.write_regs(REG_FORCE_SET, self._check6(forces))

    def get_angles(self) -> list[int]:
        return self.read_holding(REG_ANGLE_ACT, NUM_DOF)

    def get_forces(self) -> list[int]:
        # FORCE_ACT is signed 16-bit on the hardware.
        return [v - 65536 if v > 32767 else v for v in self.read_holding(REG_FORCE_ACT, NUM_DOF)]
