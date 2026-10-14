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

import os
import socket
import struct
import threading

FALLBACK_IP = "192.168.123.211"
FALLBACK_PORT = 6000
DEFAULT_UNIT_ID = 1


def env_ip() -> str:
    """Hand IP from ``INSPIRE_HAND_IP`` (falls back to 192.168.123.211)."""
    return os.environ.get("INSPIRE_HAND_IP") or FALLBACK_IP


def env_port() -> int:
    """Hand port from ``INSPIRE_HAND_PORT`` (falls back to 6000)."""
    return int(os.environ.get("INSPIRE_HAND_PORT") or FALLBACK_PORT)


# Resolved at import time for backwards compatibility; prefer env_ip()/env_port().
DEFAULT_IP = env_ip()
DEFAULT_PORT = env_port()

NUM_DOF = 6
DOF_NAMES = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")

REG_ANGLE_SET = 1486
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582

_FC_READ_HOLDING = 0x03
_FC_WRITE_MULTIPLE = 0x10


class ModbusProtocolError(IOError):
    """Malformed / mismatched Modbus TCP response."""


class ModbusDeviceError(IOError):
    """Device returned a Modbus exception (function code | 0x80)."""


def pack_request(tx: int, unit_id: int, pdu: bytes) -> bytes:
    """Build an MBAP-framed Modbus TCP request."""
    return struct.pack(">HHHB", tx & 0xFFFF, 0, len(pdu) + 1, unit_id) + pdu


def parse_mbap(header: bytes) -> tuple[int, int, int, int]:
    """Parse a 7-byte MBAP header -> (tx_id, protocol_id, length, unit_id)."""
    if len(header) != 7:
        raise ModbusProtocolError(f"MBAP header must be 7 bytes, got {len(header)}")
    tx, proto, length, unit = struct.unpack(">HHHB", header)
    if length < 1 or length > 254:
        raise ModbusProtocolError(f"invalid MBAP length {length}")
    return tx, proto, length, unit


def check_response(
    tx: int, proto: int, unit: int, body: bytes, want_tx: int, want_unit: int, want_fc: int
) -> None:
    if tx != want_tx:
        raise ModbusProtocolError(f"transaction id mismatch: got {tx}, expected {want_tx}")
    if proto != 0:
        raise ModbusProtocolError(f"bad protocol id {proto}")
    if unit != want_unit:
        raise ModbusProtocolError(f"unit id mismatch: got {unit}, expected {want_unit}")
    if not body:
        raise ModbusProtocolError("empty response PDU")
    if body[0] == (want_fc | 0x80):
        code = body[1] if len(body) > 1 else 0
        raise ModbusDeviceError(f"Modbus exception 0x{code:02x}")
    if body[0] != want_fc:
        raise ModbusProtocolError(
            f"function code mismatch: got 0x{body[0]:02x}, expected 0x{want_fc:02x}"
        )


def _clamp(v: int) -> int:
    return max(0, min(1000, int(v)))


class InspireModbusClient:
    """Minimal Modbus TCP client. Thread-safe (one request in flight at a time)."""

    def __init__(
        self,
        ip: str | None = None,
        port: int | None = None,
        unit_id: int = DEFAULT_UNIT_ID,
        timeout: float = 1.0,
    ) -> None:
        self.ip = ip or env_ip()
        self.port = int(port) if port else env_port()
        self.unit_id = unit_id
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._tx = 0
        self._lock = threading.Lock()

    # -- connection -------------------------------------------------------
    def connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self.timeout)
        except OSError:
            sock.close()
            raise
        with self._lock:
            self._sock = sock

    def _invalidate(self) -> None:
        """Drop the socket after an I/O error (caller holds ``self._lock``)."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

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
        """Discard any stale bytes (unread write ACKs) before a new request.

        Raises ConnectionError if the peer has closed the connection (recv -> b"").
        """
        assert self._sock is not None
        self._sock.setblocking(False)
        try:
            while True:
                try:
                    if not self._sock.recv(4096):
                        raise ConnectionError("Inspire hand closed the connection")
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
        """Send one PDU and return the response PDU (caller holds ``self._lock``).

        Validates transaction id, protocol id, unit id and function code. Any
        socket error / mismatch invalidates the connection so callers reconnect.
        """
        if self._sock is None:
            raise ConnectionError("Inspire hand not connected")
        self._tx = (self._tx + 1) & 0xFFFF
        frame = pack_request(self._tx, self.unit_id, pdu)
        try:
            self._drain()
            self._sock.sendall(frame)
            header = self._recv_exact(7)
            tx, proto, length, unit = parse_mbap(header)
            body = self._recv_exact(max(length - 1, 0))
            check_response(tx, proto, unit, body, self._tx, self.unit_id, pdu[0])
        except ModbusDeviceError:
            raise  # well-formed reply; stream is still in sync
        except (OSError, ConnectionError, ModbusProtocolError):
            # Desynced or dead stream: drop it; next call must reconnect.
            self._invalidate()
            raise
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
            raise OSError(f"Short Modbus read response ({len(body)} bytes)")
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
