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

"""Hardware-free tests for the Inspire Modbus TCP client (uses socketpair)."""

from __future__ import annotations

import socket
import struct

import pytest

from dimos.robot.unitree.g1.effectors.inspire_hand import modbus
from dimos.robot.unitree.g1.effectors.inspire_hand.modbus import (
    REG_ANGLE_ACT,
    REG_ANGLE_SET,
    InspireModbusClient,
    ModbusDeviceError,
    ModbusProtocolError,
    pack_request,
    parse_mbap,
)


def _client() -> tuple[InspireModbusClient, socket.socket]:
    a, b = socket.socketpair()
    a.settimeout(1.0)
    b.settimeout(1.0)
    c = InspireModbusClient("127.0.0.1", 1, unit_id=1, timeout=1.0)
    c._sock = a
    return c, b


def _reply(tx: int, unit: int, pdu: bytes) -> bytes:
    return struct.pack(">HHHB", tx, 0, len(pdu) + 1, unit) + pdu


def _read_req(peer: socket.socket) -> tuple[int, bytes]:
    hdr = peer.recv(7)
    tx, _, length, _ = struct.unpack(">HHHB", hdr)
    return tx, peer.recv(length - 1)


def test_pack_request_layout() -> None:
    pdu = struct.pack(">BHH", 0x03, REG_ANGLE_ACT, 6)
    frame = pack_request(0x1234, 1, pdu)
    assert frame[:7] == bytes([0x12, 0x34, 0, 0, 0, 6, 1])
    assert frame[7:] == pdu
    assert parse_mbap(frame[:7]) == (0x1234, 0, 6, 1)


def test_parse_mbap_rejects_bad_length() -> None:
    with pytest.raises(ModbusProtocolError):
        parse_mbap(struct.pack(">HHHB", 1, 0, 0, 1))
    with pytest.raises(ModbusProtocolError):
        parse_mbap(b"\x00\x01")


def test_read_holding_roundtrip() -> None:
    c, peer = _client()
    values = [1000, 900, 800, 700, 600, 0xFFFF]
    # Pre-queue the reply; drain() runs before send, so reply must come after.
    import threading

    def serve() -> None:
        tx, pdu = _read_req(peer)
        fc, addr, count = struct.unpack(">BHH", pdu)
        assert (fc, addr, count) == (0x03, REG_ANGLE_ACT, 6)
        data = struct.pack(">6H", *values)
        peer.sendall(_reply(tx, 1, bytes([0x03, len(data)]) + data))

    t = threading.Thread(target=serve)
    t.start()
    assert c.read_holding(REG_ANGLE_ACT, 6) == values
    t.join()


def test_write_regs_frame_and_clamp() -> None:
    import threading

    c, peer = _client()
    seen: dict[str, bytes] = {}

    def serve() -> None:
        tx, pdu = _read_req(peer)
        seen["pdu"] = pdu
        peer.sendall(_reply(tx, 1, struct.pack(">BHH", 0x10, REG_ANGLE_SET, 6)))

    t = threading.Thread(target=serve)
    t.start()
    c.set_angles([-5, 0, 500, 1000, 2000, 1])
    t.join()
    fc, addr, count, nbytes = struct.unpack(">BHHB", seen["pdu"][:6])
    assert (fc, addr, count, nbytes) == (0x10, REG_ANGLE_SET, 6, 12)
    assert list(struct.unpack(">6H", seen["pdu"][6:])) == [0, 0, 500, 1000, 1000, 1]


def _serve_once(peer: socket.socket, make_reply) -> object:  # type: ignore[no-untyped-def]
    import threading

    def serve() -> None:
        tx, _ = _read_req(peer)
        peer.sendall(make_reply(tx))

    t = threading.Thread(target=serve)
    t.start()
    return t


def test_transaction_id_mismatch_raises_and_drops_socket() -> None:
    c, peer = _client()
    t = _serve_once(peer, lambda tx: _reply((tx + 1) & 0xFFFF, 1, bytes([0x03, 12]) + b"\0" * 12))
    with pytest.raises(ModbusProtocolError, match="transaction id"):
        c.read_holding(REG_ANGLE_ACT, 6)
    t.join()  # type: ignore[attr-defined]
    assert not c.connected


def test_unit_and_function_code_mismatch() -> None:
    c, peer = _client()
    t = _serve_once(peer, lambda tx: _reply(tx, 2, bytes([0x03, 12]) + b"\0" * 12))
    with pytest.raises(ModbusProtocolError, match="unit id"):
        c.read_holding(REG_ANGLE_ACT, 6)
    t.join()  # type: ignore[attr-defined]

    c, peer = _client()
    t = _serve_once(peer, lambda tx: _reply(tx, 1, bytes([0x04, 12]) + b"\0" * 12))
    with pytest.raises(ModbusProtocolError, match="function code"):
        c.read_holding(REG_ANGLE_ACT, 6)
    t.join()  # type: ignore[attr-defined]


def test_modbus_exception_response_keeps_connection() -> None:
    c, peer = _client()
    t = _serve_once(peer, lambda tx: _reply(tx, 1, bytes([0x83, 0x02])))
    with pytest.raises(ModbusDeviceError, match="0x02"):
        c.read_holding(REG_ANGLE_ACT, 6)
    t.join()  # type: ignore[attr-defined]
    assert c.connected


def test_drain_detects_peer_close() -> None:
    c, peer = _client()
    peer.close()
    with pytest.raises(ConnectionError):
        c.read_holding(REG_ANGLE_ACT, 6)
    assert not c.connected


def test_not_connected_raises() -> None:
    c = InspireModbusClient("127.0.0.1", 1)
    with pytest.raises(ConnectionError):
        c.get_angles()


def test_env_ip_and_port(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("INSPIRE_HAND_IP", "10.0.0.9")
    monkeypatch.setenv("INSPIRE_HAND_PORT", "6100")
    c = InspireModbusClient()
    assert (c.ip, c.port) == ("10.0.0.9", 6100)
    monkeypatch.delenv("INSPIRE_HAND_IP")
    monkeypatch.delenv("INSPIRE_HAND_PORT")
    assert modbus.env_ip() == modbus.FALLBACK_IP
    assert modbus.env_port() == modbus.FALLBACK_PORT
