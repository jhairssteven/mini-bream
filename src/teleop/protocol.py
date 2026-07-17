"""Shared framing for ground-station ↔ radio ↔ robot emergency teleop.

Serial packet (9 bytes, little-endian):
  magic(2)=0xAA55 | left_i16 | right_i16 | flags_u8 | seq_u8 | crc8

Thrust encoding: int16 = round(thrust * 1000), thrust in [-1.0, 1.0]
flags bit0 = arm (deadman held)

UDP IPC to the PWM daemon uses the same fields as a JSON object
(see ipc.py) so ROS and radio_rx share one command shape.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"\xaa\x55"
PACKET_FMT = "<2shhBB"  # magic, left, right, flags, seq  — CRC appended separately
PACKET_SIZE = 9  # 8 + crc
FLAG_ARM = 0x01

SOURCE_ROS = "ros"
SOURCE_RADIO = "radio"


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def thrust_to_i16(thrust: float) -> int:
    return int(max(-1000, min(1000, round(float(thrust) * 1000.0))))


def i16_to_thrust(value: int) -> float:
    return max(-1.0, min(1.0, float(value) / 1000.0))


@dataclass
class TeleopCommand:
    left: float
    right: float
    arm: bool
    seq: int = 0
    source: str = SOURCE_RADIO

    def encode_serial(self) -> bytes:
        flags = FLAG_ARM if self.arm else 0
        body = struct.pack(
            PACKET_FMT,
            MAGIC,
            thrust_to_i16(self.left),
            thrust_to_i16(self.right),
            flags,
            self.seq & 0xFF,
        )
        return body + bytes([crc8(body)])

    @classmethod
    def decode_serial(cls, packet: bytes, source: str = SOURCE_RADIO) -> TeleopCommand | None:
        if len(packet) != PACKET_SIZE:
            return None
        body, check = packet[:-1], packet[-1]
        if crc8(body) != check:
            return None
        magic, left_i, right_i, flags, seq = struct.unpack(PACKET_FMT, body)
        if magic != MAGIC:
            return None
        return cls(
            left=i16_to_thrust(left_i),
            right=i16_to_thrust(right_i),
            arm=bool(flags & FLAG_ARM),
            seq=seq,
            source=source,
        )


class SerialFrameParser:
    """Byte-stream reassembler for the fixed-size serial frame."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[TeleopCommand]:
        self._buf.extend(data)
        out: list[TeleopCommand] = []
        while True:
            idx = self._buf.find(MAGIC)
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < PACKET_SIZE:
                break
            packet = bytes(self._buf[:PACKET_SIZE])
            cmd = TeleopCommand.decode_serial(packet)
            if cmd is not None:
                del self._buf[:PACKET_SIZE]
                out.append(cmd)
            else:
                # False sync — skip one byte and search again
                del self._buf[:1]
        return out
