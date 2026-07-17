"""UDP JSON IPC between ROS / radio_rx and the PWM daemon."""

from __future__ import annotations

import json
import os
import socket
from dataclasses import asdict, dataclass
from typing import Optional

from protocol import SOURCE_RADIO, SOURCE_ROS, TeleopCommand

DEFAULT_HOST = os.environ.get("PWM_DAEMON_HOST", "127.0.0.1")
DEFAULT_PORT = int(os.environ.get("PWM_DAEMON_PORT", "5600"))


@dataclass
class IpcCommand:
    source: str
    left: float
    right: float
    arm: bool = True
    seq: int = 0

    def to_json(self) -> bytes:
        return json.dumps(asdict(self), separators=(",", ":")).encode("utf-8")

    @classmethod
    def from_json(cls, raw: bytes) -> Optional["IpcCommand"]:
        try:
            data = json.loads(raw.decode("utf-8"))
            source = str(data.get("source", SOURCE_ROS))
            if source not in (SOURCE_ROS, SOURCE_RADIO):
                return None
            return cls(
                source=source,
                left=float(data["left"]),
                right=float(data["right"]),
                arm=bool(data.get("arm", True)),
                seq=int(data.get("seq", 0)),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError):
            return None

    def to_teleop(self) -> TeleopCommand:
        return TeleopCommand(
            left=self.left,
            right=self.right,
            arm=self.arm,
            seq=self.seq,
            source=self.source,
        )


class PwmDaemonClient:
    """Fire-and-forget UDP client used by ROS and radio_rx."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, source: str, left: float, right: float, arm: bool = True, seq: int = 0) -> None:
        cmd = IpcCommand(source=source, left=left, right=right, arm=arm, seq=seq)
        self._sock.sendto(cmd.to_json(), self.addr)

    def close(self) -> None:
        self._sock.close()
