"""Load PWM device profiles from teleop/devices/*.json."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

DEVICES_DIR = Path(__file__).resolve().parent / "devices"


@dataclass(frozen=True)
class DeviceConfig:
    name: str
    backend: str
    left_pin: int
    right_pin: int
    radio_timeout_s: float = 0.25
    ros_timeout_s: float = 0.5
    daemon_host: str = "127.0.0.1"
    daemon_port: int = 5600
    thrust_scale: float = 0.7

    @classmethod
    def from_dict(cls, data: dict) -> "DeviceConfig":
        return cls(
            name=str(data["name"]),
            backend=str(data["backend"]),
            left_pin=int(data["left_pin"]),
            right_pin=int(data["right_pin"]),
            radio_timeout_s=float(data.get("radio_timeout_s", 0.25)),
            ros_timeout_s=float(data.get("ros_timeout_s", 0.5)),
            daemon_host=str(data.get("daemon_host", "127.0.0.1")),
            daemon_port=int(data.get("daemon_port", 5600)),
            thrust_scale=float(data.get("thrust_scale", 0.7)),
        )


def resolve_device_path(device: str) -> Path:
    """Accept a profile name ('rpi', 'jetson') or an absolute/relative JSON path."""
    path = Path(device)
    if path.suffix == ".json" and path.is_file():
        return path
    candidate = DEVICES_DIR / f"{device}.json"
    if candidate.is_file():
        return candidate
    # Also allow bare filename inside devices/
    candidate = DEVICES_DIR / device
    if candidate.is_file():
        return candidate
    known = sorted(p.stem for p in DEVICES_DIR.glob("*.json"))
    raise FileNotFoundError(
        f"Device config '{device}' not found. Known profiles: {', '.join(known) or '(none)'}"
    )


def load_device_config(device: str | None = None) -> DeviceConfig:
    """Load config from DEVICE_CONFIG env or an explicit name/path (default: rpi)."""
    name = device or os.environ.get("DEVICE_CONFIG", "rpi")
    path = resolve_device_path(name)
    with path.open() as f:
        data = json.load(f)
    cfg = DeviceConfig.from_dict(data)
    return cfg
