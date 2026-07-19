"""Minimal Linux joystick reader (/dev/input/js*)."""

from __future__ import annotations

import array
import fcntl
import select
import struct
from pathlib import Path
from typing import List

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
_JS_EVENT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# linux/joystick.h
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12


class LinuxJoystick:
    def __init__(self, device: str = "/dev/input/js0") -> None:
        path = Path(device)
        if not path.exists():
            raise FileNotFoundError(f"Joystick device not found: {device}")
        self.device = str(path)
        self._f = open(self.device, "rb")  # noqa: SIM115
        n_axes = self._ioctl_u8(JSIOCGAXES)
        n_buttons = self._ioctl_u8(JSIOCGBUTTONS)
        self.axes: List[float] = [0.0] * n_axes
        self.buttons: List[int] = [0] * n_buttons
        self._drain_init()

    def _ioctl_u8(self, op: int) -> int:
        buf = array.array("B", [0])
        fcntl.ioctl(self._f.fileno(), op, buf)
        return int(buf[0])

    def _ensure_size(self, kind: str, index: int) -> None:
        if kind == "axis":
            while len(self.axes) <= index:
                self.axes.append(0.0)
        else:
            while len(self.buttons) <= index:
                self.buttons.append(0)

    def _apply(self, value: int, typ: int, number: int) -> None:
        typ = typ & ~JS_EVENT_INIT
        if typ == JS_EVENT_AXIS:
            self._ensure_size("axis", number)
            self.axes[number] = max(-1.0, min(1.0, float(value) / 32767.0))
        elif typ == JS_EVENT_BUTTON:
            self._ensure_size("button", number)
            self.buttons[number] = 1 if value else 0

    def _drain_init(self) -> None:
        """Consume buffered INIT events so state matches the kernel."""
        for _ in range(256):
            r, _, _ = select.select([self._f], [], [], 0.05)
            if not r:
                break
            data = self._f.read(_JS_EVENT.size)
            if len(data) != _JS_EVENT.size:
                break
            _t, value, typ, number = _JS_EVENT.unpack(data)
            self._apply(value, typ, number)

    def poll(self) -> None:
        """Non-blocking: apply all pending events."""
        while True:
            r, _, _ = select.select([self._f], [], [], 0.0)
            if not r:
                break
            data = self._f.read(_JS_EVENT.size)
            if len(data) != _JS_EVENT.size:
                break
            _t, value, typ, number = _JS_EVENT.unpack(data)
            self._apply(value, typ, number)

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:  # noqa: BLE001
            pass
