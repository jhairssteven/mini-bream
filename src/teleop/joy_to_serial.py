#!/usr/bin/env python3
"""Ground station: Linux joystick → left/right thrust → telemetry serial.

Reads /dev/input/js* directly.

Default Xbox mapping (from js_dump):
  axis[1]  → left thrust
  axis[4]  → right thrust
  button[5] → deadman (must hold to arm thrusters)
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from js_reader import LinuxJoystick
from protocol import TeleopCommand

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("teleop.joy_to_serial")


def apply_deadband(value: float, deadband: float) -> float:
    """Zero values inside ±deadband; optionally rescale outside (snap to zero only)."""
    if abs(value) <= deadband:
        return 0.0
    return value


def map_thrust(
    axes: list[float],
    buttons: list[int],
    left_axis: int,
    right_axis: int,
    deadman_button: int,
    deadband: float = 0.09,
) -> tuple[float, float, bool]:
    if deadman_button >= len(buttons) or not buttons[deadman_button]:
        return 0.0, 0.0, False

    left = axes[left_axis] if left_axis < len(axes) else 0.0
    right = axes[right_axis] if right_axis < len(axes) else 0.0
    # Linux js axes: stick up is typically negative; invert so up = forward thrust (+)
    left = apply_deadband(max(-1.0, min(1.0, -left)), deadband)
    right = apply_deadband(max(-1.0, min(1.0, -right)), deadband)
    return left, right, True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joy-device", default=os.environ.get("JOY_DEVICE", "/dev/input/js0"))
    parser.add_argument("--port", default=os.environ.get("RADIO_SERIAL_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--baud", type=int, default=int(os.environ.get("RADIO_BAUD", "57600")))
    parser.add_argument("--left-axis", type=int, default=int(os.environ.get("JOY_LEFT_AXIS", "1")))
    parser.add_argument("--right-axis", type=int, default=int(os.environ.get("JOY_RIGHT_AXIS", "4")))
    parser.add_argument(
        "--deadman-button",
        type=int,
        default=int(os.environ.get("JOY_DEADMAN_BUTTON", "5")),
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=float(os.environ.get("JOY_DEADBAND", "0.09")),
        help="Zero thrust when |axis| <= this (default 0.09)",
    )
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args()

    import serial

    joy = LinuxJoystick(args.joy_device)
    ser = serial.Serial(args.port, args.baud, timeout=0)
    logger.info(
        "Joystick %s (%d axes, %d buttons) → serial %s @ %d  "
        "[left=axis[%d] right=axis[%d] deadman=button[%d] deadband=±%.2f]",
        args.joy_device,
        len(joy.axes),
        len(joy.buttons),
        args.port,
        args.baud,
        args.left_axis,
        args.right_axis,
        args.deadman_button,
        args.deadband,
    )

    running = True

    def _stop(*_a: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    seq = 0
    period = 1.0 / args.rate
    last_arm = False
    try:
        while running:
            t0 = time.monotonic()
            joy.poll()
            left, right, arm = map_thrust(
                joy.axes,
                joy.buttons,
                args.left_axis,
                args.right_axis,
                args.deadman_button,
                deadband=args.deadband,
            )
            if arm != last_arm:
                logger.info("Deadman %s", "ARMED" if arm else "disarmed")
                last_arm = arm
            cmd = TeleopCommand(left=left, right=right, arm=arm, seq=seq)
            seq = (seq + 1) & 0xFF
            try:
                ser.write(cmd.encode_serial())
            except Exception as exc:  # noqa: BLE001
                logger.error("Serial write failed: %s", exc)
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))
    finally:
        try:
            ser.write(TeleopCommand(0.0, 0.0, arm=False, seq=seq).encode_serial())
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        joy.close()
        logger.info("joy_to_serial stopped")


if __name__ == "__main__":
    main()
