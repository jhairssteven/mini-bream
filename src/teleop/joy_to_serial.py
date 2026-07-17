#!/usr/bin/env python3
"""Ground station: ROS joy → differential thrust → telemetry serial.

Runs alongside joy_node. Deadman (default button 0) must be held to arm.
Axis mapping matches joystick_control defaults (linear=1, angular=2, scale=3).
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from protocol import TeleopCommand


class JoyToSerial(Node):
    def __init__(
        self,
        serial_port: str,
        baud: int,
        linear_axis: int,
        angular_axis: int,
        scale_axis: int,
        deadman_button: int,
        rate_hz: float,
    ) -> None:
        super().__init__("joy_to_serial")
        self.linear_axis = linear_axis
        self.angular_axis = angular_axis
        self.scale_axis = scale_axis
        self.deadman_button = deadman_button
        self.seq = 0
        self.left = 0.0
        self.right = 0.0
        self.arm = False

        import serial

        self.ser = serial.Serial(serial_port, baud, timeout=0)
        self.get_logger().info("Serial %s @ %d", serial_port, baud)

        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _on_joy(self, msg: Joy) -> None:
        if self.deadman_button >= len(msg.buttons):
            self.arm = False
            self.left = self.right = 0.0
            return

        self.arm = bool(msg.buttons[self.deadman_button])
        if not self.arm:
            self.left = self.right = 0.0
            return

        lin = msg.axes[self.linear_axis] if self.linear_axis < len(msg.axes) else 0.0
        ang = msg.axes[self.angular_axis] if self.angular_axis < len(msg.axes) else 0.0
        # axis 3 typically [-1,1] → scale [0,1]
        if self.scale_axis < len(msg.axes):
            scale = (msg.axes[self.scale_axis] + 1.0) / 2.0
        else:
            scale = 1.0

        # Differential drive in [-1, 1]
        left = max(-1.0, min(1.0, (lin - ang) * scale))
        right = max(-1.0, min(1.0, (lin + ang) * scale))
        self.left = left
        self.right = right

    def _tick(self) -> None:
        cmd = TeleopCommand(left=self.left, right=self.right, arm=self.arm, seq=self.seq)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.ser.write(cmd.encode_serial())
        except Exception as exc:  # noqa: BLE001 — keep node alive on radio glitches
            self.get_logger().error("Serial write failed: %s", exc, throttle_duration_sec=2.0)

    def destroy_node(self) -> None:
        try:
            # Disarm on exit
            self.ser.write(TeleopCommand(0.0, 0.0, arm=False, seq=self.seq).encode_serial())
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=os.environ.get("RADIO_SERIAL_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--baud", type=int, default=int(os.environ.get("RADIO_BAUD", "57600")))
    parser.add_argument("--linear-axis", type=int, default=1)
    parser.add_argument("--angular-axis", type=int, default=2)
    parser.add_argument("--scale-axis", type=int, default=3)
    parser.add_argument("--deadman-button", type=int, default=0)
    parser.add_argument("--rate", type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = JoyToSerial(
        serial_port=args.port,
        baud=args.baud,
        linear_axis=args.linear_axis,
        angular_axis=args.angular_axis,
        scale_axis=args.scale_axis,
        deadman_button=args.deadman_button,
        rate_hz=args.rate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
