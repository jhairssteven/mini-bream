#!/usr/bin/env python3
"""Adapt MPC thrust commands to the pwm_daemon motor path."""

from __future__ import annotations

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32

TELEOP_PATH = os.environ.get("TELEOP_PATH", "/opt/teleop")
if TELEOP_PATH not in sys.path:
    sys.path.insert(0, TELEOP_PATH)


class ThrustBridgeNode(Node):
    """Forward MPC thrust to motor_controller (/pwm/*) or pwm_daemon (UDP)."""

    def __init__(
        self,
        input_left: str,
        input_right: str,
        mode: str,
        pwm_left: str,
        pwm_right: str,
        daemon_host: str,
        daemon_port: int,
    ):
        super().__init__("molo_boat_thrust_bridge")
        self._mode = mode
        self._left = 0.0
        self._right = 0.0
        self._seq = 0
        self._client = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.create_subscription(Float32, input_left, self._left_cb, qos)
        self.create_subscription(Float32, input_right, self._right_cb, qos)

        if mode == "pwm_topics":
            self._left_pub = self.create_publisher(Float32, pwm_left, qos)
            self._right_pub = self.create_publisher(Float32, pwm_right, qos)
        elif mode == "pwm_daemon":
            from ipc import PwmDaemonClient  # noqa: E402
            from protocol import SOURCE_ROS  # noqa: E402

            self._source_ros = SOURCE_ROS
            self._client = PwmDaemonClient(host=daemon_host, port=daemon_port)
            self.get_logger().info(f"PWM daemon udp://{daemon_host}:{daemon_port}")
        elif mode != "log_only":
            raise ValueError(f"unknown thrust mode: {mode}")

        self.create_timer(0.05, self._publish)
        self.get_logger().info(f"thrust bridge mode={mode}")

    def _left_cb(self, msg: Float32) -> None:
        self._left = float(msg.data)

    def _right_cb(self, msg: Float32) -> None:
        self._right = float(msg.data)

    def _publish(self) -> None:
        if self._mode == "log_only":
            if abs(self._left) > 1e-4 or abs(self._right) > 1e-4:
                self.get_logger().info(
                    f"thrust L={self._left:+.3f} R={self._right:+.3f}",
                    throttle_duration_sec=2.0,
                )
            return

        if self._mode == "pwm_topics":
            self._left_pub.publish(Float32(data=self._left))
            self._right_pub.publish(Float32(data=self._right))
            return

        if self._mode == "pwm_daemon" and self._client is not None:
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            self._client.send(
                source=self._source_ros,
                left=self._left,
                right=self._right,
                arm=True,
                seq=self._seq,
            )

    def destroy_node(self):
        if self._client is not None:
            try:
                from protocol import SOURCE_ROS  # noqa: E402

                self._client.send(SOURCE_ROS, 0.0, 0.0, arm=False)
                self._client.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="MPC thrust bridge for pwm_daemon path")
    parser.add_argument("--input-left", default="/molo_boat/thrust_left")
    parser.add_argument("--input-right", default="/molo_boat/thrust_right")
    parser.add_argument(
        "--mode",
        choices=("pwm_topics", "pwm_daemon", "log_only"),
        default="pwm_topics",
        help="pwm_topics: /pwm/*_thrust_cmd → motor_controller; pwm_daemon: UDP direct",
    )
    parser.add_argument("--pwm-left", default="/pwm/left_thrust_cmd")
    parser.add_argument("--pwm-right", default="/pwm/right_thrust_cmd")
    parser.add_argument("--daemon-host", default=os.environ.get("PWM_DAEMON_HOST", "127.0.0.1"))
    parser.add_argument("--daemon-port", type=int, default=int(os.environ.get("PWM_DAEMON_PORT", "5600")))
    args = parser.parse_args()

    rclpy.init()
    node = ThrustBridgeNode(
        args.input_left,
        args.input_right,
        args.mode,
        args.pwm_left,
        args.pwm_right,
        args.daemon_host,
        args.daemon_port,
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
