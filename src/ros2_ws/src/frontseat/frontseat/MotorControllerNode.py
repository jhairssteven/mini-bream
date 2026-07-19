#! /usr/bin/python3
"""ROS bridge: /pwm/*_thrust_cmd → PWM daemon (does NOT touch GPIO)."""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from frontseat.qos_profiles import reliable_volatile_qos

# Mounted at /opt/teleop in the prod container
import sys

sys.path.insert(0, os.environ.get("TELEOP_PATH", "/opt/teleop"))
from ipc import PwmDaemonClient  # noqa: E402
from protocol import SOURCE_ROS  # noqa: E402


class MotorControllerNode(Node):
    def __init__(self, node_name: str = "motor_controller"):
        super().__init__(node_name)
        self.right_max_thrust_pgt = 0.8
        self.left_max_thrust_pgt = 1.0
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.seq = 0

        host = os.environ.get("PWM_DAEMON_HOST", "127.0.0.1")
        port = int(os.environ.get("PWM_DAEMON_PORT", "5600"))
        self.client = PwmDaemonClient(host=host, port=port)
        self.get_logger().info(f"PWM daemon at udp://{host}:{port}")

        self.create_subscription(
            Float32, "/pwm/right_thrust_cmd", self.__right_thrust_cbk, reliable_volatile_qos
        )
        self.create_subscription(
            Float32, "/pwm/left_thrust_cmd", self.__left_thrust_cbk, reliable_volatile_qos
        )
        # Keep daemon fed even if only one side updates
        self.create_timer(0.05, self.__publish_to_daemon)

    def __left_thrust_cbk(self, msg: Float32):
        self.left_thrust = float(msg.data) * self.left_max_thrust_pgt

    def __right_thrust_cbk(self, msg: Float32):
        self.right_thrust = float(msg.data) * self.right_max_thrust_pgt

    def __publish_to_daemon(self):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        self.client.send(
            source=SOURCE_ROS,
            left=self.left_thrust,
            right=self.right_thrust,
            arm=True,
            seq=self.seq,
        )

    def destroy_node(self):
        try:
            self.client.send(SOURCE_ROS, 0.0, 0.0, arm=False)
            self.client.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
