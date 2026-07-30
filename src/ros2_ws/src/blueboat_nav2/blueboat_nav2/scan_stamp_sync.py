#!/usr/bin/env python3
"""Re-stamp LaserScan messages to the node clock for Nav2 costmap TF lookups.

pointcloud_to_laserscan copies the PointCloud2 header stamp. On the field stack the
Airy driver often publishes clouds whose stamps lag estimated odometry by tens of
seconds. Nav2 accepts scans briefly, then drops them once the TF buffer only holds
recent transforms ("timestamp earlier than all the data in the transform cache").
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanStampSync(Node):
    def __init__(self) -> None:
        super().__init__("scan_stamp_sync")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, input_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"Re-stamping {input_topic} -> {output_topic}")

    def _cb(self, msg: LaserScan) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = ScanStampSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
