#!/usr/bin/env python3
"""Moving-base RTK: Publish average GPS + heading receivers' NAV-RELPOSNED"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from ublox_msgs.msg import NavRELPOSNED9
from visualization_msgs.msg import Marker

from frontseat.heading.relpos import parse_nav_relposned9
from frontseat.heading.ros_msgs import build_center_fix, build_heading_marker, build_imu_msg
from frontseat.qos_profiles import best_effort_volatile_qos, reliable_volatile_qos


class RelPosHeadingNode(Node):
    def __init__(self) -> None:
        super().__init__('rel_pos_heading')

        self.declare_parameter('navrelposned_topic', '/navrelposned')
        self.declare_parameter('rover_fix_topic', '/fix/rover')
        self.declare_parameter('base_fix_topic', '/fix/base')
        self.declare_parameter('center_fix_topic', '/fix/center')
        self.declare_parameter('heading_frame_id', 'gps_center_link')
        self.declare_parameter('imu_topic', '/baseline/heading')
        self.declare_parameter('heading_deg_topic', '/baseline/heading/deg')
        self.declare_parameter('marker_topic', '/baseline/heading/marker')
        self.declare_parameter('gps_publish_rate_hz', 8.0)
        self.declare_parameter('arrow_length', 0.65)

        navrelposned_topic = self.get_parameter('navrelposned_topic').value
        rover_fix_topic = self.get_parameter('rover_fix_topic').value
        base_fix_topic = self.get_parameter('base_fix_topic').value
        center_fix_topic = self.get_parameter('center_fix_topic').value
        self._heading_frame_id = self.get_parameter('heading_frame_id').value
        imu_topic = self.get_parameter('imu_topic').value
        heading_deg_topic = self.get_parameter('heading_deg_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        gps_publish_rate_hz = float(self.get_parameter('gps_publish_rate_hz').value)
        self._arrow_length = float(self.get_parameter('arrow_length').value)

        self._rover_fix: NavSatFix | None = None
        self._base_fix: NavSatFix | None = None

        self._imu_pub = self.create_publisher(Imu, imu_topic, best_effort_volatile_qos)
        self._deg_pub = self.create_publisher(Float32, heading_deg_topic, reliable_volatile_qos)
        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self._center_fix_pub = self.create_publisher(NavSatFix, center_fix_topic, best_effort_volatile_qos)

        self.create_subscription(
            NavRELPOSNED9, navrelposned_topic, self._relpos_cb, reliable_volatile_qos
        )
        self.create_subscription(
            NavSatFix, rover_fix_topic, self._rover_fix_cb, reliable_volatile_qos
        )
        self.create_subscription(
            NavSatFix, base_fix_topic, self._base_fix_cb, reliable_volatile_qos
        )
        self.create_timer(1.0 / gps_publish_rate_hz, self._publish_center_fix)

        self.get_logger().info(
            f'Moving-base RTK: heading from {navrelposned_topic}, '
            f'center GPS from {rover_fix_topic}+{base_fix_topic}, '
            f'frame={self._heading_frame_id}'
        )

    def _rover_fix_cb(self, msg: NavSatFix) -> None:
        self._rover_fix = msg

    def _base_fix_cb(self, msg: NavSatFix) -> None:
        self._base_fix = msg

    def _publish_center_fix(self) -> None:
        if self._rover_fix is None or self._base_fix is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._center_fix_pub.publish(
            build_center_fix(stamp, self._heading_frame_id, self._rover_fix, self._base_fix)
        )

    def _relpos_cb(self, msg: NavRELPOSNED9) -> None:
        measurement = parse_nav_relposned9(msg)
        if not measurement.heading_valid:
            self.get_logger().warning('NAV-RELPOSNED heading invalid; skipping update')
            return

        stamp = self.get_clock().now().to_msg()
        yaw_rad = measurement.yaw_enu_rad

        self._imu_pub.publish(
            build_imu_msg(
                stamp,
                self._heading_frame_id,
                yaw_rad,
                measurement.variance_rad2,
            )
        )
        self._deg_pub.publish(Float32(data=yaw_rad * 180.0 / math.pi))
        self._marker_pub.publish(
            build_heading_marker(
                stamp,
                self._heading_frame_id,
                yaw_rad,
                namespace='heading_rel_pos',
                color=(0.5, 0.0, 0.8, 1.0),
                arrow_length=self._arrow_length,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelPosHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
