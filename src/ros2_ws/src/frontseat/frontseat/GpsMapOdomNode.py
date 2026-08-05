#!/usr/bin/env python3
"""Publish map->base_link from GPS center fix and heading; origin latched at stack start."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from frontseat.heading.angles import yaw_from_quaternion
from frontseat.heading.geo import latlon_to_local_enu
from frontseat.heading.ros_msgs import yaw_to_quaternion
from frontseat.qos_profiles import best_effort_volatile_qos, reliable_volatile_qos


class GpsMapOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_map_odom')

        self.declare_parameter('gps_topic', '/fix/center')
        self.declare_parameter('heading_topic', '/baseline/heading/raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('min_fix_status', 0)

        gps_topic = self.get_parameter('gps_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._min_fix_status = int(self.get_parameter('min_fix_status').value)

        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._origin_alt: float | None = None
        self._last_yaw_rad: float | None = None
        self._last_position_enu: tuple[float, float, float] | None = None
        self._last_stamp = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, best_effort_volatile_qos
        )

        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, best_effort_volatile_qos)
        self.create_subscription(Imu, heading_topic, self._heading_cb, best_effort_volatile_qos)
        self.create_timer(0.1, self._republish_cb)

        self.get_logger().info(
            f'gps_map_odom: GPS={gps_topic}, heading={heading_topic}, '
            f'{self._map_frame}->{self._base_frame}'
        )

    def _gps_cb(self, msg: NavSatFix) -> None:
        if msg.status.status < self._min_fix_status:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        if self._origin_lat is None:
            self._origin_lat = msg.latitude
            self._origin_lon = msg.longitude
            self._origin_alt = msg.altitude
            self.get_logger().info(
                f'Map origin latched: lat={self._origin_lat:.8f}, '
                f'lon={self._origin_lon:.8f}, alt={self._origin_alt:.3f}'
            )

        assert self._origin_lat is not None
        assert self._origin_lon is not None
        assert self._origin_alt is not None

        self._last_position_enu = latlon_to_local_enu(
            msg.latitude,
            msg.longitude,
            msg.altitude,
            self._origin_lat,
            self._origin_lon,
            self._origin_alt,
        )
        self._last_stamp = msg.header.stamp
        self._publish()

    def _heading_cb(self, msg: Imu) -> None:
        # build_imu_msg marks roll/pitch unknown (cov[0,4]=-1); yaw variance is at [8].
        if msg.orientation_covariance[8] < 0.0:
            return
        self._last_yaw_rad = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self._publish()

    def _republish_cb(self) -> None:
        self._publish()

    def _publish(self) -> None:
        if self._last_position_enu is None:
            return

        stamp = self.get_clock().now().to_msg()
        east, north, up = self._last_position_enu
        yaw_rad = self._last_yaw_rad if self._last_yaw_rad is not None else 0.0
        quat = yaw_to_quaternion(yaw_rad)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation = quat
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 0.25
        odom.pose.covariance[35] = 0.05
        self._odom_pub.publish(odom)

        if not self._publish_tf:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._map_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = east
        tf_msg.transform.translation.y = north
        tf_msg.transform.translation.z = up
        tf_msg.transform.rotation = quat
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsMapOdomNode()
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
