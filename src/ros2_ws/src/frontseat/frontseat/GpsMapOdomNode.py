#!/usr/bin/env python3
"""Publish map->base_link from GPS center fix and heading; origin latched at stack start."""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from frontseat.heading.angles import normalize_angle, yaw_from_quaternion
from frontseat.heading.geo import latlon_to_local_enu
from frontseat.heading.ros_msgs import yaw_to_quaternion
from frontseat.qos_profiles import best_effort_volatile_qos


class GpsMapOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('gps_map_odom')

        self.declare_parameter('gps_topic', '/gps/center')
        self.declare_parameter('heading_topic', '/baseline/heading/raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('min_fix_status', 0)
        self.declare_parameter('flatten_z', True)
        self.declare_parameter('recent_path_max_points', 150)
        self.declare_parameter('recent_path_topic', '/gps/center/recent_path')
        self.declare_parameter('velocity_window_s', 0.4)

        gps_topic = self.get_parameter('gps_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        self._min_fix_status = int(self.get_parameter('min_fix_status').value)
        self._flatten_z = bool(self.get_parameter('flatten_z').value)
        self._recent_path_max = max(2, int(self.get_parameter('recent_path_max_points').value))
        self._velocity_window_s = max(0.1, float(self.get_parameter('velocity_window_s').value))
        recent_path_topic = self.get_parameter('recent_path_topic').value

        self._origin_lat: float | None = None
        self._origin_lon: float | None = None
        self._origin_alt: float | None = None
        self._last_yaw_rad: float | None = None
        self._last_position_enu: tuple[float, float, float] | None = None
        self._prev_xy: tuple[float, float] | None = None
        self._prev_yaw: float | None = None
        self._prev_t: float | None = None
        self._u = 0.0
        self._v = 0.0
        self._r = 0.0
        self._recent_points: deque[tuple[float, float]] = deque(maxlen=self._recent_path_max)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, best_effort_volatile_qos
        )
        self._recent_path_pub = self.create_publisher(Marker, recent_path_topic, 10)

        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, best_effort_volatile_qos)
        self.create_subscription(Imu, heading_topic, self._heading_cb, best_effort_volatile_qos)
        self.create_timer(0.1, self._republish_cb)

        z_mode = 'flattened (2D)' if self._flatten_z else 'GPS altitude'
        self.get_logger().info(
            f'gps_map_odom: GPS={gps_topic}, heading={heading_topic}, '
            f'{self._map_frame}->{self._base_frame}, z={z_mode}, '
            f'recent_path={recent_path_topic} (max {self._recent_path_max})'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

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
        east, north, _ = self._last_position_enu
        self._recent_points.append((east, north))
        self._update_twist(east, north)
        self._publish()

    def _heading_cb(self, msg: Imu) -> None:
        if msg.orientation_covariance[8] < 0.0:
            return
        self._last_yaw_rad = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        if msg.angular_velocity.z != 0.0:
            self._r = float(msg.angular_velocity.z)
        self._publish()

    def _update_twist(self, east: float, north: float) -> None:
        t = self._now_sec()
        yaw = self._last_yaw_rad if self._last_yaw_rad is not None else 0.0
        if self._prev_xy is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-3:
                vx = (east - self._prev_xy[0]) / dt
                vy = (north - self._prev_xy[1]) / dt
                c = math.cos(yaw)
                s = math.sin(yaw)
                u_raw = c * vx + s * vy
                v_raw = -s * vx + c * vy
                alpha = min(1.0, dt / self._velocity_window_s)
                self._u = (1.0 - alpha) * self._u + alpha * u_raw
                self._v = (1.0 - alpha) * self._v + alpha * v_raw
                if self._prev_yaw is not None:
                    r_raw = normalize_angle(yaw - self._prev_yaw) / dt
                    self._r = (1.0 - alpha) * self._r + alpha * r_raw
        self._prev_xy = (east, north)
        self._prev_yaw = yaw
        self._prev_t = t

    def _republish_cb(self) -> None:
        self._publish()

    def _publish(self) -> None:
        if self._last_position_enu is None:
            return

        stamp = self.get_clock().now().to_msg()
        east, north, up = self._last_position_enu
        if self._flatten_z:
            up = 0.0

        yaw_rad = self._last_yaw_rad if self._last_yaw_rad is not None else 0.0
        quat = yaw_to_quaternion(yaw_rad)

        self._publish_recent_path(stamp)

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
        odom.twist.twist.linear.x = self._u
        odom.twist.twist.linear.y = self._v
        odom.twist.twist.angular.z = self._r
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

    def _publish_recent_path(self, stamp) -> None:
        if len(self._recent_points) < 2:
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._map_frame
        marker.ns = 'gps_center_recent'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color = ColorRGBA(r=1.0, g=0.65, b=0.1, a=0.9)
        marker.pose.orientation.w = 1.0

        for x, y in self._recent_points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05
            marker.points.append(point)

        self._recent_path_pub.publish(marker)


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
