#!/usr/bin/env python3
"""Estimate body-frame surge/sway/yaw-rate from GPS and IMU for MPC feedback."""

from __future__ import annotations

import argparse
import math
from typing import Optional, Tuple

import rclpy
import utm
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from tf_transformations import euler_from_quaternion


def gps_to_local(lat: float, lon: float, origin_utm: Tuple[float, float]) -> Tuple[float, float]:
    x, y, _, _ = utm.from_latlon(lat, lon)
    return x - origin_utm[0], y - origin_utm[1]


class VelocityOdomNode(Node):
    def __init__(
        self,
        gps_topic: str,
        imu_topic: str,
        odom_topic: str,
        origin_lat: float,
        origin_lon: float,
        frame_id: str = "world",
        child_frame_id: str = "base_link",
        window_s: float = 0.4,
    ):
        super().__init__("molo_boat_velocity_odom")
        self._frame_id = frame_id
        self._child_frame_id = child_frame_id
        self._origin_utm = utm.from_latlon(origin_lat, origin_lon)[:2]
        self._window_s = max(window_s, 0.1)

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_gps = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, qos_gps)
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos_sensor)
        self._pub = self.create_publisher(Odometry, odom_topic, qos_pub)

        self._psi: Optional[float] = None
        self._r = 0.0
        self._last_xy: Optional[Tuple[float, float]] = None
        self._last_t: Optional[float] = None
        self._u = 0.0
        self._v = 0.0

        self.create_timer(0.1, self._publish)

    def _yaw_from_imu(self, msg: Imu) -> float:
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        return float(yaw)

    def _imu_cb(self, msg: Imu) -> None:
        self._psi = self._yaw_from_imu(msg)
        self._r = float(msg.angular_velocity.z)

    def _gps_cb(self, msg: NavSatFix) -> None:
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        if self._psi is None:
            return

        x, y = gps_to_local(msg.latitude, msg.longitude, self._origin_utm)
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._last_xy is not None and self._last_t is not None:
            dt = t - self._last_t
            if dt > 1e-3:
                vx_world = (x - self._last_xy[0]) / dt
                vy_world = (y - self._last_xy[1]) / dt
                c = math.cos(self._psi)
                s = math.sin(self._psi)
                u_raw = c * vx_world + s * vy_world
                v_raw = -s * vx_world + c * vy_world
                alpha = min(1.0, dt / self._window_s)
                self._u = (1.0 - alpha) * self._u + alpha * u_raw
                self._v = (1.0 - alpha) * self._v + alpha * v_raw

        self._last_xy = (x, y)
        self._last_t = t

    def _publish(self) -> None:
        if self._last_xy is None or self._psi is None:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id
        msg.pose.pose.position.x = self._last_xy[0]
        msg.pose.pose.position.y = self._last_xy[1]
        msg.twist.twist.linear.x = self._u
        msg.twist.twist.linear.y = self._v
        msg.twist.twist.angular.z = self._r
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="GPS+IMU velocity odometry for boat MPC")
    parser.add_argument("--gps-topic", default="/wamv/sensors/gps/gps/fix")
    parser.add_argument("--imu-topic", default="/wamv/sensors/imu/imu/data")
    parser.add_argument("--odom-topic", default="/molo_boat/estimated_odometry")
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--child-frame-id", default="base_link")
    parser.add_argument("--window-s", type=float, default=0.4)
    args = parser.parse_args()

    rclpy.init()
    node = VelocityOdomNode(
        args.gps_topic,
        args.imu_topic,
        args.odom_topic,
        args.origin_lat,
        args.origin_lon,
        frame_id=args.frame_id,
        child_frame_id=args.child_frame_id,
        window_s=args.window_s,
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
