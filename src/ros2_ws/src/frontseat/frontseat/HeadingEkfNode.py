#!/usr/bin/env python3
"""EKF fusion of GPS RTK heading with ZED IMU gyro for high-rate nav output."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from frontseat.heading.angles import normalize_angle, yaw_from_quaternion
from frontseat.heading.filter_config import load_heading_filter_config
from frontseat.heading.filters import HeadingEkf
from frontseat.heading.ros_msgs import build_heading_marker, build_imu_msg
from frontseat.qos_profiles import best_effort_volatile_qos, reliable_volatile_qos


class HeadingEkfNode(Node):
    def __init__(self) -> None:
        super().__init__('heading_ekf')

        self.declare_parameter('heading_frame_id', 'gps_center_link')
        self.declare_parameter('marker_frame_id', 'base_link')
        self.declare_parameter('arrow_length', 0.65)

        filter_config = load_heading_filter_config(self)
        ekf_config = filter_config.ekf
        self._enabled = ekf_config.enabled
        self._heading_frame_id = self.get_parameter('heading_frame_id').value
        self._marker_frame_id = self.get_parameter('marker_frame_id').value
        self._arrow_length = float(self.get_parameter('arrow_length').value)
        self._gyro_z_max = ekf_config.gyro_z_max_rad_s
        self._max_innovation_reset_rad = math.radians(ekf_config.max_innovation_reset_deg)
        self._imu_timeout_s = ekf_config.imu_timeout_s
        self._use_imu_predict = ekf_config.use_imu_predict

        self._ekf = HeadingEkf(
            process_noise_var=ekf_config.process_noise_var,
            max_variance_rad2=ekf_config.max_variance_rad2,
            gps_variance_scale=ekf_config.gps_variance_scale,
        )
        self._last_imu_stamp: rclpy.time.Time | None = None
        self._last_gps_stamp = None
        self._last_gps_wall_time: rclpy.time.Time | None = None
        self._last_imu_wall_time: rclpy.time.Time | None = None
        self._last_gps_variance_rad2 = 0.1
        self._last_raw_yaw_rad: float | None = None
        self._max_predict_gap_s = 0.2

        self._imu_pub = self.create_publisher(
            Imu, ekf_config.output_imu_topic, best_effort_volatile_qos
        )
        self._deg_pub = self.create_publisher(
            Float32, ekf_config.output_deg_topic, reliable_volatile_qos
        )
        self._marker_pub = self.create_publisher(
            Marker, ekf_config.output_marker_topic, 10
        )

        if not self._enabled:
            self.get_logger().info('heading_ekf disabled by parameter; node idle')
            return

        self.create_subscription(
            Imu,
            ekf_config.gps_heading_topic,
            self._gps_heading_cb,
            best_effort_volatile_qos,
        )
        self.create_subscription(
            Imu,
            ekf_config.imu_topic,
            self._zed_imu_cb,
            reliable_volatile_qos,
        )
        self.create_timer(1.0 / ekf_config.publish_rate_hz, self._publish_timer_cb)

        self.get_logger().info(
            f'heading_ekf active: GPS={ekf_config.gps_heading_topic}, '
            f'IMU={ekf_config.imu_topic} -> {ekf_config.output_imu_topic}'
        )

    def _gps_heading_cb(self, msg: Imu) -> None:
        yaw_rad = yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        variance_rad2 = msg.orientation_covariance[8]
        if variance_rad2 <= 0.0:
            variance_rad2 = 0.1
        self._last_gps_variance_rad2 = variance_rad2
        self._last_raw_yaw_rad = yaw_rad
        self._last_gps_stamp = msg.header.stamp
        self._last_gps_wall_time = self.get_clock().now()
        if self._ekf.initialized:
            state = self._ekf.state()
            assert state is not None
            innovation = abs(normalize_angle(yaw_rad - state.yaw_rad))
            if innovation > self._max_innovation_reset_rad:
                self.get_logger().warning(
                    f'heading_ekf innovation {math.degrees(innovation):.1f} deg; snapping to GPS'
                )
                self._ekf.reset()
        self._ekf.update(yaw_rad, variance_rad2)

    def _zed_imu_cb(self, msg: Imu) -> None:
        if not self._use_imu_predict or not self._ekf.initialized:
            return

        now = self.get_clock().now()
        if self._last_imu_wall_time is not None:
            imu_gap_s = (now - self._last_imu_wall_time).nanoseconds * 1e-9
            if imu_gap_s > self._imu_timeout_s:
                self._last_imu_stamp = None
        self._last_imu_wall_time = now

        if (
            self._last_gps_wall_time is not None
            and (now - self._last_gps_wall_time).nanoseconds * 1e-9 > self._max_predict_gap_s
        ):
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp
            return

        dt_s = (stamp - self._last_imu_stamp).nanoseconds * 1e-9
        self._last_imu_stamp = stamp
        if dt_s <= 0.0 or dt_s > 1.0:
            return

        gyro_z = max(-self._gyro_z_max, min(self._gyro_z_max, msg.angular_velocity.z))
        self._ekf.predict(gyro_z, dt_s)

    def _publish_timer_cb(self) -> None:
        state = self._ekf.state()
        if state is None:
            return

        stamp = (
            self._last_gps_stamp
            if self._last_gps_stamp is not None
            else self.get_clock().now().to_msg()
        )
        marker_yaw = 0.0
        if self._last_raw_yaw_rad is not None:
            marker_yaw = normalize_angle(state.yaw_rad - self._last_raw_yaw_rad)
        self._imu_pub.publish(
            build_imu_msg(
                stamp,
                self._heading_frame_id,
                state.yaw_rad,
                state.variance_rad2,
            )
        )
        self._deg_pub.publish(Float32(data=state.yaw_rad * 180.0 / math.pi))
        self._marker_pub.publish(
            build_heading_marker(
                stamp,
                self._marker_frame_id,
                marker_yaw,
                namespace='heading_ekf',
                color=(0.1, 0.85, 0.35, 1.0),
                arrow_length=self._arrow_length,
                position_offset=(0.0, -0.15, 0.0),
                body_aligned=True,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingEkfNode()
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
