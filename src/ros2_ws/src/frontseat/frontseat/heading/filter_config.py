"""Load centralized heading filter parameters from ROS."""

from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node


@dataclass(frozen=True)
class LowPassFilterConfig:
    enabled: bool
    alpha: float


@dataclass(frozen=True)
class HeadingEkfConfig:
    enabled: bool
    process_noise_var: float
    imu_topic: str
    gps_heading_topic: str
    output_imu_topic: str
    output_deg_topic: str
    output_marker_topic: str
    publish_rate_hz: float
    gyro_z_max_rad_s: float
    max_variance_rad2: float
    gps_variance_scale: float
    max_innovation_reset_deg: float
    imu_timeout_s: float
    use_imu_predict: bool


@dataclass(frozen=True)
class HeadingFilterConfig:
    low_pass: LowPassFilterConfig
    ekf: HeadingEkfConfig


def declare_heading_filter_parameters(node: Node) -> None:
    """Declare the shared heading filter parameter tree on a node."""
    node.declare_parameter('low_pass.enabled', True)
    node.declare_parameter('low_pass.alpha', 0.25)

    node.declare_parameter('ekf.enabled', False)
    node.declare_parameter('ekf.process_noise_var', 0.01)
    node.declare_parameter('ekf.imu_topic', '/zed/zed/imu/data')
    node.declare_parameter('ekf.gps_heading_topic', '/baseline/heading/raw')
    node.declare_parameter('ekf.output_imu_topic', '/baseline/heading')
    node.declare_parameter('ekf.output_deg_topic', '/baseline/heading/deg')
    node.declare_parameter('ekf.output_marker_topic', '/baseline/heading/marker/ekf')
    node.declare_parameter('ekf.publish_rate_hz', 50.0)
    node.declare_parameter('ekf.gyro_z_max_rad_s', 3.0)
    node.declare_parameter('ekf.max_variance_rad2', 0.01)
    node.declare_parameter('ekf.gps_variance_scale', 0.05)
    node.declare_parameter('ekf.max_innovation_reset_deg', 25.0)
    node.declare_parameter('ekf.imu_timeout_s', 0.25)
    node.declare_parameter('ekf.use_imu_predict', False)


def load_heading_filter_config(node: Node) -> HeadingFilterConfig:
    declare_heading_filter_parameters(node)
    return HeadingFilterConfig(
        low_pass=LowPassFilterConfig(
            enabled=bool(node.get_parameter('low_pass.enabled').value),
            alpha=float(node.get_parameter('low_pass.alpha').value),
        ),
        ekf=HeadingEkfConfig(
            enabled=bool(node.get_parameter('ekf.enabled').value),
            process_noise_var=float(node.get_parameter('ekf.process_noise_var').value),
            imu_topic=str(node.get_parameter('ekf.imu_topic').value),
            gps_heading_topic=str(node.get_parameter('ekf.gps_heading_topic').value),
            output_imu_topic=str(node.get_parameter('ekf.output_imu_topic').value),
            output_deg_topic=str(node.get_parameter('ekf.output_deg_topic').value),
            output_marker_topic=str(node.get_parameter('ekf.output_marker_topic').value),
            publish_rate_hz=float(node.get_parameter('ekf.publish_rate_hz').value),
            gyro_z_max_rad_s=float(node.get_parameter('ekf.gyro_z_max_rad_s').value),
            max_variance_rad2=float(node.get_parameter('ekf.max_variance_rad2').value),
            gps_variance_scale=float(node.get_parameter('ekf.gps_variance_scale').value),
            max_innovation_reset_deg=float(node.get_parameter('ekf.max_innovation_reset_deg').value),
            imu_timeout_s=float(node.get_parameter('ekf.imu_timeout_s').value),
            use_imu_predict=bool(node.get_parameter('ekf.use_imu_predict').value),
        ),
    )
