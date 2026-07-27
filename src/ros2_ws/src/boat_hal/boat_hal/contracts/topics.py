"""Canonical ROS topic names shared by simulation and field hardware."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class TopicContract:
    """Stable interface between navigation stack and HAL providers."""

    gps: str = "/blueboat/sensors/gps/gps/fix"
    imu: str = "/blueboat/sensors/imu/imu/data"
    ground_truth_odometry: str = "/blueboat/sensors/position/ground_truth_odometry"
    left_thrust: str = "/blueboat/thrusters/left/thrust"
    right_thrust: str = "/blueboat/thrusters/right/thrust"
    pwm_left: str = "/pwm/left_thrust_cmd"
    pwm_right: str = "/pwm/right_thrust_cmd"
    lidar_points: str = "/rslidar_points"
    lidar_points_filtered: str = "/rslidar_points/filtered"
    lidar_imu: str = "/rslidar_imu_data"
    zed_rgb_image: str = "/zed/zed/rgb/color/rect/image"
    zed_rgb_camera_info: str = "/zed/zed/rgb/color/rect/camera_info"
    zed_point_cloud: str = "/zed/zed/point_cloud/cloud_registered"
    clock: str = "/clock"
    tf_static: str = "/tf_static"
    tf: str = "/tf"

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TopicContract":
        fields = {key: value for key, value in data.items() if hasattr(cls, key)}
        return cls(**fields)

    def as_dict(self) -> Dict[str, str]:
        return {
            "gps": self.gps,
            "imu": self.imu,
            "ground_truth_odometry": self.ground_truth_odometry,
            "left_thrust": self.left_thrust,
            "right_thrust": self.right_thrust,
            "pwm_left": self.pwm_left,
            "pwm_right": self.pwm_right,
            "lidar_points": self.lidar_points,
            "lidar_points_filtered": self.lidar_points_filtered,
            "lidar_imu": self.lidar_imu,
            "zed_rgb_image": self.zed_rgb_image,
            "zed_rgb_camera_info": self.zed_rgb_camera_info,
            "zed_point_cloud": self.zed_point_cloud,
            "clock": self.clock,
            "tf_static": self.tf_static,
            "tf": self.tf,
        }
