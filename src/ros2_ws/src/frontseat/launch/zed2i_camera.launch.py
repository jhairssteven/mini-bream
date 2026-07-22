"""Launch ZED 2i with minimal topic set (RGB, camera_info, point cloud)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    override = os.path.join(
        get_package_share_directory("frontseat"),
        "config",
        "zed2i",
        "zed2i_minimal.yaml",
    )

    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"),
        "launch",
        "zed_camera.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    "camera_model": "zed2i",
                    "camera_name": "zed",
                    "namespace": "zed",
                    "ros_params_override_path": override,
                    "base_frame": "base_link",
                    "publish_tf": "true",
                    "publish_map_tf": "false",
                    "publish_urdf": "false",
                    "publish_imu_tf": "false",
                    "enable_ipc": "false",
                }.items(),
            ),
        ]
    )
