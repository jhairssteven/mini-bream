"""Launch RoboSense RS-LiDAR-AIRY via rslidar_sdk with frontseat airy.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Humble defaults to Fast-DDS which can drop Airy frame rate; prefer Cyclone.
    if os.environ.get("ROS_DISTRO") == "humble" and not os.environ.get("RMW_IMPLEMENTATION"):
        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    config_path = os.path.join(
        get_package_share_directory("frontseat"),
        "config",
        "rslidar_airy",
        "airy.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
        ]
    )
