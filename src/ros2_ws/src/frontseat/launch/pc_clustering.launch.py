"""Launch DBSCAN clustering to drop sparse LiDAR noise."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description=(
                'Path to clustering YAML (default: package clustering.yaml)'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Use /clock (set true when playing a rosbag with --clock)'
            ),
        ),
        DeclareLaunchArgument(
            'debug_enabled',
            default_value='false',
            description=(
                'Enable AABB markers and removed-cloud publishing '
                '(ORs with YAML)'
            ),
        ),
        Node(
            package='frontseat',
            executable='pc_clustering',
            name='pc_clustering',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'debug_enabled': ParameterValue(
                    LaunchConfiguration('debug_enabled'),
                    value_type=bool,
                ),
            }],
        ),
    ])
