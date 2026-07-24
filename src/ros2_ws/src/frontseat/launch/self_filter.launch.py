"""Launch boat hull / mount-bar self-filter for LiDAR point clouds."""

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
            description='Path to self-filter YAML (default: package blueboat_self_filter.yaml)',
        ),
        DeclareLaunchArgument(
            'debug_enabled',
            default_value='false',
            description='Enable debug markers and removed-cloud publishing (overrides YAML)',
        ),
        Node(
            package='frontseat',
            executable='self_filter',
            name='self_filter',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'debug_enabled': ParameterValue(
                    LaunchConfiguration('debug_enabled'),
                    value_type=bool,
                ),
            }],
        ),
    ])
