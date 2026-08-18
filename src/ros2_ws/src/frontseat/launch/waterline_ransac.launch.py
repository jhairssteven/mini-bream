"""Launch RANSAC waterline fitting for RoboSense point clouds."""

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
            description='Path to waterline YAML (default: package waterline.yaml)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (set true when playing a rosbag with --clock)',
        ),
        Node(
            package='frontseat',
            executable='waterline_ransac',
            name='waterline_ransac',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
            }],
        ),
    ])
