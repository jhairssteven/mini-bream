"""Launch the LiDAR filtering pipeline: self-filter → waterline RANSAC → clustering."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool,
    )
    debug_enabled = ParameterValue(
        LaunchConfiguration('debug_enabled'), value_type=bool,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock (set true when playing a rosbag with --clock)',
        ),
        DeclareLaunchArgument(
            'debug_enabled',
            default_value='false',
            description=(
                'Enable debug markers and removed-cloud publishing on '
                'self-filter and clustering (ORs with YAML)'
            ),
        ),
        DeclareLaunchArgument(
            'self_filter_config_path',
            default_value='',
            description=(
                'Self-filter YAML (default: package blueboat_self_filter.yaml)'
            ),
        ),
        DeclareLaunchArgument(
            'ransac_config_path',
            default_value='',
            description='Waterline YAML (default: package waterline.yaml)',
        ),
        DeclareLaunchArgument(
            'clustering_config_path',
            default_value='',
            description='Clustering YAML (default: package clustering.yaml)',
        ),
        Node(
            package='frontseat',
            executable='self_filter',
            name='self_filter',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('self_filter_config_path'),
                'debug_enabled': debug_enabled,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='frontseat',
            executable='waterline_ransac',
            name='waterline_ransac',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('ransac_config_path'),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='frontseat',
            executable='pc_clustering',
            name='pc_clustering',
            output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('clustering_config_path'),
                'use_sim_time': use_sim_time,
                'debug_enabled': debug_enabled,
            }],
        ),
    ])
