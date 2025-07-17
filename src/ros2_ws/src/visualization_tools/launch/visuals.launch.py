from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare vehicle argument with default 'vrx'
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='vrx',
        description='Vehicle type for parameter loading'
    )

    # Visualization group: launch rviz with config file
    visualization_group = GroupAction([
        Node(
            package='rviz2',  # ROS2 rviz package is 'rviz2'
            executable='rviz2',
            namespace='visualization',
            name='rviz',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('visualization_tools'),
                    'visualizers_config',
                    'rviz_config',
                    'trajectory.rviz'
                ])
            ],
            output='screen'
        )
    ])

    # Parameters path using vehicle launch arg
    param_file_path = PathJoinSubstitution([
        FindPackageShare('backseat'),
        'params',
        [LaunchConfiguration('vehicle'), '.yaml']
    ])

    # RvizVisualsAdapter group
    rviz_visuals_adapter_group = GroupAction([
        Node(
            package='visualization_tools',
            executable='rviz_visuals_adapter',  # This must match your entry_point in setup.py
            namespace='rvizvisualsadapter',
            name='rviz_visuals_adapter',
            parameters=[param_file_path],
            output='screen'
        )
    ])

    return LaunchDescription([
        vehicle_arg,
        visualization_group,
        rviz_visuals_adapter_group,
    ])
