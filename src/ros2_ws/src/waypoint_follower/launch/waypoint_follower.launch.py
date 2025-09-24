from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Declare input arguments
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='vrx',
        description='Vehicle configuration (e.g., mini_bream, bream, ground_vehicle)'
    )

    vehicle_param_file = PathJoinSubstitution([
        FindPackageShare('backseat'),
        'params',
        [LaunchConfiguration('vehicle'), '.yaml']
    ])


    waypoint_follower = Node(
        package='waypoint_follower',
        executable='wtp_follower',
        name='wtp_follower',
        output='screen',
        parameters=[
            vehicle_param_file,
        ]
    )

    return LaunchDescription([
        vehicle_arg,
        waypoint_follower,
    ])
