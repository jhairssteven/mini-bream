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

    load_mission_from_file_arg = DeclareLaunchArgument(
        'load_mission_from_file',
        default_value='false',
        description='If True, loads mission waypoints from a CSV file. If False, runs in online/interactive mode.'
    )

    mission_name_arg = DeclareLaunchArgument(
        'mission_name',
        default_value='default',
        description='Directory name containing the mission.csv file'
    )
    # TODO: Update to ros2
    # Path to parameter YAML based on vehicle
    vehicle_param_file = PathJoinSubstitution([
        FindPackageShare('backseat'),
        'params',
        [LaunchConfiguration('vehicle'), '.yaml']
    ])

    # Path to mission file if client_type == FILE
    mission_csv_path = PathJoinSubstitution([
        FindPackageShare('mission_planner'),
        'missions',
        [LaunchConfiguration('mission_name'), '.csv']
    ])

    action_client = Node(
        package='mission_planner',
        executable='nav_goal_to_waypoint',
        name='action_client',
        output='screen',
        parameters=[
            {
                'mission': mission_csv_path,
                'load_mission_from_file': LaunchConfiguration('load_mission_from_file')
            },
            vehicle_param_file,
        ]
    )

    return LaunchDescription([
        vehicle_arg,
        load_mission_from_file_arg,
        mission_name_arg,
        action_client,
    ])
