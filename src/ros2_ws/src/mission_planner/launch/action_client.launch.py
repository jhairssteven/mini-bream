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

    client_type_arg = DeclareLaunchArgument(
        'client_type',
        default_value='API',
        description='Client mode: API (interactive) or FILE (autonomous mission)'
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
        LaunchConfiguration('mission_name'),
        'mission.csv'
    ])

    # Group for FILE-based mission loading
    file_mode_group = GroupAction([
        Node(
            package='mission_planner',
            executable='nav_goal_to_waypoint',
            name='action_client',
            output='screen',
            parameters=[
                {'mission': mission_csv_path}
            ]
        )
    ], condition=IfCondition(PythonExpression(["'", LaunchConfiguration('client_type'), "' == 'FILE'"])))

    # Default node launch
    default_node = Node(
        package='mission_planner',
        executable='nav_goal_to_waypoint',
        name='action_client',
        output='screen',
        parameters=[vehicle_param_file]
    )

    return LaunchDescription([
        vehicle_arg,
        client_type_arg,
        mission_name_arg,
        # The vehicle-specific parameter file (passed to node in both cases)
        default_node,
        # Conditional mission parameter if FILE mode is used
        file_mode_group,
    ])
