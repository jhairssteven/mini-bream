from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the vehicle argument
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='vrx',
        description='Vehicle configuration (e.g., vrx, mini_bream, ground_vehicle)'
    )

    # Construct the path to the YAML file
    param_file_path = PathJoinSubstitution([
        FindPackageShare('backseat'),
        'params',
        [LaunchConfiguration('vehicle'), '.yaml']
    ])

    planner_node = Node(
        package='backseat',
        executable='PathPlannerNode',
        name='path_planner',
        output='screen',
        parameters=[param_file_path]
    )

    return LaunchDescription([
        vehicle_arg,
        planner_node
    ])
