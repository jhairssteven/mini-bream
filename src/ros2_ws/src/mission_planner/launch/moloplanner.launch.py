from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Data mocking
        Node(
            package='mission_planner',
            executable='mock_planner_data_pub',
            name='mock_planner_data_publisher',
            output='screen'
        ),
        Node(
            package='mission_planner',
            executable='moloplanner_node',
            name='moloplanner_node',
            output='screen',
            parameters=[
                {"config_path": "/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/config.yaml"},
                {"overrides": ["depth_pipeline.max_depth=20"]}
            ]
        ),
    ])
