from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_broadcaster',
            arguments=[
                '0', '0', '0',          # translation x y z
                '0', '0', '0',          # rotation roll pitch yaw
                'world',                # parent frame
                'map'                   # child frame
            ]
        ),
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
            ],
            remappings=[
                ('/next_waypoint/geo_pose', '/goal_geopose'),
                ('/camera_origin/gps', '/blueboat/sensors/gps/gps/fix'),
                ('/heading/imu/data', '/blueboat/sensors/imu/imu/data'),
                ('/camera/input_image', '/blueboat/sensors/cameras/front_camera_sensor/image_raw'),
                #('/camera/image_id', ),
            ]
        ),
    ])
