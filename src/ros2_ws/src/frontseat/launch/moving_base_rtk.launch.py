from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
import launch
import os

def get_gps_node_launcher(gps_params, fix_topic='/fix1'):
    return Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='screen',
        parameters=[
            gps_params,            
        ],
        remappings=[
            ("/ublox_gps_node/fix", fix_topic),  # source-built driver (>=2.3.0, node-private topics)
            ("fix", fix_topic)                   # apt driver 2.3.0 (publishes on plain /fix)
        ]
    )

def generate_launch_description():

    config_directory = os.path.join(get_package_share_directory('frontseat'), 'config', 'ublox_gps')
    base_gps_params = os.path.join(config_directory, 'base.yaml')
    rover_gps_params = os.path.join(config_directory, 'rover.yaml')

    gps_base = get_gps_node_launcher(base_gps_params, fix_topic='/fix/base')
    gps_rover = get_gps_node_launcher(rover_gps_params, fix_topic='/fix/rover')
    
    moving_base_rtk_node = Node(
        package='frontseat',
        executable='moving_base_rtk',
        name='moving_base_rtk_node',
        output='screen',
        remappings=[
            ('/baseline/heading', '/wamv/sensors/imu/imu/data'),
            ('/fix/center/avg', '/wamv/sensors/gps/gps/fix')
        ]
    )

    return LaunchDescription([
        gps_base,
        gps_rover,
        moving_base_rtk_node,

        # Event handler to shut down the whole launch file when either gps node dies
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=gps_base,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=gps_rover,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
        ]
    )
