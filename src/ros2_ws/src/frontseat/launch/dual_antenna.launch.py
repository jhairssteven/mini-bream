from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
import launch
import os

def generate_launch_description():

    config_directory = os.path.join(
        get_package_share_directory('ublox_gps'),
        'config')
    params = os.path.join(config_directory, 'c94_m8p_rover.yaml')

    ublox_gps1_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[
            params,      
            {
                'rate': 19.0,
                'device': '/dev/ttyACM0'
            }
            
        ],
        remappings=[
            ("/ublox_gps_node/fix", "/fix1")
        ]
    )
    ublox_gps2_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[
            params,
            {
                'rate': 19.0,
                'device': '/dev/ttyACM1'
            }
        ],
        remappings=[
            ("/ublox_gps_node/fix", "/fix2")
        ]
    )
    
    dual_antenna = Node(
        package='frontseat',
        executable='dual_antenna',
        name='dual_antenna',
        output='screen',

        )

    return LaunchDescription([
        ublox_gps1_node,
        ublox_gps2_node,
        dual_antenna,

        # Event handler to shut down the whole launch file when the gps node dies
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ublox_gps1_node,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=ublox_gps2_node,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
        ]
    )
