from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
import launch
import os

def get_gps_node_launcher(gps_params, publish_rate=19.0, device='/dev/ttyACM0', output_topic='/fix1'):
    return Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='screen',
        parameters=[
            gps_params,      
            {
                'rate': publish_rate,
                'device': device
            }
            
        ],
        remappings=[
            ("/ublox_gps_node/fix", output_topic)
        ]
    )

def generate_launch_description():

    #config_directory = os.path.join(
    #    get_package_share_directory('ublox_gps'),
    #    'config')
    #gps_params = os.path.join(config_directory, 'c94_m8p_rover.yaml')

    #gps1_node = get_gps_node_launcher(gps_params, publish_rate=19.0, device='/dev/ttyACM0', output_topic='/fix1')
    #gps2_node = get_gps_node_launcher(gps_params, publish_rate=19.0, device='/dev/ttyACM1', output_topic='/fix2')
    
    dual_antenna_node = Node(
        package='frontseat',
        executable='dual_antenna',
        name='dual_antenna',
        output='screen',
        remappings=[
            ('/baseline/heading', '/wamv/sensors/imu/imu/data'),
            ('/dA/gps/center/fix', '/wamv/sensors/gps/gps/fix')
        ]
    )

    return LaunchDescription([
        #gps1_node,
        #gps2_node,
        dual_antenna_node,

        # Event handler to shut down the whole launch file when either gps node dies
        #RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=gps1_node,
        #        on_exit=[EmitEvent(
        #            event=Shutdown())],
        #    )),
        #RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=gps2_node,
        #        on_exit=[EmitEvent(
        #            event=Shutdown())],
        #    )),
        ]
    )
