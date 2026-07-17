from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
import launch
import os

def get_gps_node_launcher(gps_params, publish_rate=19.0, device='/dev/ttyACM0', ns='gps', output_topic='/fix1'):
    return Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='screen',
        parameters=[
            gps_params,      
            #{
            #    'rate': publish_rate,
            #    'device': device
            #}
            
        ],
        remappings=[
            ("/ublox_gps_node/fix", output_topic),  # source-built driver (>=2.3.0, node-private topics)
            ("fix", output_topic)                   # apt driver 2.3.0 (publishes on plain /fix)
        ]
    )

def generate_launch_description():

    config_directory = os.path.join(
        get_package_share_directory('frontseat'),
        'config', 'ublox_gps')
    base_gps_params = os.path.join(config_directory, 'base.yaml')
    rover_gps_params = os.path.join(config_directory, 'rover.yaml')

    gps_base = get_gps_node_launcher(base_gps_params, publish_rate=19.0, device='/dev/ttyACM0', ns='base_gps', output_topic='/fix/base')
    gps_rover = get_gps_node_launcher(rover_gps_params, publish_rate=19.0, device='/dev/ttyACM1', ns='rover_gps', output_topic='/fix/rover')
    
    dual_antenna_node = Node(
        package='frontseat',
        executable='dual_antenna',
        name='dual_antenna',
        output='screen',
        remappings=[
            ('/fix1', '/fix/rover'),
            ('/fix2', '/fix/base'),
            ('/baseline/heading', '/wamv/sensors/imu/imu/data'),
            ('/dA/gps/center/fix', '/wamv/sensors/gps/gps/fix')
        ]
    )

    return LaunchDescription([
        #gps_base,
        #gps_rover,
        dual_antenna_node,
 #       imu_estimation,

        # Event handler to shut down the whole launch file when either gps node dies
        #RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=gps_base,
        #        on_exit=[EmitEvent(
        #            event=Shutdown())],
        #    )),
        #RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=gps_rover,
        #        on_exit=[EmitEvent(
        #            event=Shutdown())],
        #    )),
        ]
    )
