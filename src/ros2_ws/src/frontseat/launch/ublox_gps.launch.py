from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gps_config_file = PathJoinSubstitution([
        FindPackageShare('ublox_gps'),
        'config/zed_f9p.yaml'
    ])

    ublox_gps_node = Node(package='ublox_gps', 
                                             executable='ublox_gps_node', 
                                             output='both',
                                             parameters=[gps_config_file],
                                             respawn=True)
    
    return LaunchDescription([
        ublox_gps_node, 
    ])