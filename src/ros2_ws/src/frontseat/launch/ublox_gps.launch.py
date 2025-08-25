import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the other launch file
    ublox_launch_file = os.path.join(
        get_package_share_directory('ublox_gps'),
        'launch',
        'ublox_gps_node-launch.py'
    )
    
    imu_estimation = Node(
        package='frontseat',
        executable='imu_estimation',
        name='imu_estimation',
        output='screen',
        remappings=[
            ('/wamv/sensors/gps/gps/fix', '/ublox_gps_node/fix')
        ]
        )

    return LaunchDescription([
        imu_estimation,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ublox_launch_file)
        )]
    )
