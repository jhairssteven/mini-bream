import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the other launch file
    ublox_launch_file = os.path.join(
        get_package_share_directory('ublox_gps'),
        'launch',
        'ublox_gps_node-launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ublox_launch_file)
        )
    ])
