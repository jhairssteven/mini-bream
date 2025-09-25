from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

# Install dependency with sudo apt-get install ros-<ros2-distro>-usb-cam
front_camera_params = os.path.join(
    get_package_share_directory('frontseat'), 'config', 'cameras', 'front_camera_params.yaml'
    )

def generate_launch_description():
    web_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera_node',
        output='screen',
        parameters=[front_camera_params],

    )
    return LaunchDescription([
        web_cam_node
        ])
