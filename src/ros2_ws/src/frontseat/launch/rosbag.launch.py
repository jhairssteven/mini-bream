from launch import LaunchDescription
from launch.actions import ExecuteProcess
from datetime import datetime
import os

def generate_launch_description():
    topics_to_record = [
        '/wamv/thrusters/left/thrust/actual',
        '/wamv/thrusters/right/thrust/actual',
    ]
    
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    day = datetime.now().strftime('%Y-%m-%d')
    bag_name = f"/frontseat/logs/{day}/rosbag_{timestamp}"

    bag_folder = os.path.dirname(bag_name)
    os.makedirs(bag_folder, exist_ok=True)
    
    ros2_bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name] + topics_to_record,
        output='screen'
    )

    return LaunchDescription([
        ros2_bag_record
    ])
