from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_controller_node = Node(
        package='joystick_control',
        executable='joy_controller_node',
        name='joy_controller_node',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([joy_controller_node,joy_node])