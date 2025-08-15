from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction

def generate_launch_description():

    joystick_nodes = GroupAction([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        Node(
            package='frontseat',
            executable='joystick',
            name='joystick',
            output='screen'
            )
    ])

    
    thrust_source_selector = Node(
        package='frontseat',
        executable='thrust_source_selector',
        name='thrust_source_selector',
        output='screen'
        )
    
    motor_controller = Node(
        package='frontseat',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
        )
    
    return LaunchDescription([
        joystick_nodes,
        motor_controller,
        thrust_source_selector
    ])
