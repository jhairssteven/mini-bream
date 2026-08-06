"""Launch dual GPS + production moving-base RTK for the nav stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node
import launch


def get_gps_node_launcher(gps_params, node_name, fix_topic, rover=False):
    remappings = [
        (f'/{node_name}/fix', fix_topic),
        ('fix', fix_topic),
    ]
    if rover:
        remappings.extend([
            (f'/{node_name}/navrelposned', '/navrelposned'),
            ('navrelposned', '/navrelposned'),
            (f'/{node_name}/navheading', '/navheading'),
            ('navheading', '/navheading'),
        ])

    return Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name=node_name,
        output='screen',
        parameters=[gps_params],
        remappings=remappings,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('frontseat')
    config_dir = os.path.join(pkg_share, 'config')
    ublox_config_dir = os.path.join(config_dir, 'ublox_gps')
    tf_config = os.path.join(config_dir, 'tf', 'blueboat_extrinsics.yaml')

    gps_rover = get_gps_node_launcher(
        os.path.join(ublox_config_dir, 'rover.yaml'),
        node_name='ublox_gps_rover',
        fix_topic='/fix/rover',
        rover=True,
    )
    gps_base = get_gps_node_launcher(
        os.path.join(ublox_config_dir, 'base.yaml'),
        node_name='ublox_gps_base',
        fix_topic='/fix/base',
        rover=False,
    )

    static_tf = Node(
        package='frontseat',
        executable='static_tf_broadcaster',
        name='static_tf_broadcaster',
        output='screen',
        parameters=[{'config_path': tf_config}],
    )

    filtering_config = os.path.join(config_dir, 'heading', 'filtering.yaml')

    rel_pos_heading = Node(
        package='frontseat',
        executable='rel_pos_heading',
        name='rel_pos_heading',
        output='screen',
        parameters=[filtering_config],
        remappings=[
            ('/fix/center', '/wamv/sensors/gps/gps/fix'),
            ('/baseline/heading', '/wamv/sensors/imu/imu/data'),
            ('/baseline/heading/deg', '/heading/deg'),
        ],
    )

    heading_ekf = Node(
        package='frontseat',
        executable='heading_ekf',
        name='heading_ekf',
        output='screen',
        parameters=[filtering_config],
        remappings=[
            ('/baseline/heading', '/wamv/sensors/imu/imu/data'),
            ('/baseline/heading/deg', '/heading/deg'),
        ],
    )

    gps_map_odom = Node(
        package='frontseat',
        executable='gps_map_odom',
        name='gps_map_odom',
        output='screen',
        parameters=[{
            'gps_topic': '/wamv/sensors/gps/gps/fix',
            'heading_topic': '/baseline/heading/raw',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_topic': '/odom',
            'publish_tf': True,
            'min_fix_status': 0,
            'flatten_z': True,
            'recent_path_max_points': 150,
            'recent_path_topic': '/gps/center/recent_path',
        }],
    )

    return LaunchDescription([
        gps_rover,
        gps_base,
        static_tf,
        rel_pos_heading,
        heading_ekf,
        gps_map_odom,
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=gps_rover,
                on_exit=[EmitEvent(event=Shutdown())],
            ),
        ),
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=gps_base,
                on_exit=[EmitEvent(event=Shutdown())],
            ),
        ),
    ])
