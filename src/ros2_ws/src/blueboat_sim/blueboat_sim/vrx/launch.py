import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from blueboat_sim.vrx import bridges as vrx_bridges


def simulation(world_name, headless=False, paused=False, extra_gz_args=''):
    gz_args = ['-v 4']
    if not paused:
        gz_args.append('-r')
    if headless:
        gz_args.append('-s')
    if extra_gz_args:
        gz_args.append(extra_gz_args)

    world_base = os.path.basename(world_name)
    world_path = os.path.join(
        get_package_share_directory('blueboat_sim'),
        'worlds',
        f'{world_base}.sdf',
    )
    if not os.path.isfile(world_path):
        raise RuntimeError(f'World file not found: {world_path}')
    gz_args.append(world_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments={
            'gz_args': ' '.join(gz_args),
            'gz_version': '7',
        }.items())

    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', os.path.join(
            get_package_share_directory('vrx_ros'), 'launch', 'monitor_sim.py')],
        name='monitor_sim',
        output='screen',
    )
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim_proc,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulation ended'))],
        )
    )
    return [gz_sim, monitor_sim_proc, sim_exit_event_handler]


def competition_bridges(world_name, competition_mode=False):
    del world_name, competition_mode
    bridges = [
        vrx_bridges.clock(),
        vrx_bridges.task_info(),
        vrx_bridges.usv_wind_speed(),
        vrx_bridges.usv_wind_direction(),
    ]
    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    )]


def spawn(sim_mode, world_name, models, robot=None):
    if not isinstance(models, list):
        models = [models]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    launch_processes = []
    for model in models:
        if robot and model.model_name != robot:
            continue

        model.generate()

        gz_spawn_entity = None
        if sim_mode in ('full', 'sim'):
            gz_spawn_entity = ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', f'/world/{world_name}/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '60000',
                    '--req', model.spawn_service_request(world_name),
                ],
                output='screen',
            )
            launch_processes.append(TimerAction(period=15.0, actions=[gz_spawn_entity]))

        if sim_mode in ('full', 'bridge'):
            bridges, nodes, custom_launches = model.bridges(world_name)
            payload_bridges, payload_nodes, payload_launches = model.payload_bridges(world_name)
            bridges.extend(payload_bridges)
            nodes.extend(payload_nodes)

            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                output='screen',
                arguments=[bridge.argument() for bridge in bridges],
                remappings=[bridge.remapping() for bridge in bridges],
            ))
            nodes.append(Node(
                package='vrx_ros',
                executable='pose_tf_broadcaster',
                output='screen',
            ))

            model_tmp_dir = os.path.join(
                get_package_share_directory('blueboat_sim'),
                'models', 'spawn_tmp')
            urdf_file = os.path.join(model_tmp_dir, 'model.urdf')
            with open(urdf_file, 'r') as infp:
                robot_desc = infp.read()
            frame_prefix = f'{model.model_name}/'
            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_prefix': frame_prefix,
                    'robot_description': robot_desc,
                }],
                remappings=[('/joint_states', f'/{model.model_name}/joint_states')],
            ))

            group_action = GroupAction([
                PushRosNamespace(model.model_name),
                *nodes,
            ])

            if sim_mode == 'full':
                launch_processes.append(RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=gz_spawn_entity,
                        on_exit=[group_action],
                    )
                ))
            else:
                launch_processes.append(group_action)

            launch_processes.extend(payload_launches)
            launch_processes.extend(custom_launches)

    return launch_processes
