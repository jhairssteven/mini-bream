"""Launch BlueBoat in the lidar obstacle course world (VRX hydrodynamics + HAL topic layout)."""

from launch import LaunchDescription

from blueboat_sim.vrx.sim_launch import common_launch_arguments


def generate_launch_description():
    return LaunchDescription(common_launch_arguments(
        default_world='lidar_obstacle_course_harner',
        world_description='Lidar obstacle course world',
    ))
