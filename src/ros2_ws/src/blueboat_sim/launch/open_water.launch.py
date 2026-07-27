"""Launch BlueBoat in the VRX open-water world (VRX hydrodynamics + HAL topic layout)."""

from launch import LaunchDescription

from blueboat_sim.vrx.sim_launch import common_launch_arguments


def generate_launch_description():
    return LaunchDescription(common_launch_arguments(
        default_world='open_water_harner',
        world_description='Open-water world (no shore/dock obstacles)',
    ))
