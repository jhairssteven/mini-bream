#!/bin/bash
# Gazebo resource paths for blueboat_sim (meshes, worlds, spawn staging).
# The share/ root is required so package://blueboat_sim/... URIs resolve at runtime.
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$COLCON_CURRENT_PREFIX/share"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$COLCON_CURRENT_PREFIX/share/blueboat_sim/worlds"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$COLCON_CURRENT_PREFIX/share/blueboat_sim/models"
