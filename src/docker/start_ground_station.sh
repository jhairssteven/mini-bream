#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

# Build frontseat for BlueBoat URDF/meshes (mounted workspace).
if [[ -d /workspace/ros2_ws/src/frontseat ]]; then
  cd /workspace/ros2_ws
  colcon build --packages-select frontseat --symlink-install
  source install/setup.bash
fi

# Local robot_description for RViz RobotModel (TF still comes from Jetson over DDS).
if ! pgrep -f '[r]obot_state_publisher' >/dev/null 2>&1; then
  ros2 launch frontseat robot_description.launch.py &
  sleep 2
fi

RVIZ_CONFIG="${RVIZ_CONFIG:-/opt/ground_station/ground_station.rviz}"
RVIZ_ARGS=()
if [[ "${USE_SIM_TIME:-0}" == "1" ]]; then
  RVIZ_ARGS+=(--ros-args -p use_sim_time:=true)
fi
exec rviz2 -d "${RVIZ_CONFIG}" "${RVIZ_ARGS[@]}"
