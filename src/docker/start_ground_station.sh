#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
# Workspace optional on ground station (standard sensor_msgs only for RViz).
if [[ -f /workspace/ros2_ws/install/setup.bash ]] && [[ -f /workspace/ros2_ws/install/.colcon_install_layout ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

RVIZ_CONFIG="${RVIZ_CONFIG:-/opt/ground_station/ground_station.rviz}"
exec rviz2 -d "${RVIZ_CONFIG}"
