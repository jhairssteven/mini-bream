#!/usr/bin/env bash
# Build the mounted ROS 2 workspace inside Docker (or on host).
# Skips the ublox git submodule — images install ros-humble-ublox-* via apt.
set -euo pipefail

cd /workspace/ros2_ws

source /opt/ros/humble/setup.bash
if [[ -f /opt/zed_ws/install/setup.bash ]]; then
  source /opt/zed_ws/install/setup.bash
fi

colcon build --symlink-install \
  --packages-ignore ublox_gps ublox_msgs ublox_serialization \
  "$@"
