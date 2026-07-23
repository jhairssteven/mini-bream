#!/usr/bin/env bash
# Record a rosbag with all overlays sourced (required for ZED + workspace msg types).
# Usage inside perception / frontseat / ground containers:
#   record_rosbag.sh bag_suffix:=zed_test
#   record_rosbag.sh bag_storage:=sqlite3 bag_suffix:=pi_motors
set -euo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /opt/zed_ws/install/setup.bash ]]; then
  source /opt/zed_ws/install/setup.bash
fi
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

exec ros2 launch frontseat rosbag.launch.py "$@"
