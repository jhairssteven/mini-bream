#!/usr/bin/env bash
# Start mock frontseat sensor publishers on the Pi (for Jetson-side h0_boat testing).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source /opt/ros/humble/setup.bash
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -f /etc/cyclonedds.xml ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds.xml}"
fi

H0_DIR="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat"
exec python3 "${H0_DIR}/mock_frontseat.py" "$@"
