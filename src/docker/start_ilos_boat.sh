#!/usr/bin/env bash
# Run the ILOS boat experiment stack (Jetson or any ROS host).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  source /workspace/ros2_ws/install/setup.bash
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -f /etc/cyclonedds.jetson.xml ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds.jetson.xml}"
elif [[ -f /etc/cyclonedds.xml ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds.xml}"
fi

ILOS_DIR="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat"
exec python3 "${ILOS_DIR}/run_ilos_experiment.py" "$@"
