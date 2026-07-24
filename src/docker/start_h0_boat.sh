#!/usr/bin/env bash
# Run the H0 boat experiment stack (Jetson or any ROS host with MPC deps).
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

H0_DIR="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat"
exec python3 "${H0_DIR}/run_h0_experiment.py" "$@"
