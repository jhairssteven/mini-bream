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

# shellcheck source=/workspace/docker/dds_env.sh
source /workspace/docker/dds_env.sh
configure_cyclonedds

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

ILOS_DIR="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat"
exec python3 "${ILOS_DIR}/run_ilos_experiment.py" "$@"
