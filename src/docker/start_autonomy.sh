#!/usr/bin/env bash
# Keep the autonomy container alive; experiments are launched via docker exec.
set -eo pipefail

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

echo "[autonomy] ready — launch experiments with:"
echo "  docker exec mini_bream_autonomy /workspace/docker/start_h0_boat.sh --platform sim"
echo "  docker exec mini_bream_autonomy /workspace/docker/start_ilos_boat.sh --platform sim"
echo "[autonomy] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

exec tail -f /dev/null
