#!/usr/bin/env bash
# Build autonomy packages and keep the container alive; launch experiments via docker exec.
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

cd /workspace/ros2_ws
colcon build \
  --packages-select blueboat_nav2 mission_planner molo_wpt_follower \
  --symlink-install

grep -qxF 'source /workspace/ros2_ws/install/setup.bash' ~/.bashrc || \
  echo 'source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc

set +u
source /workspace/ros2_ws/install/setup.bash
set -u

# shellcheck source=/workspace/docker/dds_env.sh
source /workspace/docker/dds_env.sh
configure_cyclonedds

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[autonomy] workspace built (blueboat_nav2, mission_planner, molo_wpt_follower)"
echo "[autonomy] ready — launch experiments with:"
echo "  docker exec -it mini_bream_autonomy bash"
echo "  ros2 launch mission_planner molo_autonomy.launch.py platform:=blueboat_sim controller:=ilos"
echo "  ros2 launch mission_planner molo_autonomy.launch.py platform:=blueboat controller:=ilos use_sim_time:=false"
echo "  rviz2 -d /workspace/docker/config/molo_autonomy.rviz --ros-args -p use_sim_time:=true"
echo "[autonomy] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"

exec tail -f /dev/null
