#!/bin/bash
# Build rslidar packages and start the Airy driver (Jetson perception stack).
set -eo pipefail

source /opt/ros/humble/setup.bash

colcon build \
  --packages-select rslidar_msg rslidar_sdk frontseat \
  --symlink-install \
  --cmake-args -DENABLE_IMU_DATA_PARSE=ON

grep -qxF 'source /workspace/ros2_ws/install/setup.bash' ~/.bashrc || \
  echo 'source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc

source /workspace/ros2_ws/install/setup.bash

# Secondary IP for factory LiDAR destination 192.168.1.102 (idempotent).
if [[ "${SETUP_AIRY_NETWORK:-1}" == "1" ]]; then
  NET_SCRIPT="/workspace/ros2_ws/src/frontseat/config/rslidar_airy/setup_network.sh"
  if [[ -x "${NET_SCRIPT}" ]]; then
    LIDAR_IFACE="${LIDAR_IFACE:-eth0}" bash "${NET_SCRIPT}" || \
      echo "[lidar] warning: airy network setup failed (need CAP_NET_ADMIN / privileged)"
  fi
fi

start_launch() {
  local session="$1"
  local launch_file="$2"

  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[lidar] tmux session '$session' already exists — skipping"
    return
  fi

  tmux new-session -d -s "$session" \
    "bash -lc 'source /opt/ros/humble/setup.bash && \
               source /workspace/ros2_ws/install/setup.bash && \
               exec ros2 launch frontseat ${launch_file}'"
  echo "[lidar] started tmux session '$session' → ${launch_file}"
}

start_launch airy_lidar airy_lidar.launch.py

echo "[lidar] sessions: $(tmux ls 2>/dev/null || echo none)"

if [[ -t 0 ]]; then
  exec bash
else
  exec sleep infinity
fi
