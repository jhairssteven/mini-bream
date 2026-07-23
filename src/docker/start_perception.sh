#!/bin/bash
# Build perception packages and start LiDAR + ZED 2i (Jetson).
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/zed_ws/install/setup.bash

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
      echo "[perception] warning: airy network setup failed (need CAP_NET_ADMIN / privileged)"
  fi
fi

# Offline ZED calibration: require SN conf in mounted settings dir.
ZED_SETTINGS="/usr/local/zed/settings"
if ! compgen -G "${ZED_SETTINGS}/SN*.conf" > /dev/null; then
  echo "[perception] warning: no ZED calibration in ${ZED_SETTINGS} (offline start will fail)" >&2
fi

start_launch() {
  local session="$1"
  local launch_file="$2"

  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[perception] tmux session '$session' already exists — skipping"
    return
  fi

  tmux new-session -d -s "$session" \
    "bash -lc 'source /opt/ros/humble/setup.bash && \
               source /opt/zed_ws/install/setup.bash && \
               source /workspace/ros2_ws/install/setup.bash && \
               exec ros2 launch frontseat ${launch_file}'"
  echo "[perception] started tmux session '$session' → ${launch_file}"
}

start_launch static_tf static_tf.launch.py
start_launch robot_description robot_description.launch.py
start_launch zed2i zed2i_camera.launch.py
start_launch airy_lidar airy_lidar.launch.py

echo "[perception] sessions: $(tmux ls 2>/dev/null || echo none)"

if [[ -t 0 ]]; then
  exec bash
else
  exec sleep infinity
fi
