#!/bin/bash
# Build frontseat and start motor/RTK launch files in detached tmux sessions.
set -eo pipefail

source /opt/ros/humble/setup.bash
colcon build --packages-select frontseat --symlink-install

grep -qxF 'source /workspace/ros2_ws/install/setup.bash' ~/.bashrc || \
  echo 'source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc

source /workspace/ros2_ws/install/setup.bash

start_launch() {
  local session="$1"
  local launch_file="$2"

  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[frontseat] tmux session '$session' already exists — skipping"
    return
  fi

  tmux new-session -d -s "$session" \
    "bash -lc 'source /opt/ros/humble/setup.bash && \
               source /workspace/ros2_ws/install/setup.bash && \
               exec ros2 launch frontseat ${launch_file}'"
  echo "[frontseat] started tmux session '$session' → ${launch_file}"
}

start_launch motor_control motor_control.launch.py
start_launch moving_base_rtk moving_base_rtk.launch.py

echo "[frontseat] sessions: $(tmux ls 2>/dev/null || echo none)"

if [[ -t 0 ]]; then
  exec bash
else
  exec sleep infinity
fi
