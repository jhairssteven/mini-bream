#!/usr/bin/env bash
# Pi-side bridge: /molo_boat/thrust_* (from Jetson autonomy) → /pwm/*_thrust_cmd
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file:///etc/cyclonedds.xml}"

H0_DIR="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat"
exec python3 "${H0_DIR}/thrust_bridge.py" \
  --mode pwm_topics \
  --input-left /molo_boat/thrust_left \
  --input-right /molo_boat/thrust_right \
  --pwm-left /pwm/left_thrust_cmd \
  --pwm-right /pwm/right_thrust_cmd
