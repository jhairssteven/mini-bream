#!/usr/bin/env bash
# Source ROS 2 Humble, the workspace overlay, and CycloneDDS if available.
# Intended to be sourced from other h0_boat scripts (not executed).
#
# shellcheck disable=SC1091

H0_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
H0_ROS2_WS="$(cd "${H0_DIR}/../../.." && pwd)"       # .../src/ros2_ws
H0_SRC_ROOT="$(cd "${H0_ROS2_WS}/.." && pwd)"        # .../src

_h0_source_ros() {
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/opt/ros/humble/setup.bash
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
    source /workspace/ros2_ws/install/setup.bash
  elif [[ -f "${H0_ROS2_WS}/install/setup.bash" ]]; then
    source "${H0_ROS2_WS}/install/setup.bash"
  fi
  if [[ -f /workspace/vrx_ws/install/setup.bash ]]; then
    source /workspace/vrx_ws/install/setup.bash
  fi
  set -u
}

_h0_configure_dds() {
  local dds_env=""
  if [[ -f /workspace/docker/dds_env.sh ]]; then
    dds_env="/workspace/docker/dds_env.sh"
  elif [[ -f "${H0_SRC_ROOT}/docker/dds_env.sh" ]]; then
    dds_env="${H0_SRC_ROOT}/docker/dds_env.sh"
  fi
  if [[ -n "${dds_env}" ]]; then
    # shellcheck source=../../../../docker/dds_env.sh
    source "${dds_env}"
    configure_cyclonedds
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
}

h0_require_ros() {
  _h0_source_ros
  _h0_configure_dds
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[h0-boat] ERROR: ros2 not on PATH. Source Humble (and the workspace) first." >&2
    return 1
  fi
}
