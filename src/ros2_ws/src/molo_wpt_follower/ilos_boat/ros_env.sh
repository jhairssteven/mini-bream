#!/usr/bin/env bash
# Source ROS 2 Humble, the workspace overlay, and CycloneDDS if available.
# Intended to be sourced from other ilos_boat scripts (not executed).
#
# shellcheck disable=SC1091

ILOS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ILOS_ROS2_WS="$(cd "${ILOS_DIR}/../../.." && pwd)"       # .../src/ros2_ws
ILOS_SRC_ROOT="$(cd "${ILOS_ROS2_WS}/.." && pwd)"        # .../src

_ilos_source_ros() {
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/opt/ros/humble/setup.bash
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
    source /workspace/ros2_ws/install/setup.bash
  elif [[ -f "${ILOS_ROS2_WS}/install/setup.bash" ]]; then
    source "${ILOS_ROS2_WS}/install/setup.bash"
  fi
  if [[ -f /workspace/vrx_ws/install/setup.bash ]]; then
    source /workspace/vrx_ws/install/setup.bash
  fi
  set -u
}

_ilos_configure_dds() {
  local dds_env=""
  if [[ -f /workspace/docker/dds_env.sh ]]; then
    dds_env="/workspace/docker/dds_env.sh"
  elif [[ -f "${ILOS_SRC_ROOT}/docker/dds_env.sh" ]]; then
    dds_env="${ILOS_SRC_ROOT}/docker/dds_env.sh"
  fi
  if [[ -n "${dds_env}" ]]; then
    # shellcheck source=../../../../docker/dds_env.sh
    source "${dds_env}"
    configure_cyclonedds
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
}

ilos_require_ros() {
  _ilos_source_ros
  _ilos_configure_dds
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ilos-boat] ERROR: ros2 not on PATH. Source Humble (and the workspace) first." >&2
    return 1
  fi
}
