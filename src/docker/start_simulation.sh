#!/usr/bin/env bash
# Build VRX plugins + BlueBoat sim package and launch Gazebo Garden simulation.
set -eo pipefail

export GZ_VERSION="${GZ_VERSION:-garden}"

source /opt/ros/humble/setup.bash
# Garden-linked ros_gz (apt packages use ignition-transport11 and cannot bridge gz-sim7).
if [[ -f /opt/ros_gz_ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros_gz_ws/install/setup.bash
fi

source_ros2_pkg() {
  local pkg="$1"
  local setup
  for setup in \
    "install/${pkg}/share/${pkg}/local_setup.bash" \
    "install/${pkg}/share/${pkg}/package.bash"; do
    if [[ -f "${setup}" ]]; then
      # shellcheck disable=SC1090
      source "${setup}"
      return 0
    fi
  done
  echo "warning: could not source ros2 package ${pkg}" >&2
  return 1
}

# --- VRX workspace (Gazebo Garden plugins; coast_waves and physics) ---
cd /workspace/vrx_ws
colcon build \
  --packages-up-to vrx_gz vrx_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# --- ROS 2 workspace (BlueBoat sim + HAL) ---
cd /workspace/ros2_ws
colcon build --symlink-install \
  --packages-select blueboat_sim boat_hal \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

for pkg in blueboat_sim boat_hal; do
  source_ros2_pkg "${pkg}"
done

VRX_GZ_SHARE="$(ros2 pkg prefix vrx_gz)/share"
BB_PREFIX="$(ros2 pkg prefix blueboat_sim)"
BB_SIM_SHARE="${BB_PREFIX}/share/blueboat_sim"
export GZ_SIM_RESOURCE_PATH="${BB_PREFIX}/share:${BB_SIM_SHARE}/worlds:${BB_SIM_SHARE}/models:${VRX_GZ_SHARE}/models:${VRX_GZ_SHARE}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

WORLD="${SIM_WORLD:-open_water_harner}"
HEADLESS="${SIM_HEADLESS:-False}"

exec ros2 launch blueboat_sim open_water.launch.py \
  world:="${WORLD}" \
  headless:="${HEADLESS}"
