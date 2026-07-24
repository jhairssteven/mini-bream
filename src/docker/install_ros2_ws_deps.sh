#!/usr/bin/env bash
# Shared ROS 2 workspace dependencies for Pi / Jetson / ground-station images.
# Installs apt packages so colcon can build the mounted workspace and ros2 bag
# can deserialize common robot topics (ublox, sensors, tf, etc.).
#
# ublox: use apt ros-humble-ublox-* (do not colcon-build the ublox submodule;
# source build needs libasio-dev and is slower to maintain).
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  pkg-config \
  git \
  python3-pip \
  python3-numpy \
  python3-colcon-common-extensions \
  python3-rosdep \
  libasio-dev \
  libpcap-dev \
  libyaml-cpp-dev \
  nlohmann-json3-dev \
  ros-humble-ros-base \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ros2bag \
  ros-humble-rosbag2-storage-mcap \
  ros-humble-joy \
  ros-humble-tf-transformations \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-geographic-msgs \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-nmea-msgs \
  ros-humble-angles \
  ros-humble-ublox-msgs \
  ros-humble-ublox-gps \
  ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport \
  ros-humble-compressed-depth-image-transport \
  ros-humble-theora-image-transport \
  ros-humble-diagnostic-updater \
  ros-humble-xacro \
  gcc \
  g++ \
  git

rm -rf /var/lib/apt/lists/*

# Python deps used by frontseat GPS nodes and molo MPC follower (see install_mpc_deps.sh).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=install_mpc_deps.sh
source "${SCRIPT_DIR}/install_mpc_deps.sh"
install_mpc_python_deps
echo "ros2_ws + mpc deps ok"
