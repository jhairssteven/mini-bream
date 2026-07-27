#!/usr/bin/env bash
# VRX + Gazebo Garden dependencies for BlueBoat simulation (ROS 2 Humble).
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  lsb-release \
  curl \
  gnupg \
  tmux \
  nano \
  ruby \
  libeigen3-dev \
  python3-colcon-common-extensions

# Gazebo Garden (VRX requires gz-sim7)
if ! apt-cache show gz-garden &>/dev/null; then
  curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
  apt-get update
fi

apt-get install -y --no-install-recommends \
  gz-garden \
  libgz-sim7-dev \
  libsdformat13-dev \
  libgz-common5-dev \
  libgz-math7-dev \
  libgz-msgs9-dev \
  libgz-transport12-dev \
  libgz-plugin2-dev \
  libgz-rendering7-dev \
  libgz-sensors7-dev \
  libgz-utils2-dev \
  python3-sdformat13 \
  ros-humble-topic-tools \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  git

# apt ros-humble-ros-gz-* links ignition-transport11 (Fortress) and cannot
# bridge to gz-sim7 (Garden). Build ros_gz from source with GZ_VERSION=garden.
export GZ_VERSION=garden
ROS_GZ_WS=/opt/ros_gz_ws
mkdir -p "${ROS_GZ_WS}/src"
if [[ ! -d "${ROS_GZ_WS}/src/ros_gz" ]]; then
  git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git "${ROS_GZ_WS}/src/ros_gz"
fi
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u
cd "${ROS_GZ_WS}"
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble || true
colcon build \
  --packages-select ros_gz_interfaces ros_gz_bridge ros_gz_sim ros_gz_image \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source ${ROS_GZ_WS}/install/setup.bash" >> /etc/bash.bashrc

rm -rf /var/lib/apt/lists/*
echo "simulation deps ok (Gazebo Garden + ros_gz built for garden)"
