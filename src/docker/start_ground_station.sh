#!/usr/bin/env bash
set -eo pipefail

# Generate Cyclone DDS config for ground station (wifi interface or IP bind).
write_cyclonedds_config() {
  local out="/tmp/cyclonedds.ground.runtime.xml"
  local iface="${GROUND_IFACE:-}"
  local address="${GROUND_DDS_ADDRESS:-}"
  local iface_xml

  if [[ -n "${iface}" ]]; then
    iface_xml="<NetworkInterface name=\"${iface}\" multicast=\"true\"/>"
  elif [[ -n "${address}" ]]; then
    iface_xml="<NetworkInterface address=\"${address}\" multicast=\"true\"/>"
  else
    iface_xml="<NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\"/>"
  fi

  cat > "${out}" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        ${iface_xml}
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="192.168.0.100"/>
        <Peer address="192.168.0.102"/>
        <Peer address="192.168.0.105"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  export CYCLONEDDS_URI="file://${out}"
  echo "[ground_station] CYCLONEDDS_URI=${CYCLONEDDS_URI} (iface=${iface:-none}, address=${address:-auto})"
}

write_cyclonedds_config

source /opt/ros/humble/setup.bash

# Build frontseat for BlueBoat URDF/meshes (mounted workspace).
if [[ -d /workspace/ros2_ws/src/frontseat ]]; then
  cd /workspace/ros2_ws
  colcon build --packages-select frontseat --symlink-install
  source install/setup.bash
fi

# Local robot_description for RViz RobotModel (TF and odom come from Pi over DDS).
if ! pgrep -f '[r]obot_state_publisher' >/dev/null 2>&1; then
  ros2 launch frontseat robot_description.launch.py &
  sleep 2
fi

RVIZ_CONFIG="${RVIZ_CONFIG:-/workspace/docker/config/molo_autonomy.rviz}"
RVIZ_ARGS=()
if [[ "${USE_SIM_TIME:-0}" == "1" ]]; then
  RVIZ_ARGS+=(--ros-args -p use_sim_time:=true)
fi
exec rviz2 -d "${RVIZ_CONFIG}" "${RVIZ_ARGS[@]}"
