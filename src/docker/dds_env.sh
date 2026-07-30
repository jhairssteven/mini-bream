#!/usr/bin/env bash
# Cyclone DDS profile for autonomy / experiment containers.
# Field Jetson: pin to robot LAN. Dev laptop: leave unset (all interfaces).
configure_cyclonedds() {
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

  # Compose may pass CYCLONEDDS_URI="" — treat as unset so auto-detection runs.
  if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
    unset CYCLONEDDS_URI
  fi

  if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
    return 0
  fi

  if [[ -n "${AUTONOMY_CYCLONEDDS_URI:-}" ]]; then
    export CYCLONEDDS_URI="${AUTONOMY_CYCLONEDDS_URI}"
    return 0
  fi

  if [[ -f /etc/cyclonedds.jetson.xml ]] && ip link show eth0 &>/dev/null; then
    local jetson_ip=""
    jetson_ip="$(ip -4 addr show eth0 2>/dev/null | awk '/inet / {print $2}' | cut -d/ -f1 | head -1)"
    if [[ "${jetson_ip}" == "192.168.0.102" ]]; then
      export CYCLONEDDS_URI=file:///etc/cyclonedds.jetson.xml
      return 0
    fi
  fi

  if [[ -f /etc/cyclonedds.xml ]] && ip link show eth0 &>/dev/null; then
    export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
    return 0
  fi

  unset CYCLONEDDS_URI
}
