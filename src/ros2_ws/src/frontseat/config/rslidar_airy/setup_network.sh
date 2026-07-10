#!/usr/bin/env bash
# Configure the host Ethernet interface for a RoboSense RS-LiDAR-AIRY.
# Default factory addressing: LiDAR 192.168.1.200, host 192.168.1.102/24.

set -euo pipefail

IFACE="${LIDAR_IFACE:-enp4s0}"
HOST_IP="${LIDAR_HOST_IP:-192.168.1.102}"
NETMASK="${LIDAR_NETMASK:-24}"

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "Interface ${IFACE} not found. Set LIDAR_IFACE to your Ethernet NIC." >&2
  exit 1
fi

ip link set "${IFACE}" up
if ip addr show "${IFACE}" | grep -q "inet ${HOST_IP}/${NETMASK}"; then
  echo "${IFACE} already has ${HOST_IP}/${NETMASK}"
else
  ip addr add "${HOST_IP}/${NETMASK}" dev "${IFACE}" 2>/dev/null || true
  echo "Assigned ${HOST_IP}/${NETMASK} to ${IFACE}"
fi

echo "Network ready. Ping the LiDAR with: ping -c 3 192.168.1.200"
