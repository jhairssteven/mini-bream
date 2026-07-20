#!/usr/bin/env bash
# Add a secondary IPv4 address so this host can receive RS-LiDAR-AIRY traffic.
#
# Factory defaults (unchanged):
#   LiDAR device IP : 192.168.1.200
#   Destination host: 192.168.1.102  (MSOP 6699 / DIFOP 7788 / IMU 6688)
#
# On a shared L2 LAN (LiDAR plugged into the same router as the Jetson/Pi),
# the host keeps its primary DHCP address (e.g. 192.168.0.102) and adds
# 192.168.1.102/24 as a secondary address on the same interface.

set -euo pipefail

IFACE="${LIDAR_IFACE:-eth0}"
HOST_IP="${LIDAR_HOST_IP:-192.168.1.102}"
NETMASK="${LIDAR_NETMASK:-24}"
LIDAR_IP="${LIDAR_IP:-192.168.1.200}"

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "Interface ${IFACE} not found. Set LIDAR_IFACE to the NIC on the LiDAR LAN." >&2
  exit 1
fi

ip link set "${IFACE}" up
if ip addr show "${IFACE}" | grep -q "inet ${HOST_IP}/${NETMASK}"; then
  echo "${IFACE} already has ${HOST_IP}/${NETMASK}"
else
  ip addr add "${HOST_IP}/${NETMASK}" dev "${IFACE}"
  echo "Assigned ${HOST_IP}/${NETMASK} to ${IFACE}"
fi

echo "Network ready. Verify with: ping -c 3 ${LIDAR_IP}"
echo "Optional UDP check: tcpdump -i ${IFACE} -c 5 -n udp port 6699"
