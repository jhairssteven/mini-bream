#!/usr/bin/env bash
# Measure ROS sensor bandwidth across Pi / Jetson / ground station and print a report.
#
# Run from any host that can SSH to the others (typically the Pi):
#   cd src/docker
#   ./bandwidth_report.sh
#   ./bandwidth_report.sh | tee reports/bandwidth_$(date +%Y%m%d_%H%M%S).txt
#
# Optional environment overrides:
#   PI_IP=192.168.0.101  JETSON_IP=192.168.0.102  GROUND_IP=192.168.0.103
#   PI_SSH_USER=pi  JETSON_SSH_USER=orin-nano  GROUND_SSH_USER=steven
#   GROUND_REPO=/path/on/ground/to/mini-bream/src/docker
#   MEASURE_SEC=25  IPERF_SEC=12  ROUTER_MODEL="TP-Link TL-WR841N"
#   GROUND_WIFI_IFACE=wlp5s0  PI_ETH_IFACE=eth0
#   SKIP_IPERF=1  SKIP_GROUND=1  SKIP_JETSON=1
#
# SSH auth: prefer keys. For password auth, set (not recommended for commits):
#   SSHPASS_PI=... SSHPASS_JETSON=... SSHPASS_GROUND=...

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_SRC="$(cd "${SCRIPT_DIR}/.." && pwd)"

# --- defaults (robot LAN) ---
PI_IP="${PI_IP:-192.168.0.101}"
JETSON_IP="${JETSON_IP:-192.168.0.102}"
GROUND_IP="${GROUND_IP:-192.168.0.103}"

PI_SSH_USER="${PI_SSH_USER:-pi}"
JETSON_SSH_USER="${JETSON_SSH_USER:-orin-nano}"
GROUND_SSH_USER="${GROUND_SSH_USER:-steven}"

PI_SSH="${PI_SSH_USER}@${PI_IP}"
JETSON_SSH="${JETSON_SSH_USER}@${JETSON_IP}"
GROUND_SSH="${GROUND_SSH_USER}@${GROUND_IP}"

GROUND_REPO="${GROUND_REPO:-${GROUND_REPO_PATH:-}}"
JETSON_REPO="${JETSON_REPO:-/home/orin-nano/mini-bream/src/docker}"

MEASURE_SEC="${MEASURE_SEC:-25}"
IPERF_SEC="${IPERF_SEC:-12}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROUTER_MODEL="${ROUTER_MODEL:-TP-Link TL-WR841N}"

GROUND_WIFI_IFACE="${GROUND_WIFI_IFACE:-wlp5s0}"
PI_ETH_IFACE="${PI_ETH_IFACE:-eth0}"

PERCEPTION_CONTAINER="${PERCEPTION_CONTAINER:-mini_bream_perception}"
FRONTSEAT_CONTAINER="${FRONTSEAT_CONTAINER:-mini_bream_frontseat}"
GROUND_IMAGE="${GROUND_IMAGE:-mini-bream:ground-station}"
GROUND_DDS_FILE="${GROUND_DDS_FILE:-cyclonedds.ground.xml}"

TOPICS=(
  /rslidar_points
  /rslidar_imu_data
  /zed/zed/rgb/color/rect/image
  /zed/zed/point_cloud/cloud_registered
)

HEAVY_TOPICS=(
  /rslidar_points
  /zed/zed/rgb/color/rect/image
  /zed/zed/point_cloud/cloud_registered
)

# --- helpers ---
ssh_with_pass() {
  local pass_var="$1"
  local target="$2"
  shift 2
  local pass="${!pass_var:-}"
  if [[ -n "${pass}" ]] && command -v sshpass >/dev/null 2>&1; then
    sshpass -p "${pass}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "${target}" "$@"
  else
    ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "${target}" "$@"
  fi
}

ssh_pi()    { ssh_with_pass SSHPASS_PI    "${PI_SSH}" "$@"; }
ssh_jetson(){ ssh_with_pass SSHPASS_JETSON "${JETSON_SSH}" "$@"; }
ssh_ground(){ ssh_with_pass SSHPASS_GROUND "${GROUND_SSH}" "$@"; }

ping_ok() {
  ping -c 1 -W 2 "$1" >/dev/null 2>&1
}

mbps_from_mbs() {
  awk -v mbs="$1" 'BEGIN { printf "%.1f", mbs * 8 }'
}

section() {
  echo ""
  echo "================================================================"
  echo " $1"
  echo "================================================================"
}

subsection() {
  echo ""
  echo "--- $1 ---"
}

# Parse last "X.XX MB/s" or "X.XX KB/s" line from ros2 topic bw output; convert to MB/s.
parse_ros_bw_mbs() {
  local line
  line="$(grep -E '[0-9]+\.[0-9]+ (MB/s|KB/s)' | tail -1 || true)"
  if [[ -z "${line}" ]]; then
    echo ""
    return 0
  fi
  if [[ "${line}" == *MB/s* ]]; then
    echo "${line}" | grep -oE '[0-9]+\.[0-9]+' | head -1
  else
    local kb
    kb="$(echo "${line}" | grep -oE '[0-9]+\.[0-9]+' | head -1)"
    awk -v kb="${kb}" 'BEGIN { printf "%.4f", kb / 1024 }'
  fi
}

jetson_ros() {
  local cmd="$1"
  ssh_jetson "docker exec ${PERCEPTION_CONTAINER} bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/zed_ws/install/setup.bash
    source /workspace/ros2_ws/install/setup.bash
    ${cmd}
  '"
}

pi_ros() {
  local cmd="$1"
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${FRONTSEAT_CONTAINER}"; then
    docker exec "${FRONTSEAT_CONTAINER}" bash -lc "
      source /opt/ros/humble/setup.bash
      source /workspace/ros2_ws/install/setup.bash
      ${cmd}
    "
  else
    ssh_pi "docker exec ${FRONTSEAT_CONTAINER} bash -lc '
      source /opt/ros/humble/setup.bash
      source /workspace/ros2_ws/install/setup.bash
      ${cmd}
    '"
  fi
}

ground_ros_bw() {
  local topic="$1"
  local repo_path="$2"
  local iface="$3"
  ssh_ground "bash -s" <<EOF
set -eo pipefail
REPO='${repo_path}'
IFACE='${iface}'
TOPIC='${topic}'
SEC='${MEASURE_SEC}'
RX1=\$(cat /sys/class/net/\${IFACE}/statistics/rx_bytes 2>/dev/null || echo 0)
OUT=\$(docker run --rm --network host \\
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.ground.xml \\
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \\
  -v "\${REPO}/${GROUND_DDS_FILE}:/etc/cyclonedds.ground.xml:ro" \\
  ${GROUND_IMAGE} bash -lc "source /opt/ros/humble/setup.bash; sleep 8; timeout \${SEC} ros2 topic bw \${TOPIC}" 2>&1 || true)
RX2=\$(cat /sys/class/net/\${IFACE}/statistics/rx_bytes 2>/dev/null || echo 0)
ROS_BW=\$(echo "\${OUT}" | grep -E '[0-9]+\\.[0-9]+ (MB/s|KB/s)' | tail -1 || true)
WIFI_MBS=\$(python3 -c "print(f'{(\${RX2}-\${RX1})/\${SEC}/1e6:.4f}')" 2>/dev/null || echo "")
echo "ROS_LINE:\${ROS_BW}"
echo "WIFI_MBS:\${WIFI_MBS}"
PUBS=\$(echo "\${OUT}" | grep -c 'Subscribed' || true)
echo "SUBSCRIBED:\${PUBS}"
EOF
}

detect_ground_repo() {
  if [[ -n "${GROUND_REPO}" ]]; then
    echo "${GROUND_REPO}"
    return
  fi
  ssh_ground 'for d in \
    "$HOME/mini-bream/src/docker" \
    "$HOME/Documents/phd_while_alienware/naslab/LINC project/source code/reflexAI_directory_structure/codebase/mini-bream/src/docker"; do
    [[ -f "$d/cyclonedds.ground.xml" ]] && echo "$d" && exit 0
  done; exit 1' 2>/dev/null || echo "${SCRIPT_DIR}"
}

# --- report header ---
REPORT_TIME="$(date -Iseconds)"
section "Mini-Bream bandwidth report — ${REPORT_TIME}"
echo "Router reference: ${ROUTER_MODEL}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  measure_window=${MEASURE_SEC}s  iperf=${IPERF_SEC}s"
echo "Hosts: Pi=${PI_IP}  Jetson=${JETSON_IP}  Ground=${GROUND_IP}"

# --- preflight ---
section "1. Preflight"

subsection "Reachability"
for entry in "Pi:${PI_IP}" "Jetson:${JETSON_IP}" "Ground:${GROUND_IP}"; do
  name="${entry%%:*}"
  ip="${entry#*:}"
  if ping_ok "${ip}"; then
    echo "  ${name} (${ip}): OK"
  else
    echo "  ${name} (${ip}): UNREACHABLE"
  fi
done

subsection "Docker containers"
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qE 'mini_bream_(frontseat|pwm|radio)'; then
  docker ps --format '  {{.Names}}  {{.Status}}' | grep mini_bream || true
else
  ssh_pi 'docker ps --format "  {{.Names}}  {{.Status}}"' 2>/dev/null | grep mini_bream || echo "  (Pi containers not checked)"
fi

if [[ "${SKIP_JETSON:-0}" != "1" ]]; then
  ssh_jetson "docker ps --format '  {{.Names}}  {{.Status}}'" 2>/dev/null | grep mini_bream || echo "  Jetson perception: not running"
fi

subsection "ROS_DOMAIN_ID"
if [[ "${SKIP_JETSON:-0}" != "1" ]]; then
  echo -n "  Jetson perception: "
  ssh_jetson "docker exec ${PERCEPTION_CONTAINER} printenv ROS_DOMAIN_ID" 2>/dev/null || echo "n/a"
fi
echo -n "  Pi frontseat: "
pi_ros 'echo $ROS_DOMAIN_ID' 2>/dev/null || echo "n/a"

# --- jetson source bandwidth ---
declare -A JETSON_MBS=()

if [[ "${SKIP_JETSON:-0}" != "1" ]]; then
  section "2. Jetson source (local publish rate)"
  echo "Container: ${PERCEPTION_CONTAINER} @ ${JETSON_IP}"

  for topic in "${TOPICS[@]}"; do
    subsection "${topic}"
    OUT="$(jetson_ros "timeout ${MEASURE_SEC} ros2 topic bw ${topic} 2>&1" || true)"
    if [[ -n "${OUT}" ]]; then
      echo "${OUT}" | grep -E 'mean:|MB/s|KB/s|from [0-9]+ messages' | tail -4 || echo "  (no data — is perception publishing?)"
      mbs="$(echo "${OUT}" | parse_ros_bw_mbs || true)"
      if [[ -n "${mbs}" ]]; then
        JETSON_MBS["${topic}"]="${mbs}"
        echo "  >> parsed: ${mbs} MB/s ($(mbps_from_mbs "${mbs}") Mbps)"
      fi
    else
      echo "  (measurement failed)"
    fi
  done
fi

# --- ground station receive ---
declare -A GROUND_WIFI_MBS=()
declare -A GROUND_ROS_MBS=()

if [[ "${SKIP_GROUND:-0}" != "1" ]]; then
  section "3. Ground station receive (${GROUND_IP}, Wi‑Fi ${GROUND_WIFI_IFACE})"

  GROUND_REPO_RESOLVED="$(detect_ground_repo)"
  echo "Ground docker config: ${GROUND_REPO_RESOLVED}/${GROUND_DDS_FILE}"

  subsection "Wi‑Fi link"
  ssh_ground "iw dev ${GROUND_WIFI_IFACE} link 2>/dev/null || ip -br addr show ${GROUND_WIFI_IFACE}" 2>/dev/null || echo "  (could not read Wi‑Fi info)"

  for topic in "${HEAVY_TOPICS[@]}"; do
    subsection "${topic}"
    if OUT="$(ground_ros_bw "${topic}" "${GROUND_REPO_RESOLVED}" "${GROUND_WIFI_IFACE}" 2>/dev/null || true)"; then
      ros_line="$(echo "${OUT}" | grep '^ROS_LINE:' | cut -d: -f2- | sed 's/^://')"
      wifi_mbs="$(echo "${OUT}" | grep '^WIFI_MBS:' | cut -d: -f2-)"
      if [[ -n "${ros_line}" ]]; then
        echo "  ros2 topic bw: ${ros_line}"
      else
        echo "  ros2 topic bw: (no stable rate — link may be congested)"
      fi
      if [[ -n "${wifi_mbs}" && "${wifi_mbs}" != "0.0000" ]]; then
        GROUND_WIFI_MBS["${topic}"]="${wifi_mbs}"
        echo "  Wi‑Fi RX avg:  ${wifi_mbs} MB/s ($(mbps_from_mbs "${wifi_mbs}") Mbps)"
      fi
      mbs="$(echo "${ros_line}" | grep -oE '^[0-9]+\.[0-9]+' || true)"
      [[ -n "${mbs}" ]] && GROUND_ROS_MBS["${topic}"]="${mbs}"
    else
      echo "  (SSH/measurement failed)"
    fi
  done
fi

# --- pi receive (optional cross-check) ---
declare -A PI_MBS=()

section "4. Pi receive (${PI_IP}, ${PI_ETH_IFACE})"
for topic in "${HEAVY_TOPICS[@]}"; do
  subsection "${topic}"
  if OUT="$(pi_ros "timeout ${MEASURE_SEC} ros2 topic bw ${topic} 2>&1" || true)"; then
    echo "${OUT}" | grep -E 'mean:|MB/s|KB/s|from [0-9]+ messages|Subscribed' | tail -4 || echo "  (no data)"
    mbs="$(echo "${OUT}" | parse_ros_bw_mbs || true)"
    if [[ -n "${mbs}" ]]; then
      PI_MBS["${topic}"]="${mbs}"
      echo "  >> parsed: ${mbs} MB/s ($(mbps_from_mbs "${mbs}") Mbps)"
    fi
  else
    echo "  (measurement failed)"
  fi
done

# --- iperf ---
IPERF_MBIT=""

if [[ "${SKIP_IPERF:-0}" != "1" && "${SKIP_GROUND:-0}" != "1" ]]; then
  section "5. Wi‑Fi link test (iperf3 Ground → Pi)"
  if command -v iperf3 >/dev/null 2>&1; then
    pkill -f 'iperf3 -s' 2>/dev/null || true
    iperf3 -s -D 2>/dev/null || true
    sleep 1
    if OUT="$(ssh_ground "command -v iperf3 >/dev/null && iperf3 -c ${PI_IP} -t ${IPERF_SEC} -f m" 2>&1)"; then
      echo "${OUT}" | tail -5
      IPERF_MBIT="$(echo "${OUT}" | grep -E 'receiver|sender' | tail -1 | grep -oE '[0-9]+\.[0-9]+' | tail -1 || true)"
      [[ -n "${IPERF_MBIT}" ]] && echo "  >> sustained (~${IPERF_MBIT} Mbit/s)"
    else
      echo "  iperf3 failed (install on ground: sudo apt install iperf3)"
    fi
    pkill -f 'iperf3 -s' 2>/dev/null || true
  else
    echo "  iperf3 not installed on this host (sudo apt install iperf3)"
  fi
fi

# --- summary ---
section "6. Summary"

jetson_total=0
ground_wifi_total=0
for topic in "${HEAVY_TOPICS[@]}"; do
  j="${JETSON_MBS[$topic]:-}"
  g="${GROUND_WIFI_MBS[$topic]:-}"
  [[ -n "${j}" ]] && jetson_total="$(awk -v a="${jetson_total}" -v b="${j}" 'BEGIN { print a+b }')"
  [[ -n "${g}" ]] && ground_wifi_total="$(awk -v a="${ground_wifi_total}" -v b="${g}" 'BEGIN { print a+b }')"
done

printf "%-45s %12s %12s %12s\n" "Topic" "Jetson src" "Ground WiFi" "Pi recv"
printf "%-45s %12s %12s %12s\n" "-----" "----------" "-----------" "-------"
for topic in "${HEAVY_TOPICS[@]}"; do
  j="${JETSON_MBS[$topic]:--}"
  g="${GROUND_WIFI_MBS[$topic]:--}"
  p="${PI_MBS[$topic]:--}"
  [[ "${j}" != "-" ]] && j="${j} MB/s"
  [[ "${g}" != "-" ]] && g="${g} MB/s"
  [[ "${p}" != "-" ]] && p="${p} MB/s"
  printf "%-45s %12s %12s %12s\n" "${topic}" "${j}" "${g}" "${p}"
done

echo ""
if awk -v t="${jetson_total}" 'BEGIN { exit (t > 0) ? 0 : 1 }'; then
  echo "Jetson heavy-stream total (if all subscribed): ~${jetson_total} MB/s (~$(mbps_from_mbs "${jetson_total}") Mbps)"
else
  echo "Jetson heavy-stream total: (not measured — is perception running?)"
fi

if awk -v t="${ground_wifi_total}" 'BEGIN { exit (t > 0) ? 0 : 1 }'; then
  echo "Ground Wi‑Fi measured sum (one topic at a time): ~${ground_wifi_total} MB/s (~$(mbps_from_mbs "${ground_wifi_total}") Mbps)"
fi

[[ -n "${IPERF_MBIT}" ]] && echo "iperf3 Ground→Pi sustained: ~${IPERF_MBIT} Mbit/s"

echo ""
echo "Reference — ${ROUTER_MODEL}:"
echo "  Wi‑Fi 2.4 GHz 802.11n, 300 Mbps theoretical; 100 Mbps Ethernet ports."
echo "  Real Wi‑Fi LAN throughput is often 20–40 Mbps on this class of router."
echo ""
echo "Interpretation:"
echo "  • If Jetson total >> iperf/Ground receive, the network is the bottleneck."
echo "  • View LiDAR-only in RViz over Wi‑Fi; add ZED streams only on wired ground link."
echo "  • Re-run after router changes: ./bandwidth_report.sh | tee reports/bw_\$(date +%Y%m%d).txt"

section "Done"
