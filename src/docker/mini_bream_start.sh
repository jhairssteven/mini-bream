#!/usr/bin/env bash
# Start mini-bream Docker stack for Pi, Jetson, or ground station.
#
# Usage:
#   ./mini_bream_start.sh pi              # pwm_daemon + radio_rx + frontseat
#   ./mini_bream_start.sh jetson          # LiDAR + ZED perception
#   ./mini_bream_start.sh gs              # telemetry_tx + RViz ground_station
#
# Options:
#   --build              Rebuild images before starting
#   --dry-run            Pi only: PWM dry_run (no motor output)
#   --no-telemetry       GS only: skip telemetry_tx (RViz only)
#   --detach-frontseat   Pi only: run frontseat in background
#   -h, --help           Show help
#
# Environment (optional):
#   ROS_DOMAIN_ID=0      Must match on all hosts (default 0)
#   LIDAR_IFACE=eth0     Jetson LiDAR NIC (default eth0)
#   GROUND_WIFI_IFACE=wlp5s0
#   RADIO_SERIAL_PORT=...  SiK radio by-id path
#   PWM_BACKEND_OVERRIDE=dry_run

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

COMPOSE_FRONTSEAT="docker-compose.frontseat.yml"
COMPOSE_GROUND="docker-compose.ground.yml"
COMPOSE_TELEMETRY="docker-compose.ground.telemetry.yaml"
LIDAR_NET_SCRIPT="../ros2_ws/src/frontseat/config/rslidar_airy/setup_network.sh"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID

BUILD=0
DRY_RUN=0
NO_TELEMETRY=0
DETACH_FRONTSEAT=0
ROLE=""

usage() {
  sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

log() { echo "[mini-bream] $*"; }
die() { echo "[mini-bream] ERROR: $*" >&2; exit 1; }

need_docker() {
  command -v docker >/dev/null 2>&1 || die "docker not found"
  docker info >/dev/null 2>&1 || die "docker daemon not running"
}

compose() {
  docker compose "$@"
}

build_flag() {
  [[ "${BUILD}" -eq 1 ]] && echo --build
}

setup_jetson_lidar_network() {
  local iface="${LIDAR_IFACE:-eth0}"
  local script_path="${SCRIPT_DIR}/${LIDAR_NET_SCRIPT}"
  [[ -f "${script_path}" ]] || die "LiDAR network script not found: ${script_path}"

  log "LiDAR secondary IP on ${iface} (192.168.1.102/24)..."
  if [[ "$(id -u)" -eq 0 ]]; then
    LIDAR_IFACE="${iface}" bash "${script_path}"
  elif command -v sudo >/dev/null 2>&1; then
    sudo LIDAR_IFACE="${iface}" bash "${script_path}"
  else
    die "Need root or sudo to configure LiDAR secondary IP"
  fi
}

setup_ground_display() {
  if [[ -z "${DISPLAY:-}" ]]; then
    export DISPLAY=:0
    log "DISPLAY not set; using ${DISPLAY}"
  fi
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || log "warning: xhost failed (run from a graphical session?)"
  else
    log "warning: xhost not found; RViz may not open"
  fi
}

verify_ros_topics() {
  local container="$1"
  local pattern="${2:-rslidar|zed|fix}"
  sleep 5
  if docker exec "${container}" bash -lc \
    'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null' \
    | grep -qE "${pattern}"; then
    log "ROS topics visible in ${container}"
  else
    log "warning: no matching topics yet in ${container} (stack may still be starting)"
  fi
}

start_pi() {
  need_docker
  log "Starting Pi stack (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    export PWM_BACKEND_OVERRIDE=dry_run
    log "PWM dry_run enabled (no motor output)"
  fi

  log "Bringing up pwm_daemon + radio_rx (detached)..."
  compose -f "${COMPOSE_FRONTSEAT}" up -d $(build_flag) pwm_daemon radio_rx

  log "Starting frontseat..."
  if [[ "${DETACH_FRONTSEAT}" -eq 1 ]]; then
    compose -f "${COMPOSE_FRONTSEAT}" up -d $(build_flag) frontseat
    log "frontseat running detached (attach: docker exec -it mini_bream_frontseat bash)"
  else
    log "frontseat attaches here (tmux sessions inside; Ctrl+C to stop)"
    compose -f "${COMPOSE_FRONTSEAT}" up $(build_flag) frontseat
  fi
}

start_jetson() {
  need_docker
  log "Starting Jetson perception (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."

  setup_jetson_lidar_network

  log "Building/starting perception container..."
  compose -f "${COMPOSE_FRONTSEAT}" up -d $(build_flag) perception

  log "Perception started (logs: docker logs -f mini_bream_perception)"
  verify_ros_topics mini_bream_perception 'rslidar|zed'

  log "Done. LiDAR + ZED should publish on /rslidar_points and /zed/zed/..."
}

start_gs() {
  need_docker
  log "Starting ground station (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."

  setup_ground_display

  if [[ "${NO_TELEMETRY}" -eq 0 ]]; then
    log "Starting telemetry_tx (joystick → SiK radio, detached)..."
    compose -f "${COMPOSE_TELEMETRY}" up -d $(build_flag) telemetry_tx
  else
    log "Skipping telemetry_tx (--no-telemetry)"
  fi

  log "Starting ground_station RViz (foreground; Ctrl+C to stop)..."
  compose -f "${COMPOSE_GROUND}" up $(build_flag) ground_station
}

# --- parse args ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    pi|jetson|gs|ground|ground-station)
      [[ -z "${ROLE}" ]] || die "role already set: ${ROLE}"
      case "$1" in
        ground|ground-station) ROLE=gs ;;
        *) ROLE="$1" ;;
      esac
      ;;
    --build) BUILD=1 ;;
    --dry-run) DRY_RUN=1 ;;
    --no-telemetry) NO_TELEMETRY=1 ;;
    --detach-frontseat) DETACH_FRONTSEAT=1 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

[[ -n "${ROLE}" ]] || usage 1

case "${ROLE}" in
  pi) start_pi ;;
  jetson) start_jetson ;;
  gs) start_gs ;;
  *) die "unknown role: ${ROLE}" ;;
esac
