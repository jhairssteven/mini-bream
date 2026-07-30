#!/usr/bin/env bash
# Start or stop mini-bream Docker stacks for Pi, Jetson, or ground station.
#
# Usage:
#   ./mini_bream_env.sh start pi              # pwm_daemon + radio_rx + frontseat
#   ./mini_bream_env.sh start jetson          # LiDAR + ZED perception
#   ./mini_bream_env.sh start gs              # telemetry_tx + RViz ground_station
#   ./mini_bream_env.sh start gs --h0-boat    # RViz with boat path overlays (world frame)
#   ./mini_bream_env.sh start gs --h0-boat --sim-viz  # same + use_sim_time (Gazebo sim)
#   ./mini_bream_env.sh stop pi               # remove Pi containers (compose down -v)
#   ./mini_bream_env.sh stop jetson           # remove perception
#   ./mini_bream_env.sh start autonomy       # Jetson/dev: H0 + ILOS + planning stack
#   ./mini_bream_env.sh start sim            # BlueBoat Gazebo simulation (HAL)
#   ./mini_bream_env.sh stop sim             # remove simulation container
#
# Start options:
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

log() { echo "[mini-bream] $*"; }
die() { echo "[mini-bream] ERROR: $*" >&2; exit 1; }

COMPOSE_FRONTSEAT="docker-compose.frontseat.yml"
COMPOSE_GROUND="docker-compose.ground.yml"
COMPOSE_SIMULATION="docker-compose.simulation.yml"
COMPOSE_AUTONOMY="docker-compose.autonomy.yml"
COMPOSE_TELEMETRY="docker-compose.ground.telemetry.yaml"

LIDAR_NET_SCRIPT="../ros2_ws/src/frontseat/config/rslidar_airy/setup_network.sh"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID

# Propagate host timezone into containers (images default to UTC; RTC may read as 1970).
export_host_tz() {
  if [[ -n "${TZ:-}" ]]; then
    return 0
  fi
  if [[ -f /etc/timezone ]]; then
    TZ="$(tr -d '[:space:]' < /etc/timezone)"
  elif command -v timedatectl >/dev/null 2>&1; then
    TZ="$(timedatectl show -p Timezone --value 2>/dev/null || true)"
  fi
  if [[ -n "${TZ:-}" ]]; then
    export TZ
    log "Container TZ=${TZ} (from host)"
  fi
}
export_host_tz

# Bind-mount resolved zoneinfo (host /etc/localtime is often a symlink).
if [[ -z "${LOCALTIME_PATH:-}" && -e /etc/localtime ]]; then
  LOCALTIME_PATH="$(readlink -f /etc/localtime)"
  export LOCALTIME_PATH
fi

ACTION=""
ROLE=""
BUILD=0
DRY_RUN=0
NO_TELEMETRY=0
DETACH_FRONTSEAT=0
H0_BOAT_RVIZ=0
SIM_VIZ=0

usage() {
  sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

normalize_role() {
  case "$1" in
    ground|ground-station) echo gs ;;
    pi|jetson|gs|sim|autonomy) echo "$1" ;;
    *) return 1 ;;
  esac
}

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

teardown_services() {
  local compose_file="$1"
  shift
  local -a services=("$@")
  local found=0

  for svc in "${services[@]}"; do
    if docker ps -a --format '{{.Names}}' | grep -qx "mini_bream_${svc}"; then
      found=1
      break
    fi
  done

  if [[ "${found}" -eq 0 ]]; then
    log "No containers for: ${services[*]}"
    return 0
  fi

  log "Removing ${services[*]} (docker compose down -v)..."
  compose -f "${compose_file}" down -v "${services[@]}"
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

stop_pi() {
  need_docker
  log "Tearing down Pi stack..."
  teardown_services "${COMPOSE_FRONTSEAT}" frontseat radio_rx pwm_daemon
  log "Pi stack removed"
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

stop_jetson() {
  need_docker
  log "Tearing down Jetson perception..."
  teardown_services "${COMPOSE_FRONTSEAT}" perception
  log "Jetson perception removed"
}

start_gs() {
  need_docker
  log "Starting ground station (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."

  setup_ground_display

  if [[ "${H0_BOAT_RVIZ}" -eq 1 ]]; then
    export RVIZ_CONFIG=/workspace/docker/config/h0_boat.rviz
    log "Using boat experiment RViz config (${RVIZ_CONFIG})"
  fi

  if [[ "${SIM_VIZ}" -eq 1 ]]; then
    export USE_SIM_TIME=1
    log "RViz use_sim_time enabled (required for Gazebo / blueboat_sim topics)"
  fi

  if [[ "${NO_TELEMETRY}" -eq 0 ]]; then
    log "Starting telemetry_tx (joystick → SiK radio, detached)..."
    compose -f "${COMPOSE_TELEMETRY}" up -d $(build_flag) telemetry_tx
  else
    log "Skipping telemetry_tx (--no-telemetry)"
  fi

  log "Starting ground_station RViz (foreground; Ctrl+C to stop)..."
  compose -f "${COMPOSE_GROUND}" up $(build_flag) ground_station
}

stop_gs() {
  need_docker
  log "Tearing down ground station stack..."
  teardown_services "${COMPOSE_GROUND}" ground_station
  teardown_services "${COMPOSE_TELEMETRY}" telemetry_tx
  log "Ground station stack removed"
}

start_sim() {
  need_docker
  log "Starting BlueBoat simulation (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."
  setup_ground_display
  log "Building/starting simulation container..."
  compose -f "${COMPOSE_SIMULATION}" up $(build_flag) simulation
}

stop_sim() {
  need_docker
  log "Tearing down simulation..."
  teardown_services "${COMPOSE_SIMULATION}" simulation
  log "Simulation removed"
}

start_autonomy() {
  need_docker
  log "Starting autonomy stack (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})..."
  compose -f "${COMPOSE_AUTONOMY}" up -d $(build_flag) autonomy
  log "Autonomy started (attach: docker exec -it mini_bream_autonomy bash)"
  log "Experiments: h0_boat/run_real_boat.sh or ilos_boat/run_real_boat.sh"
}

stop_autonomy() {
  need_docker
  log "Tearing down autonomy..."
  teardown_services "${COMPOSE_AUTONOMY}" autonomy
  log "Autonomy removed"
}

run_action() {
  case "${ACTION}:${ROLE}" in
    start:pi) start_pi ;;
    stop:pi) stop_pi ;;
    start:jetson) start_jetson ;;
    stop:jetson) stop_jetson ;;
    start:gs) start_gs ;;
    stop:gs) stop_gs ;;
    start:sim) start_sim ;;
    stop:sim) stop_sim ;;
    start:autonomy) start_autonomy ;;
    stop:autonomy) stop_autonomy ;;
    *) die "unknown action/role: ${ACTION} ${ROLE}" ;;
  esac
}

# --- parse args ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    start|stop)
      [[ -z "${ACTION}" ]] || die "action already set: ${ACTION}"
      ACTION="$1"
      ;;
    pi|jetson|gs|sim|autonomy|ground|ground-station)
      [[ -z "${ROLE}" ]] || die "role already set: ${ROLE}"
      ROLE="$(normalize_role "$1")" || die "unknown role: $1"
      ;;
    --build) BUILD=1 ;;
    --dry-run) DRY_RUN=1 ;;
    --no-telemetry) NO_TELEMETRY=1 ;;
    --detach-frontseat) DETACH_FRONTSEAT=1 ;;
    --h0-boat|--ilos-boat) H0_BOAT_RVIZ=1 ;;
    --sim-viz) SIM_VIZ=1 ;;
    -h|--help) usage 0 ;;
    *) die "unknown argument: $1 (try --help)" ;;
  esac
  shift
done

[[ -n "${ACTION}" ]] || usage 1
[[ -n "${ROLE}" ]] || die "missing role (pi, jetson, gs, sim, or autonomy)"

if [[ "${ACTION}" == stop ]]; then
  if [[ "${BUILD}" -eq 1 || "${DRY_RUN}" -eq 1 || "${NO_TELEMETRY}" -eq 1 || "${DETACH_FRONTSEAT}" -eq 1 ]]; then
    die "start-only options cannot be used with stop"
  fi
fi

run_action
