#!/usr/bin/env bash
# Host-side helper: stop any running BlueBoat Gazebo sim, start a fresh one,
# wait until GPS/odom are publishing. Run from the host (not inside autonomy).
#
# Usage:
#   ./ensure_clean_sim.sh
#   ./ensure_clean_sim.sh --no-start   # only stop
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(cd "${SCRIPT_DIR}/../../../../docker" && pwd)"
COMPOSE_FILE="${DOCKER_DIR}/docker-compose.simulation.yml"
NO_START=0

log() { echo "[gs-clean-sim] $*"; }
die() { echo "[gs-clean-sim] ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-start) NO_START=1 ;;
    -h|--help)
      echo "Usage: $0 [--no-start]"; exit 0 ;;
    *) die "unknown arg: $1" ;;
  esac
  shift
done

[[ -f "${COMPOSE_FILE}" ]] || die "missing ${COMPOSE_FILE}"

# Kill autonomy-side controller leftovers that may still publish thrust
if docker ps --format '{{.Names}}' | grep -qx mini_bream_autonomy; then
  log "Stopping leftover GS/ILOS/H0 stacks in autonomy..."
  docker exec mini_bream_autonomy bash -lc '
    pkill -f "ground_speed_mpc_ilos/stack_runner.py" 2>/dev/null || true
    pkill -f "ilos_boat/stack_runner.py" 2>/dev/null || true
    pkill -f "h0_boat/stack_runner.py" 2>/dev/null || true
    pkill -f "molo_wpt_follower/mpc/mpc.py" 2>/dev/null || true
  ' || true
fi

if docker ps -a --format '{{.Names}}' | grep -qx mini_bream_simulation; then
  log "Stopping existing mini_bream_simulation..."
  docker compose -f "${COMPOSE_FILE}" stop simulation >/dev/null 2>&1 || true
  docker compose -f "${COMPOSE_FILE}" rm -f simulation >/dev/null 2>&1 || true
fi

if [[ "${NO_START}" -eq 1 ]]; then
  log "Sim stopped (--no-start)."
  exit 0
fi

log "Starting clean mini_bream_simulation (open_water_harner)..."
# Force open water — no shore/dock/obstacle course that can trap the boat.
export SIM_WORLD="${SIM_WORLD:-open_water_harner}"
export SIM_LAUNCH="${SIM_LAUNCH:-open_water.launch.py}"
docker compose -f "${COMPOSE_FILE}" up -d simulation

log "Waiting for GPS/odom from autonomy domain..."
deadline=$((SECONDS + 120))
ok=0
while (( SECONDS < deadline )); do
  if docker exec mini_bream_autonomy bash -lc '
      source /opt/ros/humble/setup.bash
      if [[ -f /workspace/docker/dds_env.sh ]]; then source /workspace/docker/dds_env.sh; configure_cyclonedds; fi
      export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      timeout 3 ros2 topic echo /blueboat/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude
    ' >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 2
done
[[ "${ok}" -eq 1 ]] || die "sim started but GPS never appeared (timeout 120s)"

# Confirm we are not on the obstacle course world (best-effort from sim logs).
if docker logs mini_bream_simulation 2>&1 | tail -n 80 | grep -q "lidar_obstacle"; then
  log "WARNING: sim logs still mention lidar_obstacle — check SIM_WORLD/SIM_LAUNCH"
elif docker logs mini_bream_simulation 2>&1 | tail -n 80 | grep -q "open_water"; then
  log "Confirmed open_water world in sim logs."
fi
log "Clean sim ready (world=${SIM_WORLD})."
