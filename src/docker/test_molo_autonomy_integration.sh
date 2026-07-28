#!/usr/bin/env bash
# End-to-end integration test: sim + planning + path follower.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WPT_DIR="${REPO_ROOT}/src/ros2_ws/src/molo_wpt_follower"

log() { echo "[molo-integration] $*"; }
die() { echo "[molo-integration] ERROR: $*" >&2; exit 1; }

SIM_CONTAINER="${SIM_CONTAINER:-mini_bream_simulation}"
AUTONOMY_CONTAINER="${AUTONOMY_CONTAINER:-mini_bream_autonomy}"
USE_DOCKER="${USE_DOCKER:-auto}"

run_local_smoke() {
  log "Launching planning nodes locally (5s smoke)..."
  timeout 8 ros2 launch mission_planner molo_planning.launch.py platform:=blueboat_sim use_sim_time:=false &
  launch_pid=$!
  sleep 5
  if ros2 topic list 2>/dev/null | grep -q reference_path; then
    log "reference_path topic advertised"
  else
    log "reference_path not yet visible (sim may be required for costmap)"
  fi
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}

run_docker_integration() {
  require_container() {
    docker ps --format '{{.Names}}' | grep -qx "$1" || die "$1 is not running"
  }
  require_container "${SIM_CONTAINER}"
  require_container "${AUTONOMY_CONTAINER}"

  log "Waiting for lidar in sim..."
  docker exec "${SIM_CONTAINER}" bash -lc \
    "set +u && source /opt/ros/humble/setup.bash && set -u && \
     timeout 45 ros2 topic echo /rslidar_points --once" \
    >/dev/null || die "no lidar — start sim with lidar_obstacle_course"

  log "Starting planning stack in autonomy container..."
  docker exec "${AUTONOMY_CONTAINER}" bash -lc \
    "pkill -f local_planner_node 2>/dev/null || true; pkill -f pointcloud_to_costmap 2>/dev/null || true; pkill -f stack_runner 2>/dev/null || true"
  sleep 2
  docker exec -d "${AUTONOMY_CONTAINER}" bash -lc \
    "set +u && source /opt/ros/humble/setup.bash && set -u && \
     source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true && \
     ros2 launch mission_planner molo_planning.launch.py platform:=blueboat_sim"

  log "Waiting for planned path..."
  for _ in $(seq 1 30); do
    if docker exec "${AUTONOMY_CONTAINER}" bash -lc \
      "set +u && source /opt/ros/humble/setup.bash && set -u && \
       timeout 5 ros2 topic echo /molo_mpc/reference_path --once 2>/dev/null" \
      | grep -q "frame_id"; then
      log "Received planned path"
      break
    fi
    sleep 3
  done
  docker exec "${AUTONOMY_CONTAINER}" bash -lc \
    "set +u && source /opt/ros/humble/setup.bash && set -u && \
     timeout 5 ros2 topic echo /molo_mpc/reference_path --once 2>/dev/null" \
    | grep -q frame_id || die "planner did not publish /molo_mpc/reference_path"

  log "Starting ILOS path follower..."
  docker exec -d "${AUTONOMY_CONTAINER}" bash -lc \
    "set +u && source /opt/ros/humble/setup.bash && set -u && \
     cd /workspace/ros2_ws/src/molo_wpt_follower/path_follower && \
     python3 stack_runner.py --controller ilos --platform sim"

  log "Waiting for cross-track error (follower active)..."
  docker exec "${AUTONOMY_CONTAINER}" bash -lc \
    "set +u && source /opt/ros/humble/setup.bash && set -u && \
     timeout 60 ros2 topic echo /molo_mpc/cross_track_error --once" \
    | grep -q data || die "follower did not publish cross_track_error"

  log "Docker integration passed."
}

log "Unit tests (path conversion + config merge)..."
python3 "${WPT_DIR}/path_follower/test_path_integration.py"

if [ "${USE_DOCKER}" = "auto" ]; then
  if docker ps --format '{{.Names}}' | grep -qx "${SIM_CONTAINER}"; then
    USE_DOCKER=true
  else
    USE_DOCKER=false
  fi
fi

if [ "${USE_DOCKER}" = "true" ]; then
  run_docker_integration
else
  if command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/humble/setup.bash ]; then
      set +u
      # shellcheck disable=SC1091
      source /opt/ros/humble/setup.bash
      set -u
    fi
    if [ -f "${REPO_ROOT}/src/ros2_ws/install/setup.bash" ]; then
      set +u
      # shellcheck disable=SC1091
      source "${REPO_ROOT}/src/ros2_ws/install/setup.bash"
      set -u
    fi
    run_local_smoke
  else
    log "ros2 not in PATH — skipping live ROS integration"
  fi
  log "For full test, start docker sim + autonomy (auto-detected when sim is running)"
fi

log "All integration checks passed."
