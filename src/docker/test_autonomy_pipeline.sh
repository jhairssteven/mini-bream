#!/usr/bin/env bash
# Smoke-test H0 + ILOS experiment launchers inside mini_bream_autonomy.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

AUTONOMY_CONTAINER="mini_bream_autonomy"
SIM_CONTAINER="mini_bream_simulation"
GPS_TOPIC="/blueboat/sensors/gps/gps/fix"

log() { echo "[test-autonomy] $*"; }
die() { echo "[test-autonomy] ERROR: $*" >&2; exit 1; }

require_container() {
  local name="$1"
  docker ps --format '{{.Names}}' | grep -qx "${name}" || die "${name} is not running"
}

log "Checking autonomy image deps..."
docker exec "${AUTONOMY_CONTAINER}" python3 -c \
  'import dubins, osqp, scipy, skopt, utm, transforms3d; print("deps ok")'

log "Waiting for GPS on ${GPS_TOPIC}..."
if ! docker exec "${SIM_CONTAINER}" bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 30 ros2 topic echo ${GPS_TOPIC} --once" \
  | grep -q latitude; then
  die "no GPS on ${GPS_TOPIC} — is sim running?"
fi

H0_DIR="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat"
ILOS_DIR="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat"

log "H0 dry-run..."
docker exec "${AUTONOMY_CONTAINER}" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 ${H0_DIR}/run_h0_experiment.py --platform sim --dry-run --out /tmp/h0_autonomy_test"

log "ILOS dry-run..."
docker exec "${AUTONOMY_CONTAINER}" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 ${ILOS_DIR}/run_ilos_experiment.py --platform sim --dry-run --out /tmp/ilos_autonomy_test"

log "ILOS smoke experiment..."
docker exec "${AUTONOMY_CONTAINER}" bash -lc \
  "source /opt/ros/humble/setup.bash && python3 ${ILOS_DIR}/run_ilos_experiment.py \
   --platform sim --no-plot --warmup 5 --duration 20 --skip-initial 5 \
   --out /tmp/ilos_autonomy_smoke"

log "All autonomy pipeline checks passed."
