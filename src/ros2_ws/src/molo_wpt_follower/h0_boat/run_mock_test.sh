#!/usr/bin/env bash
# End-to-end mock test: Pi publishes mock frontseat sensors, Jetson runs h0_boat MPC.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(cd "${SCRIPT_DIR}/../../../../docker" && pwd)"

JETSON_IP="${JETSON_IP:-192.168.0.102}"
JETSON_USER="${JETSON_SSH_USER:-orin-nano}"
JETSON_PASS="${SSHPASS_JETSON:-1}"
ORIGIN_LAT="${ORIGIN_LAT:-40.448417}"
ORIGIN_LON="${ORIGIN_LON:--86.867750}"
WARMUP="${WARMUP:-8}"
DURATION="${DURATION:-20}"
SKIP_INITIAL="${SKIP_INITIAL:-8}"
OUT_DIR="${OUT_DIR:-/tmp/h0_boat_mock_test}"
MOCK_OVERLAY="${SCRIPT_DIR}/config/h0_mock_overlay.yaml"

log() { echo "[h0-mock-test] $*"; }
die() { echo "[h0-mock-test] ERROR: $*" >&2; exit 1; }

ssh_jetson() {
  sshpass -p "${JETSON_PASS}" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    "${JETSON_USER}@${JETSON_IP}" "$@"
}

log "Syncing repo to Jetson..."
SSHPASS_JETSON="${JETSON_PASS}" "${DOCKER_DIR}/mini_bream_sync.sh" jetson || die "sync failed"

log "Checking MPC deps on Jetson perception container..."
ssh_jetson "docker exec mini_bream_perception python3 -c \
  'import dubins, osqp, scipy, matplotlib, utm, transforms3d'" \
  || die "MPC deps missing in mini_bream_perception. On Jetson rebuild:\n  cd ~/mini-bream/src/docker && docker compose -f docker-compose.frontseat.yml build perception"

log "Starting mock frontseat on Pi (frontseat container)..."
# Do not use pgrep|kill inside bash -lc: the pattern appears in the shell argv and kills the session (exit 143).
docker exec mini_bream_frontseat pkill -f 'h0_boat/mock_frontseat.py' 2>/dev/null || true
docker exec -d mini_bream_frontseat bash -lc \
  "source /opt/ros/humble/setup.bash && \
   export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   export CYCLONEDDS_URI=file:///etc/cyclonedds.xml && \
   python3 /workspace/ros2_ws/src/molo_wpt_follower/h0_boat/mock_frontseat.py \
     --origin-lat ${ORIGIN_LAT} --origin-lon ${ORIGIN_LON} \
     --gps-topic /mock/wamv/sensors/gps/gps/fix \
     --imu-topic /mock/wamv/sensors/imu/imu/data" \
  || die "failed to start mock_frontseat on Pi"

sleep 2
log "Checking mock GPS topic on Pi..."
docker exec mini_bream_frontseat bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 6 ros2 topic echo /mock/wamv/sensors/gps/gps/fix --once" \
  | grep -q latitude || die "mock GPS not publishing on Pi"

log "Running h0_boat experiment on Jetson..."
ssh_jetson "docker exec mini_bream_perception bash -lc \
  'source /opt/ros/humble/setup.bash && \
   export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} && \
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   export CYCLONEDDS_URI=file:///etc/cyclonedds.jetson.xml && \
   python3 /workspace/ros2_ws/src/molo_wpt_follower/h0_boat/run_h0_experiment.py \
     --overlay /workspace/ros2_ws/src/molo_wpt_follower/h0_boat/config/h0_mock_overlay.yaml \
     --origin-lat ${ORIGIN_LAT} --origin-lon ${ORIGIN_LON} \
     --out ${OUT_DIR} --warmup ${WARMUP} --duration ${DURATION} --skip-initial ${SKIP_INITIAL} \
     --no-plot'" \
  || die "Jetson experiment failed"

log "Verifying experiment artifacts on Jetson..."
RESULT_JSON="${OUT_DIR}/H0_baseline/result.json"
if ssh_jetson "docker exec mini_bream_perception test -f ${RESULT_JSON}"; then
  log "Result:"
  ssh_jetson "docker exec mini_bream_perception cat ${RESULT_JSON}"
else
  die "result.json not found at ${RESULT_JSON} (inside mini_bream_perception)"
fi

LOG_CSV="${OUT_DIR}/H0_baseline/log.csv"
if ssh_jetson "docker exec mini_bream_perception test -s ${LOG_CSV}"; then
  LINES=$(ssh_jetson "docker exec mini_bream_perception bash -lc 'wc -l < ${LOG_CSV}'")
  log "log.csv lines: ${LINES}"
  [[ "${LINES}" -gt 5 ]] || die "log.csv too short"
else
  die "log.csv missing or empty"
fi

log "Mock integration test PASSED"
log "RViz: ./mini_bream_env.sh start gs --h0-boat"
