#!/usr/bin/env bash
# H0 lemniscate experiment launcher (all platforms).
#
# Run inside mini_bream_autonomy. Waits for required sensor topics (and
# motor_controller on the real boat) before starting the experiment.
#
# Examples:
#   ./run_real_boat.sh --platform sim
#   ./run_real_boat.sh --platform real
#   ./run_real_boat.sh --platform sim --tune --install
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

FIELD_TESTS="/workspace/field_tests/h0_boat"
AUTONOMY_OVERLAY="/workspace/docker/config/autonomy_overlay.yaml"

PLATFORM="real"
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_SET=0
OUT_DIR=""

TUNE=0
INSTALL_TUNED=0
SMOKE=0
NO_PLOT=0
DRY_RUN=0
SKIP_CHECKS=0
EXTRA_ARGS=()
TUNE_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run_real_boat.sh [options] [-- extra run_h0_experiment.py args]

Run the H0 lemniscate MPC experiment inside mini_bream_autonomy.

Prerequisites (must already be on the ROS graph):

  --platform sim    GPS/IMU/GT odom on /blueboat/sensors/...
  --platform real   GPS/IMU on /wamv/sensors/... and motor_controller
  --platform bench  GPS/IMU only (thrust published to sink topics)

Options:
  --platform P   Platform profile: sim, real, bench (default: real)
  --bench        Shorthand for --platform bench
  --tune         Run Bayesian optimization (tune_h0.py) instead of a single experiment
  --install      With --tune: write best params to config/h0_tuned_overlay.yaml
  --quick        With --tune: 8 BO calls; otherwise shortened evaluate window
  --smoke        Short warmup/evaluate window (experiment only)
  --dry-run      Write config + ref_path only (skips ROS graph checks)
  --skip-checks  Do not wait for prerequisite topics/nodes
  --no-plot      Skip plot generation
  --out DIR      Output directory
  -h, --help     Show this help

Mock integration test (Pi→Jetson): ./run_mock_test.sh
EOF
}

log() { echo "[h0-boat] $*"; }
die() { echo "[h0-boat] ERROR: $*" >&2; exit 1; }

topic_on_graph() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "${topic}"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if topic_on_graph "${topic}"; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_for_gps_msg() {
  local topic="$1"
  local timeout_s="${2:-20}"
  timeout "${timeout_s}" ros2 topic echo "${topic}" --once 2>/dev/null | grep -q latitude
}

node_on_graph() {
  local name="$1"
  ros2 node list 2>/dev/null | grep -Eq "(^|/)${name}$"
}

check_python_deps() {
  local tune="$1"
  if ! python3 -c 'import dubins, osqp, scipy, matplotlib, utm, transforms3d' 2>/dev/null; then
    die "MPC Python deps missing (need dubins, osqp, scipy, matplotlib, utm, transforms3d)."
  fi
  if [[ "${tune}" -eq 1 ]] && ! python3 -c 'import skopt' 2>/dev/null; then
    die "scikit-optimize missing (pip install scikit-optimize)."
  fi
}

autonomy_overlay_path() {
  if [[ -f "${AUTONOMY_OVERLAY}" ]]; then
    echo "${AUTONOMY_OVERLAY}"
  fi
}

check_platform_graph() {
  local platform="$1"
  case "${platform}" in
    sim)
      log "Checking sim topics..."
      wait_for_topic "/blueboat/sensors/gps/gps/fix" 20 \
        || die "missing /blueboat/sensors/gps/gps/fix (is blueboat_sim / Gazebo publishing?)"
      wait_for_topic "/blueboat/sensors/imu/imu/data" 10 \
        || die "missing /blueboat/sensors/imu/imu/data"
      wait_for_topic "/blueboat/sensors/position/ground_truth_odometry" 10 \
        || die "missing /blueboat/sensors/position/ground_truth_odometry"
      log "Waiting for GPS message on /blueboat/sensors/gps/gps/fix..."
      wait_for_gps_msg "/blueboat/sensors/gps/gps/fix" 20 \
        || die "no GPS message on /blueboat/sensors/gps/gps/fix"
      ;;
    real|bench)
      log "Checking boat sensor topics..."
      wait_for_topic "/wamv/sensors/gps/gps/fix" 20 \
        || die "missing /wamv/sensors/gps/gps/fix (is frontseat GPS publishing?)"
      wait_for_topic "/wamv/sensors/imu/imu/data" 10 \
        || die "missing /wamv/sensors/imu/imu/data"
      log "Waiting for GPS message on /wamv/sensors/gps/gps/fix..."
      wait_for_gps_msg "/wamv/sensors/gps/gps/fix" 15 \
        || die "no GPS message on /wamv/sensors/gps/gps/fix"
      if [[ "${platform}" == "real" ]]; then
        if ! node_on_graph "motor_controller"; then
          die "node motor_controller not on the graph (needed for /pwm/*_thrust_cmd)"
        fi
      fi
      ;;
  esac
}

default_out_dir() {
  local platform="$1"
  local ts="$2"
  if [[ "${platform}" == "sim" ]]; then
    echo "${SCRIPT_DIR}/results/${ts}"
  else
    echo "${FIELD_TESTS}/${ts}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --platform)
      shift
      PLATFORM="${1:?missing value for --platform}"
      ;;
    --bench) PLATFORM="bench" ;;
    --tune) TUNE=1 ;;
    --install) INSTALL_TUNED=1 ;;
    --quick) TUNE_ARGS+=(--quick) ;;
    --smoke) SMOKE=1 ;;
    --dry-run) DRY_RUN=1 ;;
    --skip-checks) SKIP_CHECKS=1 ;;
    --no-plot) NO_PLOT=1 ;;
    --out)
      shift
      OUT_DIR="${1:?missing value for --out}"
      OUT_SET=1
      ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_ARGS+=("$@"); break ;;
    *) EXTRA_ARGS+=("$1") ;;
  esac
  shift
done

case "${PLATFORM}" in
  sim|real|bench) ;;
  *) die "unknown platform: ${PLATFORM} (expected sim, real, or bench)" ;;
esac

h0_require_ros || exit 1
check_python_deps "${TUNE}"

if [[ "${OUT_SET}" -eq 0 ]]; then
  OUT_DIR="$(default_out_dir "${PLATFORM}" "${RUN_TS}")"
fi

OVERLAY_ARGS=()
if [[ "${PLATFORM}" == "real" || "${PLATFORM}" == "bench" ]]; then
  OVERLAY="$(autonomy_overlay_path)"
  if [[ -n "${OVERLAY}" ]]; then
    OVERLAY_ARGS+=(--overlay "${OVERLAY}")
  fi
fi

if [[ "${TUNE}" -eq 1 ]]; then
  TUNE_OUT="${OUT_DIR}"
  if [[ "${OUT_SET}" -eq 0 ]]; then
    TUNE_OUT="${SCRIPT_DIR}/results/tune_${PLATFORM}_${RUN_TS}"
  fi
  mkdir -p "${TUNE_OUT}"

  if [[ "${SKIP_CHECKS}" -eq 0 ]]; then
    check_platform_graph "${PLATFORM}"
  fi

  log "Running H0 Bayesian tuning (platform=${PLATFORM})..."
  pkill -f 'h0_boat/stack_runner.py' 2>/dev/null || true
  pkill -f 'h0_boat/mock_frontseat.py' 2>/dev/null || true
  TUNE_CMD=(
    python3 "${SCRIPT_DIR}/tune_h0.py"
    --platform "${PLATFORM}"
    --out "${TUNE_OUT}"
  )
  TUNE_CMD+=("${TUNE_ARGS[@]}")
  [[ "${INSTALL_TUNED}" -eq 1 ]] && TUNE_CMD+=(--install)
  TUNE_CMD+=("${OVERLAY_ARGS[@]}")
  TUNE_CMD+=("${EXTRA_ARGS[@]}")
  "${TUNE_CMD[@]}"

  RESULT_JSON="${TUNE_OUT}/tune_result.json"
  [[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
  log "Tuning result:"
  cat "${RESULT_JSON}"
  log "H0 tuning complete"
  log "Artifacts: ${TUNE_OUT}/"
  exit 0
fi

mkdir -p "${OUT_DIR}"

RUN_ARGS=(--platform "${PLATFORM}" --out "${OUT_DIR}")
[[ "${DRY_RUN}" -eq 1 ]] && RUN_ARGS+=(--dry-run)
[[ "${NO_PLOT}" -eq 1 ]] && RUN_ARGS+=(--no-plot)
RUN_ARGS+=("${OVERLAY_ARGS[@]}")
RUN_ARGS+=("${EXTRA_ARGS[@]}")

if [[ "${SMOKE}" -eq 1 ]]; then
  if [[ "${PLATFORM}" == "sim" ]]; then
    RUN_ARGS+=(--warmup 5 --duration 20 --skip-initial 5)
  else
    RUN_ARGS+=(--warmup 8 --duration 15 --skip-initial 8)
  fi
  log "Smoke mode: shortened evaluate window"
fi

if [[ "${PLATFORM}" == "bench" ]]; then
  log "Bench mode: real sensors, thrust to sink topics (no motors)"
fi

if [[ "${DRY_RUN}" -eq 0 && "${SKIP_CHECKS}" -eq 0 ]]; then
  check_platform_graph "${PLATFORM}"
fi

pkill -f 'h0_boat/stack_runner.py' 2>/dev/null || true
pkill -f 'h0_boat/mock_frontseat.py' 2>/dev/null || true

log "Output: ${OUT_DIR}"
log "Running experiment..."
"${SCRIPT_DIR}/run_h0_boat.sh" "${RUN_ARGS[@]}"

RESULT_JSON="${OUT_DIR}/H0_baseline/result.json"
LOG_CSV="${OUT_DIR}/H0_baseline/log.csv"
[[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
if [[ "${DRY_RUN}" -eq 0 ]]; then
  [[ -s "${LOG_CSV}" ]] || die "missing or empty ${LOG_CSV}"
  LINES=$(wc -l < "${LOG_CSV}")
  log "log.csv lines: ${LINES}"
  [[ "${LINES}" -gt 5 ]] || die "log.csv too short"
fi

log "Result:"
cat "${RESULT_JSON}"
log "H0 experiment complete (platform=${PLATFORM})"
log "Artifacts: ${OUT_DIR}/H0_baseline/"
