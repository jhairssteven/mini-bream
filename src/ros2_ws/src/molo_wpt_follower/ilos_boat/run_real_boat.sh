#!/usr/bin/env bash
# ILOS+PID lemniscate experiment launcher (all platforms).
#
# Pure ILOS heading + PID yaw rate + differential thrust (no MPC).
#
# Examples:
#   ./run_real_boat.sh --platform sim
#   ./run_real_boat.sh --platform real
#   ./run_real_boat.sh --platform sim --tune --install
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
FIELD_TESTS_HOST="${REPO_ROOT}/field_tests/ilos_boat"
FIELD_TESTS_CONTAINER="/workspace/field_tests/ilos_boat"

PLATFORM="real"
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_SET=0
OUT_DIR=""
CONTAINER_OUT=""

BENCH=0
TUNE=0
INSTALL_TUNED=0
SMOKE=0
NO_PLOT=0
DRY_RUN=0
EXTRA_ARGS=()
TUNE_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run_real_boat.sh [options] [-- extra run_ilos_experiment.py args]

Run the ILOS+PID lemniscate experiment (no MPC). Start the platform stack first:

  --platform sim    blueboat_sim: cd src/docker && ./mini_bream_env.sh start sim
  --platform real   frontseat:   cd src/docker && ./mini_bream_env.sh start pi
  --platform bench  frontseat, motors idle (same as --bench)

Options:
  --platform P  Platform profile: sim, real, bench (default: real)
  --bench       Shorthand for --platform bench
  --tune        Run Bayesian optimization (tune_ilos.py) instead of a single experiment
  --install     With --tune: write best params to config/ilos_tuned_overlay.yaml
  --quick       With --tune: 8 BO calls; otherwise shortened evaluate window
  --smoke       Short warmup/evaluate window (experiment only)
  --dry-run     Write config + ref_path only
  --no-plot     Skip plot generation
  --out DIR     Output directory
  -h, --help    Show this help
EOF
}

log() { echo "[ilos-boat] $*"; }
die() { echo "[ilos-boat] ERROR: $*" >&2; exit 1; }

check_deps() {
  local container="$1"
  local rebuild_hint="$2"
  if ! docker exec "${container}" python3 -c \
      'import dubins, scipy, matplotlib, utm, transforms3d' 2>/dev/null; then
    die "Python deps missing in ${container}. Rebuild the image:\n  ${rebuild_hint}"
  fi
}

check_tune_deps() {
  local container="$1"
  check_deps "$@"
  if ! docker exec "${container}" python3 -c 'import skopt' 2>/dev/null; then
    die "scikit-optimize missing in ${container}. Install with:\n  docker exec ${container} pip3 install 'numpy<2' scikit-optimize"
  fi
}

ros_topic_ready() {
  local topic="$1"
  local timeout_s="${2:-15}"
  local runner="${3:-}"

  local cmd="source /opt/ros/humble/setup.bash"
  if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
    cmd+=" && source /workspace/ros2_ws/install/setup.bash"
  fi
  cmd+=" && timeout ${timeout_s} ros2 topic echo ${topic} --once"

  if [[ -n "${runner}" ]]; then
    docker exec "${runner}" bash -lc "${cmd}" | grep -q latitude
  elif command -v ros2 >/dev/null 2>&1; then
    bash -lc "${cmd}" | grep -q latitude
  else
    return 1
  fi
}

run_in_container() {
  local container="$1"
  shift
  docker exec "${container}" /workspace/docker/start_ilos_boat.sh "$@"
}

run_locally() {
  cd "${SCRIPT_DIR}"
  ./run_ilos_boat.sh "$@"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --platform)
      shift
      PLATFORM="${1:?missing value for --platform}"
      ;;
    --bench) BENCH=1; PLATFORM="bench" ;;
    --tune) TUNE=1 ;;
    --install) INSTALL_TUNED=1 ;;
    --quick) TUNE_ARGS+=(--quick) ;;
    --smoke) SMOKE=1 ;;
    --dry-run) DRY_RUN=1 ;;
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

container_out_for() {
  local host_path="$1"
  local rel="${host_path#${SCRIPT_DIR}/}"
  if [[ "${rel}" != "${host_path}" ]]; then
    echo "/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat/${rel}"
  else
    echo "${FIELD_TESTS_CONTAINER}/$(basename "${host_path}")"
  fi
}

if [[ "${OUT_SET}" -eq 0 ]]; then
  if [[ "${PLATFORM}" == "sim" ]]; then
    OUT_DIR="${SCRIPT_DIR}/results/${RUN_TS}"
  else
    OUT_DIR="${FIELD_TESTS_HOST}/${RUN_TS}"
  fi
fi
CONTAINER_OUT="$(container_out_for "${OUT_DIR}")"

TUNE_OUT="${OUT_DIR}"
if [[ "${TUNE}" -eq 1 ]]; then
  TUNE_OUT="${OUT_DIR:-${SCRIPT_DIR}/results/tune_${PLATFORM}_${RUN_TS}}"
  mkdir -p "${TUNE_OUT}"
  TUNE_CMD=(python3 "${SCRIPT_DIR}/tune_ilos.py" --platform "${PLATFORM}" --out "${TUNE_OUT}")
  TUNE_CMD+=("${TUNE_ARGS[@]}")
  [[ "${INSTALL_TUNED}" -eq 1 ]] && TUNE_CMD+=(--install)
  TUNE_CMD+=("${EXTRA_ARGS[@]}")

  if [[ "${PLATFORM}" == "sim" ]]; then
    if docker ps --format '{{.Names}}' | grep -qx mini_bream_simulation; then
      log "Running ILOS Bayesian tuning in mini_bream_simulation..."
      check_tune_deps mini_bream_simulation \
        "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.simulation.yml build simulation"
      CONTAINER_TUNE_OUT="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat/results/$(basename "${TUNE_OUT}")"
      docker exec mini_bream_simulation mkdir -p "$(dirname "${CONTAINER_TUNE_OUT}")"
      docker exec mini_bream_simulation bash -lc \
        "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && \
         python3 /workspace/ros2_ws/src/molo_wpt_follower/ilos_boat/tune_ilos.py \
         --platform sim --out ${CONTAINER_TUNE_OUT} \
         ${TUNE_ARGS[*]+"${TUNE_ARGS[*]}"} \
         ${INSTALL_TUNED:+--install} \
         ${EXTRA_ARGS[*]+"${EXTRA_ARGS[*]}"}"
      TUNE_OUT="${SCRIPT_DIR}/results/$(basename "${TUNE_OUT}")"
    elif command -v python3 >/dev/null 2>&1; then
      log "Running ILOS Bayesian tuning locally..."
      cd "${SCRIPT_DIR}"
      ./run_ilos_boat.sh "${TUNE_CMD[@]}"
    else
      die "no environment for tuning. Start sim: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"
    fi
  else
    if ! docker ps --format '{{.Names}}' | grep -qx mini_bream_frontseat; then
      die "mini_bream_frontseat not running. Start with: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi"
    fi
    check_tune_deps mini_bream_frontseat \
      "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.frontseat.yml build frontseat"
    CONTAINER_TUNE_OUT="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat/results/$(basename "${TUNE_OUT}")"
    log "Running ILOS field tuning in mini_bream_frontseat..."
    docker exec mini_bream_frontseat bash -lc \
      "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && \
       python3 /workspace/ros2_ws/src/molo_wpt_follower/ilos_boat/tune_ilos.py \
       --platform ${PLATFORM} --out ${CONTAINER_TUNE_OUT} \
       ${TUNE_ARGS[*]+"${TUNE_ARGS[*]}"} \
       ${INSTALL_TUNED:+--install} \
       ${EXTRA_ARGS[*]+"${EXTRA_ARGS[*]}"}"
    TUNE_OUT="${SCRIPT_DIR}/results/$(basename "${TUNE_OUT}")"
  fi

  RESULT_JSON="${TUNE_OUT}/tune_result.json"
  [[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
  log "Tuning result:"
  cat "${RESULT_JSON}"
  log "ILOS tuning complete"
  log "Artifacts: ${TUNE_OUT}/"
  exit 0
fi

mkdir -p "${OUT_DIR}"

RUN_ARGS=(--platform "${PLATFORM}" --out "${CONTAINER_OUT}")
[[ "${DRY_RUN}" -eq 1 ]] && RUN_ARGS+=(--dry-run)
[[ "${NO_PLOT}" -eq 1 ]] && RUN_ARGS+=(--no-plot)
RUN_ARGS+=("${EXTRA_ARGS[@]}")

if [[ "${PLATFORM}" == "sim" ]]; then
  GPS_TOPIC="/blueboat/sensors/gps/gps/fix"
  RUN_CONTAINER=""
  if docker ps --format '{{.Names}}' | grep -qx mini_bream_simulation; then
    RUN_CONTAINER="mini_bream_simulation"
    log "Using mini_bream_simulation container"
    check_deps mini_bream_simulation \
      "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.simulation.yml build simulation"
  elif command -v ros2 >/dev/null 2>&1; then
    log "Running experiment on host (sim must be publishing ${GPS_TOPIC})"
  else
    die "no ROS environment found. Start sim first:\n  cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"
  fi

  log "Checking GPS on ${GPS_TOPIC}..."
  if [[ -n "${RUN_CONTAINER}" ]]; then
    ros_topic_ready "${GPS_TOPIC}" 20 "${RUN_CONTAINER}" \
      || die "no GPS on ${GPS_TOPIC}. Start sim:\n  cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"
  else
    ros_topic_ready "${GPS_TOPIC}" 20 \
      || die "no GPS on ${GPS_TOPIC}. Start sim:\n  ros2 launch blueboat_sim open_water.launch.py headless:=True"
  fi

  if [[ "${SMOKE}" -eq 1 ]]; then
    RUN_ARGS+=(--warmup 5 --duration 20 --skip-initial 5)
    log "Smoke mode: shortened evaluate window"
  fi

  log "Output (host): ${OUT_DIR}"
  log "RViz (other terminal, while experiment runs):"
  log "  cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start gs --h0-boat --sim-viz"
  if [[ -n "${RUN_CONTAINER}" ]]; then
    log "Running experiment in ${RUN_CONTAINER}..."
    run_in_container "${RUN_CONTAINER}" "${RUN_ARGS[@]}"
  else
    HOST_OUT="${OUT_DIR}"
    LOCAL_ARGS=(--platform sim --out "${HOST_OUT}")
    [[ "${DRY_RUN}" -eq 1 ]] && LOCAL_ARGS+=(--dry-run)
    [[ "${NO_PLOT}" -eq 1 ]] && LOCAL_ARGS+=(--no-plot)
    LOCAL_ARGS+=("${EXTRA_ARGS[@]}")
    [[ "${SMOKE}" -eq 1 ]] && LOCAL_ARGS+=(--warmup 5 --duration 20 --skip-initial 5)
    log "Running experiment locally..."
    run_locally "${LOCAL_ARGS[@]}"
  fi

else
  GPS_TOPIC="/wamv/sensors/gps/gps/fix"

  if ! docker ps --format '{{.Names}}' | grep -qx mini_bream_frontseat; then
    die "mini_bream_frontseat not running. Start with: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi"
  fi

  if ! docker exec mini_bream_frontseat test -d "${FIELD_TESTS_CONTAINER}"; then
    die "field_tests mount missing in container. Recreate frontseat:\n  cd ${REPO_ROOT}/docker && docker compose -f docker-compose.frontseat.yml up -d frontseat"
  fi

  log "Checking deps in frontseat container..."
  check_deps mini_bream_frontseat \
    "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.frontseat.yml build frontseat"

  docker exec mini_bream_frontseat pkill -f 'ilos_boat/stack_runner.py' 2>/dev/null || true

  log "Checking GPS on ${GPS_TOPIC}..."
  ros_topic_ready "${GPS_TOPIC}" 15 mini_bream_frontseat \
    || die "no GPS fix on ${GPS_TOPIC}"

  if [[ "${PLATFORM}" == "bench" ]]; then
    log "Bench mode: real sensors, thrust_mode=log_only"
  fi
  if [[ "${SMOKE}" -eq 1 ]]; then
    RUN_ARGS+=(--warmup 8 --duration 15 --skip-initial 8)
    log "Smoke mode: shortened evaluate window"
  fi

  log "Output (host): ${OUT_DIR}"
  log "Running experiment in mini_bream_frontseat..."
  run_in_container mini_bream_frontseat "${RUN_ARGS[@]}"
fi

RESULT_JSON="${OUT_DIR}/ILOS_PID/result.json"
LOG_CSV="${OUT_DIR}/ILOS_PID/log.csv"
[[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
[[ -s "${LOG_CSV}" ]] || die "missing or empty ${LOG_CSV}"

log "Result:"
cat "${RESULT_JSON}"
LINES=$(wc -l < "${LOG_CSV}")
log "log.csv lines: ${LINES}"
[[ "${LINES}" -gt 5 ]] || die "log.csv too short"

log "ILOS experiment complete (platform=${PLATFORM})"
log "Artifacts: ${OUT_DIR}/ILOS_PID/"
if [[ "${PLATFORM}" == "sim" ]]; then
  log "RViz: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start gs --h0-boat --sim-viz"
fi
