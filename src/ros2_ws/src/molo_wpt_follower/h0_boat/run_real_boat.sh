#!/usr/bin/env bash
# H0 lemniscate experiment launcher (all platforms).
#
# The experiment is identical across platforms; only the prerequisite stack differs:
#   --platform sim   → blueboat_sim (Gazebo)
#   --platform real  → Pi sensors + autonomy container (default)
#   --platform bench → Pi sensors, log_only thrust on autonomy overlay
#
# Examples:
#   ./run_real_boat.sh --platform sim
#   ./run_real_boat.sh --platform real
#   ./run_real_boat.sh --platform sim --tune
#   ./run_real_boat.sh --platform real --tune --install
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
FIELD_TESTS_HOST="${REPO_ROOT}/field_tests/h0_boat"
FIELD_TESTS_CONTAINER="/workspace/field_tests/h0_boat"

PLATFORM="real"
RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUT_DIR:-${FIELD_TESTS_HOST}/${RUN_TS}}"
CONTAINER_OUT="${FIELD_TESTS_CONTAINER}/$(basename "${OUT_DIR}")"

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
Usage: ./run_real_boat.sh [options] [-- extra run_h0_experiment.py args]

Run the H0 lemniscate MPC experiment. Start stacks first:

  --platform sim    sim + autonomy: cd src/docker && ./mini_bream_env.sh start sim
                    (other terminal) ./mini_bream_env.sh start autonomy --build
  --platform real   pi + autonomy: ./mini_bream_env.sh start pi
                    (Jetson/dev)    ./mini_bream_env.sh start autonomy
  --platform bench  frontseat, motors idle (same as --bench)

Options:
  --platform P  Platform profile: sim, real, bench (default: real)
  --bench       Shorthand for --platform bench
  --tune        Run Bayesian optimization (tune_h0.py) instead of a single experiment
  --install     With --tune: write best params to config/h0_tuned_overlay.yaml
  --quick       With --tune: 8 BO calls; otherwise shortened evaluate window
  --smoke       Short warmup/evaluate window (experiment only)
  --dry-run     Write config + ref_path only
  --no-plot     Skip plot generation
  --out DIR     Output directory
  -h, --help    Show this help

Mock integration test (Pi→Jetson): ./run_mock_test.sh
EOF
}

log() { echo "[h0-boat] $*"; }
die() { echo "[h0-boat] ERROR: $*" >&2; exit 1; }

# shellcheck source=../../../../docker/experiment_common.sh
source "${REPO_ROOT}/docker/experiment_common.sh"
AUTONOMY_REBUILD_HINT="$(autonomy_rebuild_hint)"

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
  shift  # legacy: first arg was container name
  require_autonomy_container
  docker exec "${AUTONOMY_CONTAINER}" /workspace/docker/start_h0_boat.sh "$@"
}

run_locally() {
  cd "${SCRIPT_DIR}"
  ./run_h0_boat.sh "$@"
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
      CONTAINER_OUT="${FIELD_TESTS_CONTAINER}/$(basename "${OUT_DIR}")"
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

TUNE_OUT="${OUT_DIR}"
if [[ "${TUNE}" -eq 1 ]]; then
  TUNE_OUT="${OUT_DIR:-${SCRIPT_DIR}/results/tune_${PLATFORM}_${RUN_TS}}"
  mkdir -p "${TUNE_OUT}"
  TUNE_CMD=(python3 "${SCRIPT_DIR}/tune_h0.py" --platform "${PLATFORM}" --out "${TUNE_OUT}")
  TUNE_CMD+=("${TUNE_ARGS[@]}")
  [[ "${INSTALL_TUNED}" -eq 1 ]] && TUNE_CMD+=(--install)
  TUNE_CMD+=("${EXTRA_ARGS[@]}")

  if [[ "${PLATFORM}" == "sim" ]]; then
    container_running "${SIM_CONTAINER}" || die "mini_bream_simulation not running. Start: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"
    require_autonomy_container
    log "Running H0 Bayesian tuning in ${AUTONOMY_CONTAINER}..."
    check_tune_deps "${AUTONOMY_CONTAINER}" "${AUTONOMY_REBUILD_HINT}"
    CONTAINER_TUNE_OUT="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat/results/$(basename "${TUNE_OUT}")"
    docker exec "${AUTONOMY_CONTAINER}" mkdir -p "$(dirname "${CONTAINER_TUNE_OUT}")"
    docker exec "${AUTONOMY_CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash 2>/dev/null; \
       python3 /workspace/ros2_ws/src/molo_wpt_follower/h0_boat/tune_h0.py \
       --platform sim --out ${CONTAINER_TUNE_OUT} \
       ${TUNE_ARGS[*]+"${TUNE_ARGS[*]}"} \
       ${INSTALL_TUNED:+--install} \
       ${EXTRA_ARGS[*]+"${EXTRA_ARGS[*]}"}"
    TUNE_OUT="${SCRIPT_DIR}/results/$(basename "${TUNE_OUT}")"
  else
    container_running "${FRONTSEAT_CONTAINER}" || die "mini_bream_frontseat not running. Start: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi"
    require_autonomy_container
    check_tune_deps "${AUTONOMY_CONTAINER}" "${AUTONOMY_REBUILD_HINT}"
    CONTAINER_TUNE_OUT="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat/results/$(basename "${TUNE_OUT}")"
    log "Running H0 field tuning in ${AUTONOMY_CONTAINER}..."
    docker exec "${AUTONOMY_CONTAINER}" bash -lc \
      "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash 2>/dev/null; \
       python3 /workspace/ros2_ws/src/molo_wpt_follower/h0_boat/tune_h0.py \
       --platform ${PLATFORM} --out ${CONTAINER_TUNE_OUT} \
       --overlay ${AUTONOMY_OVERLAY} \
       ${TUNE_ARGS[*]+"${TUNE_ARGS[*]}"} \
       ${INSTALL_TUNED:+--install} \
       ${EXTRA_ARGS[*]+"${EXTRA_ARGS[*]}"}"
    TUNE_OUT="${SCRIPT_DIR}/results/$(basename "${TUNE_OUT}")"
  fi

  RESULT_JSON="${TUNE_OUT}/tune_result.json"
  [[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
  log "Tuning result:"
  cat "${RESULT_JSON}"
  log "H0 tuning complete"
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
  container_running "${SIM_CONTAINER}" || die "mini_bream_simulation not running. Start: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"
  require_autonomy_container
  log "Using ${AUTONOMY_CONTAINER} (sim: ${SIM_CONTAINER})"
  check_mpc_deps "${AUTONOMY_CONTAINER}" "${AUTONOMY_REBUILD_HINT}"

  log "Checking GPS on ${GPS_TOPIC}..."
  ros_topic_ready "${GPS_TOPIC}" 20 "${SIM_CONTAINER}" \
    || die "no GPS on ${GPS_TOPIC}. Start sim: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim"

  if [[ "${SMOKE}" -eq 1 ]]; then
    RUN_ARGS+=(--warmup 5 --duration 20 --skip-initial 5)
    log "Smoke mode: shortened evaluate window"
  fi

  log "Output (host): ${OUT_DIR}"
  log "Running experiment in ${AUTONOMY_CONTAINER}..."
  run_in_container ignored "${RUN_ARGS[@]}"

else
  GPS_TOPIC="/wamv/sensors/gps/gps/fix"

  container_running "${FRONTSEAT_CONTAINER}" || die "mini_bream_frontseat not running. Start: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi"
  require_autonomy_container

  if ! docker exec "${AUTONOMY_CONTAINER}" test -d "${FIELD_TESTS_CONTAINER}"; then
    die "field_tests mount missing in ${AUTONOMY_CONTAINER}. Recreate autonomy container."
  fi

  log "Checking MPC deps in ${AUTONOMY_CONTAINER}..."
  check_mpc_deps "${AUTONOMY_CONTAINER}" "${AUTONOMY_REBUILD_HINT}"

  docker exec "${AUTONOMY_CONTAINER}" pkill -f 'h0_boat/mock_frontseat.py' 2>/dev/null || true

  log "Checking GPS on ${GPS_TOPIC}..."
  ros_topic_ready "${GPS_TOPIC}" 15 "${AUTONOMY_CONTAINER}" \
    || die "no GPS fix on ${GPS_TOPIC}"

  RUN_ARGS+=(--overlay "${AUTONOMY_OVERLAY}")

  if [[ "${PLATFORM}" == "bench" ]]; then
    log "Bench mode: real sensors, thrust_mode=log_only"
  fi
  if [[ "${SMOKE}" -eq 1 ]]; then
    RUN_ARGS+=(--warmup 8 --duration 15 --skip-initial 8)
    log "Smoke mode: shortened evaluate window"
  fi

  log "Output (host): ${OUT_DIR}"
  log "Running experiment in ${AUTONOMY_CONTAINER}..."
  run_in_container ignored "${RUN_ARGS[@]}"
fi

RESULT_JSON="${OUT_DIR}/H0_baseline/result.json"
LOG_CSV="${OUT_DIR}/H0_baseline/log.csv"
[[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
[[ -s "${LOG_CSV}" ]] || die "missing or empty ${LOG_CSV}"

log "Result:"
cat "${RESULT_JSON}"
LINES=$(wc -l < "${LOG_CSV}")
log "log.csv lines: ${LINES}"
[[ "${LINES}" -gt 5 ]] || die "log.csv too short"

log "H0 experiment complete (platform=${PLATFORM})"
log "Artifacts: ${OUT_DIR}/H0_baseline/"
if [[ "${PLATFORM}" == "real" ]]; then
  log "RViz (while running): cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start gs --h0-boat"
fi
