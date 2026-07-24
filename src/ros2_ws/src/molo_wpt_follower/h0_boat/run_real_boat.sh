#!/usr/bin/env bash
# Run the H0 lemniscate experiment on the real boat (Pi frontseat container).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
FIELD_TESTS_HOST="${REPO_ROOT}/field_tests/h0_boat"
FIELD_TESTS_CONTAINER="/workspace/field_tests/h0_boat"

RUN_TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="${OUT_DIR:-${FIELD_TESTS_HOST}/${RUN_TS}}"
CONTAINER_OUT="${FIELD_TESTS_CONTAINER}/$(basename "${OUT_DIR}")"

BENCH=0
SMOKE=0
NO_PLOT=0
DRY_RUN=0
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./run_real_boat.sh [options] [-- extra run_h0_experiment.py args]

Run H0 MPC on the real boat via mini_bream_frontseat. Results persist under
src/field_tests/h0_boat/<timestamp>/ on the host.

Options:
  --bench       Real GPS/IMU, log_only thrust (no motor commands)
  --smoke       Short warmup/evaluate (8s / 15s); default overlay unless --bench
  --dry-run     Write config + ref_path only
  --no-plot     Skip plot generation
  --out DIR     Host output directory (default: field_tests/h0_boat/<timestamp>)
  -h, --help    Show this help

Prerequisites:
  ./src/docker/mini_bream_env.sh start pi
  Release radio deadman so pwm_daemon accepts ROS thrust (unless --bench)
EOF
}

log() { echo "[h0-boat] $*"; }
die() { echo "[h0-boat] ERROR: $*" >&2; exit 1; }

check_mpc_deps() {
  local container="$1"
  local rebuild_hint="$2"
  if ! docker exec "${container}" python3 -c \
      'import dubins, osqp, scipy, matplotlib, utm, transforms3d' 2>/dev/null; then
    die "MPC Python deps missing in ${container}. Rebuild the image:\n  ${rebuild_hint}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bench) BENCH=1 ;;
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

mkdir -p "${OUT_DIR}"

if ! docker ps --format '{{.Names}}' | grep -qx mini_bream_frontseat; then
  die "mini_bream_frontseat not running. Start with: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi"
fi

if ! docker exec mini_bream_frontseat test -d "${FIELD_TESTS_CONTAINER}"; then
  die "field_tests mount missing in container. Recreate frontseat:\n  cd ${REPO_ROOT}/docker && docker compose -f docker-compose.frontseat.yml up -d frontseat"
fi

log "Checking MPC deps in frontseat container..."
check_mpc_deps mini_bream_frontseat \
  "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.frontseat.yml build frontseat"

docker exec mini_bream_frontseat pkill -f 'h0_boat/mock_frontseat.py' 2>/dev/null || true

log "Checking GPS on /wamv/sensors/gps/gps/fix..."
docker exec mini_bream_frontseat bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 15 ros2 topic echo /wamv/sensors/gps/gps/fix --once" \
  | grep -q latitude || die "no GPS fix on /wamv/sensors/gps/gps/fix"

OVERLAY="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat/config/h0_boat_overlay.yaml"
RUN_ARGS=(--out "${CONTAINER_OUT}")
if [[ "${BENCH}" -eq 1 ]]; then
  OVERLAY="/workspace/ros2_ws/src/molo_wpt_follower/h0_boat/config/h0_bench_overlay.yaml"
  log "Bench mode: real sensors, thrust_mode=log_only"
elif [[ "${SMOKE}" -eq 1 ]]; then
  RUN_ARGS+=(--warmup 8 --duration 15 --skip-initial 8)
  log "Smoke mode: shortened evaluate window"
fi
RUN_ARGS+=(--overlay "${OVERLAY}")
[[ "${DRY_RUN}" -eq 1 ]] && RUN_ARGS+=(--dry-run)
[[ "${NO_PLOT}" -eq 1 ]] && RUN_ARGS+=(--no-plot)
RUN_ARGS+=("${EXTRA_ARGS[@]}")

log "Output (host): ${OUT_DIR}"
log "Running experiment in mini_bream_frontseat..."
docker exec mini_bream_frontseat /workspace/docker/start_h0_boat.sh "${RUN_ARGS[@]}"

RESULT_JSON="${OUT_DIR}/H0_baseline/result.json"
LOG_CSV="${OUT_DIR}/H0_baseline/log.csv"
[[ -f "${RESULT_JSON}" ]] || die "missing ${RESULT_JSON}"
[[ -s "${LOG_CSV}" ]] || die "missing or empty ${LOG_CSV}"

log "Result:"
cat "${RESULT_JSON}"
LINES=$(wc -l < "${LOG_CSV}")
log "log.csv lines: ${LINES}"
[[ "${LINES}" -gt 5 ]] || die "log.csv too short"

log "H0 boat run complete"
log "Artifacts: ${OUT_DIR}/H0_baseline/"
log "RViz (while running): cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start gs --h0-boat"
