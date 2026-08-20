#!/usr/bin/env bash
# Shared helpers for H0 experiment launchers (mini_bream_autonomy).
set -euo pipefail

AUTONOMY_CONTAINER="${AUTONOMY_CONTAINER:-mini_bream_autonomy}"
AUTONOMY_OVERLAY="${AUTONOMY_OVERLAY:-/workspace/docker/config/autonomy_overlay.yaml}"
SIM_CONTAINER="${SIM_CONTAINER:-mini_bream_simulation}"
FRONTSEAT_CONTAINER="${FRONTSEAT_CONTAINER:-mini_bream_frontseat}"

container_running() {
  docker ps --format '{{.Names}}' | grep -qx "$1"
}

require_autonomy_container() {
  container_running "${AUTONOMY_CONTAINER}" || die \
    "mini_bream_autonomy not running. Start with:\n  cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start autonomy"
}

check_mpc_deps() {
  local container="$1"
  local rebuild_hint="$2"
  if ! docker exec "${container}" python3 -c \
      'import dubins, osqp, scipy, matplotlib, utm, transforms3d' 2>/dev/null; then
    die "MPC Python deps missing in ${container}. Rebuild:\n  ${rebuild_hint}"
  fi
}

check_tune_deps() {
  local container="$1"
  local rebuild_hint="$2"
  check_mpc_deps "${container}" "${rebuild_hint}"
  if ! docker exec "${container}" python3 -c 'import skopt' 2>/dev/null; then
    die "scikit-optimize missing in ${container}. Rebuild:\n  ${rebuild_hint}"
  fi
}

autonomy_rebuild_hint() {
  echo "cd ${REPO_ROOT}/docker && docker compose -f docker-compose.autonomy.yml build autonomy"
}

# When running on Jetson/dev autonomy container, thrust PWM stays on the Pi.
autonomy_overlay_args() {
  if container_running "${AUTONOMY_CONTAINER}" && [[ "${PLATFORM}" == "real" || "${PLATFORM}" == "bench" ]]; then
    echo "--overlay" "${AUTONOMY_OVERLAY}"
  fi
}

run_in_autonomy() {
  local start_script="$1"
  shift
  require_autonomy_container
  docker exec "${AUTONOMY_CONTAINER}" "${start_script}" "$@"
}
