#!/usr/bin/env bash
# Run the ILOS controller stack only (no scoring). For RViz debugging — Ctrl+C to stop.
#
# Usage:
#   ./run_stack.sh --platform sim
#   ./run_stack.sh --platform real
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"

PLATFORM="sim"

usage() {
  cat <<'EOF'
Usage: ./run_stack.sh [--platform sim|real|bench] [--skip-checks]

Starts ilos_boat/stack_runner.py in the current ROS environment until Ctrl+C.
Requires the platform sensor topics to already be on the graph.
EOF
}

SKIP_CHECKS=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --platform)
      shift
      PLATFORM="${1:?missing value for --platform}"
      ;;
    --skip-checks) SKIP_CHECKS=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

case "${PLATFORM}" in
  sim|real|bench) ;;
  *) echo "unknown platform: ${PLATFORM}" >&2; usage; exit 1 ;;
esac

ilos_require_ros || exit 1

topic_on_graph() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    topic_on_graph "${topic}" && return 0
    sleep 0.5
  done
  return 1
}

if [[ "${SKIP_CHECKS}" -eq 0 ]]; then
  if [[ "${PLATFORM}" == "sim" ]]; then
    wait_for_topic "/blueboat/sensors/gps/gps/fix" 20 \
      || { echo "[ilos-boat] ERROR: missing /blueboat/sensors/gps/gps/fix" >&2; exit 1; }
  else
    wait_for_topic "/wamv/sensors/gps/gps/fix" 20 \
      || { echo "[ilos-boat] ERROR: missing /wamv/sensors/gps/gps/fix" >&2; exit 1; }
  fi
fi

CFG_DIR="${SCRIPT_DIR}/results/stack_run"
mkdir -p "${CFG_DIR}"
python3 - <<PY
from pathlib import Path
import sys
sys.path.insert(0, "${SCRIPT_DIR}")
from config import build_ilos_boat_config, prepare_run_config
cfg = build_ilos_boat_config(platform="${PLATFORM}")
prepare_run_config(cfg, Path("${CFG_DIR}"))
PY

echo "[ilos-boat] Starting stack-only (platform=${PLATFORM}). Ctrl+C to stop."
cd "${SCRIPT_DIR}"
exec python3 stack_runner.py --config "${CFG_DIR}/config.yaml"
