#!/usr/bin/env bash
# Run the ILOS controller stack only (no scoring). For RViz debugging — Ctrl+C to stop.
#
# Usage:
#   ./run_stack.sh --platform sim
#   ./run_stack.sh --platform real
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
PLATFORM="sim"

usage() {
  cat <<'EOF'
Usage: ./run_stack.sh [--platform sim|real|bench]

Starts ilos_boat/stack_runner.py and keeps it running until Ctrl+C.
Pair with RViz in another terminal:

  cd src/docker
  ./mini_bream_env.sh start gs --h0-boat --sim-viz   # sim
  ./mini_bream_env.sh start gs --h0-boat               # real boat
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --platform)
      shift
      PLATFORM="${1:?missing value for --platform}"
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

STACK_DIR="/workspace/ros2_ws/src/molo_wpt_follower/ilos_boat"
CFG_DIR="${SCRIPT_DIR}/results/stack_run"
CFG_FILE="${CFG_DIR}/config.yaml"

run_in_sim() {
  docker exec -it mini_bream_simulation bash -lc "
    source /opt/ros/humble/setup.bash
    source /workspace/ros2_ws/install/setup.bash
    mkdir -p ${STACK_DIR}/results/stack_run
    python3 - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, '${STACK_DIR}')
from config import build_ilos_boat_config, prepare_run_config
cfg = build_ilos_boat_config(platform='${PLATFORM}')
prepare_run_config(cfg, Path('${STACK_DIR}/results/stack_run'))
PY
    cd ${STACK_DIR}
    exec python3 stack_runner.py --config ${STACK_DIR}/results/stack_run/config.yaml
  "
}

run_on_frontseat() {
  docker exec -it mini_bream_frontseat bash -lc "
    source /opt/ros/humble/setup.bash
    source /workspace/ros2_ws/install/setup.bash
    mkdir -p ${STACK_DIR}/results/stack_run
    python3 - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, '${STACK_DIR}')
from config import build_ilos_boat_config, prepare_run_config
cfg = build_ilos_boat_config(platform='${PLATFORM}')
prepare_run_config(cfg, Path('${STACK_DIR}/results/stack_run'))
PY
    cd ${STACK_DIR}
    exec python3 stack_runner.py --config ${STACK_DIR}/results/stack_run/config.yaml
  "
}

echo "[ilos-boat] Starting stack-only (platform=${PLATFORM}). Ctrl+C to stop."
echo "[ilos-boat] RViz (other terminal): cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start gs --h0-boat$([[ \"${PLATFORM}\" == sim ]] && echo ' --sim-viz')"

case "${PLATFORM}" in
  sim)
    docker ps --format '{{.Names}}' | grep -qx mini_bream_simulation \
      || { echo "Start sim first: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start sim" >&2; exit 1; }
    run_in_sim
    ;;
  real|bench)
    docker ps --format '{{.Names}}' | grep -qx mini_bream_frontseat \
      || { echo "Start frontseat first: cd ${REPO_ROOT}/docker && ./mini_bream_env.sh start pi" >&2; exit 1; }
    run_on_frontseat
    ;;
  *) echo "unknown platform: ${PLATFORM}" >&2; exit 1 ;;
esac
