#!/usr/bin/env bash
# Run controller stack only (Ctrl+C to stop; good for RViz).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"
gs_require_ros

PLATFORM="${1:-sim}"

CFG="$(
  cd "${SCRIPT_DIR}" && python3 -c "
from config import build_gs_boat_config, prepare_run_config
from pathlib import Path
cfg = build_gs_boat_config(platform='${PLATFORM}')
run_dir = Path('${SCRIPT_DIR}') / 'results' / '_stack_live'
prepare_run_config(cfg, run_dir)
print(run_dir / 'config.yaml')
"
)"

exec python3 "${SCRIPT_DIR}/stack_runner.py" --config "${CFG}"
