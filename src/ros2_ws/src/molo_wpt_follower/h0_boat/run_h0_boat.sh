#!/usr/bin/env bash
# Run the H0 baseline lemniscate experiment on the real boat.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /workspace/ros2_ws/install/setup.bash
fi

exec python3 run_h0_experiment.py "$@"
