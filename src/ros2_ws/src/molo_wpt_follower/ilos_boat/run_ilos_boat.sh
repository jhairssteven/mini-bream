#!/usr/bin/env bash
# Run the ILOS+PID lemniscate experiment in the current ROS environment.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"
ilos_require_ros

exec python3 run_ilos_experiment.py "$@"
