#!/usr/bin/env bash
# Run the H0 lemniscate experiment in the current ROS environment.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# shellcheck source=ros_env.sh
source "${SCRIPT_DIR}/ros_env.sh"
h0_require_ros

exec python3 run_h0_experiment.py "$@"
