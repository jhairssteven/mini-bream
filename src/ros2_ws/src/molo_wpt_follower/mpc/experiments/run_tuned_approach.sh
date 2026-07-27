#!/usr/bin/env bash
# Run one BO-tuned approach in open-water Gazebo (lemniscate from best_config.yaml).
#
# Usage:
#   ./experiments/run_tuned_approach.sh H9_slow_tight
#   ./experiments/run_tuned_approach.sh --list
#   ./experiments/run_tuned_approach.sh H1_curvature_ff --eval
#
# Prerequisite: Gazebo already running, OR set LAUNCH_GAZEBO=1 to start headless sim first.

set -euo pipefail

MPC_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TUNE_ROOT="${TUNE_ROOT:-$MPC_DIR/experiments/results/tune_lemniscate_full}"
OPEN_WATER_LAUNCH="${OPEN_WATER_LAUNCH:-/workspace/ros2_ws/install/blueboat_sim/share/blueboat_sim/launch/open_water.launch.py}"

source_ros() {
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    [ -f /workspace/codebase/vrx_ws/install/setup.bash ] && source /workspace/codebase/vrx_ws/install/setup.bash
    [ -f /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash ] && \
      source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash
  fi
}


list_hypotheses() {
  echo "Tuned configs in: $TUNE_ROOT"
  echo ""
  if [ -f "$TUNE_ROOT/tune_summary.json" ]; then
    python3 - <<PY
import json
from pathlib import Path
p = Path("$TUNE_ROOT") / "tune_summary.json"
d = json.loads(p.read_text())
for r in d.get("ranked", []):
    hid = r["hypothesis_id"]
    print(f"  {hid:<22}  val={r['validation_rmse']:.4f} m  approach={r.get('approach','')}")
PY
  else
    for d in "$TUNE_ROOT"/H*/; do
      [ -f "${d}best_config.yaml" ] && basename "$d"
    done
  fi
}

RUN_EVAL=0
LAUNCH_GAZEBO="${LAUNCH_GAZEBO:-0}"
HID=""

while [ $# -gt 0 ]; do
  case "$1" in
    --list|-l) list_hypotheses; exit 0 ;;
    --eval|-e) RUN_EVAL=1; shift ;;
    --launch-gazebo) LAUNCH_GAZEBO=1; shift ;;
    -h|--help)
      cat <<EOF
Usage: $0 <hypothesis_id> [--eval] [--launch-gazebo]

Examples:
  $0 --list
  $0 H9_slow_tight
  $0 H1_curvature_ff --eval

Environment:
  TUNE_ROOT     Path to tune results (default: tune_lemniscate_full)
  LAUNCH_GAZEBO=1  Start open_water headless before MPC

In another terminal (same ROS env):
  rviz2 -d $MPC_DIR/molo_mpc.rviz
EOF
      exit 0
      ;;
    *) HID="$1"; shift ;;
  esac
done

if [ -z "$HID" ]; then
  echo "Error: provide hypothesis id (e.g. H9_slow_tight) or --list" >&2
  exit 1
fi

CFG="$TUNE_ROOT/$HID/best_config.yaml"
if [ ! -f "$CFG" ]; then
  echo "Error: config not found: $CFG" >&2
  list_hypotheses
  exit 1
fi

source_ros
cd "$MPC_DIR"

if [ "$LAUNCH_GAZEBO" = "1" ]; then
  echo "Starting open-water Gazebo (headless)..."
  pkill -9 -f 'mpc/mpc.py' 2>/dev/null || true
  pkill -9 -f 'gz sim' 2>/dev/null || true
  sleep 2
  ros2 launch "$OPEN_WATER_LAUNCH" headless:=True &
  echo "Waiting for GPS..."
  for _ in $(seq 1 45); do
    if timeout 4 ros2 topic echo /blueboat/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude; then
      echo "Gazebo ready."
      sleep 5
      break
    fi
    sleep 2
  done
fi

echo "Running MPC: $HID"
echo "  config: $CFG"
grep -E 'approach:|type:' "$CFG" | head -5 || true
echo ""
echo "Ctrl+C to stop. In another terminal:"
echo "  rviz2 -d $MPC_DIR/molo_mpc.rviz"
echo ""

if [ "$RUN_EVAL" = "1" ]; then
  python3 mpc.py --config "$CFG" &
  MPC_PID=$!
  sleep 22
  python3 evaluate_mpc.py --duration 55 --skip-initial 20
  kill -INT "$MPC_PID" 2>/dev/null || true
  wait "$MPC_PID" 2>/dev/null || true
else
  exec python3 mpc.py --config "$CFG"
fi
