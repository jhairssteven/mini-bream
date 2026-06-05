# Manual test: one tuned approach at a time (lemniscate, open water)

Use the **BO-tuned** configs from `experiments/results/tune_lemniscate_full/<H_id>/best_config.yaml`.

## 1. Environment (Docker `linc_project` or host with ROS)

```bash
source /opt/ros/humble/setup.bash
source /workspace/codebase/vrx_ws/install/setup.bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash

cd /workspace/codebase/mini-bream/src/ros2_ws/src/molo_wpt_follower/mpc
```

On **host** (paths may differ), use your install path to `open_water.launch.py` and the `mpc` folder.

## 2. Terminal A — Gazebo (open water)

```bash
ros2 launch /workspace/codebase/mini-bream/src/ros2_ws/install/linc_gz/share/linc_gz/launch/open_water.launch.py
```

Add `headless:=True` if you do not need the GUI.

Wait until the WAM-V is spawned and topics are live (`/wamv/sensors/gps/gps/fix`).

## 3. Terminal B — RViz (optional)

```bash
source ...  # same as above
cd .../molo_wpt_follower/mpc
rviz2 -d molo_mpc.rviz
python3 wind_viz.py   # vizualize wind arrow field from /vrx/debug/wind/*
```

You should see reference path, traversed path, vehicle pose, and cross-track error.

## 4. Terminal C — MPC for one approach

### Option A: helper script

```bash
chmod +x experiments/run_tuned_approach.sh

# List all tuned hypotheses (ranked by validation RMSE)
./experiments/run_tuned_approach.sh --list

# Run best on lemniscate (H9)
./experiments/run_tuned_approach.sh H9_slow_tight

# Run + print SCORE after 55 s
./experiments/run_tuned_approach.sh H9_slow_tight --eval

# Start Gazebo from the same script (single terminal)
LAUNCH_GAZEBO=1 ./experiments/run_tuned_approach.sh H1_curvature_ff
```

### Option B: direct `mpc.py`

```bash
python3 mpc.py --config experiments/results/tune_lemniscate_full/H9_slow_tight/best_config.yaml
```

Replace `H9_slow_tight` with any hypothesis folder name.

## 5. All 15 approaches (copy-paste)

| Hypothesis | Command |
|------------|---------|
| H9_slow_tight (best) | `python3 mpc.py --config experiments/results/tune_lemniscate_full/H9_slow_tight/best_config.yaml` |
| H1b_tuned_ff | `.../H1b_tuned_ff/best_config.yaml` |
| H0_baseline | `.../H0_baseline/best_config.yaml` |
| H10_lateral_vel | `.../H10_lateral_vel/best_config.yaml` |
| H1_curvature_ff | `.../H1_curvature_ff/best_config.yaml` |
| H5_geometric_ff | `.../H5_geometric_ff/best_config.yaml` |
| H3_aggressive_ff | `.../H3_aggressive_ff/best_config.yaml` |
| H2_stanley | `.../H2_stanley/best_config.yaml` |
| H12_smc_ilos | `.../H12_smc_ilos/best_config.yaml` |
| H8_pure_pursuit | `.../H8_pure_pursuit/best_config.yaml` |
| H14_frenet_ff | `.../H14_frenet_ff/best_config.yaml` |
| H13_smc_geometric | `.../H13_smc_geometric/best_config.yaml` |
| H16_contouring | `.../H16_contouring/best_config.yaml` |
| H15_backstepping | `.../H15_backstepping/best_config.yaml` |
| H11_spatial_mpc | `.../H11_spatial_mpc/best_config.yaml` |

**Between approaches:** stop MPC (`Ctrl+C`), start the next config. If the boat drifts badly, restart Gazebo:

```bash
pkill -9 -f 'mpc/mpc.py'; pkill -9 -f 'gz sim'; sleep 3
# relaunch open_water.launch.py
```

## 6. Measure XTE (optional, separate terminal)

While MPC is running:

```bash
python3 evaluate_mpc.py --duration 55 --skip-initial 20
```

Prints `SCORE <rmse>` when done (same metric as experiments).

## 7. What each config contains

- **Trajectory:** lemniscate (`scale_m: 7`, from tuning)
- **Approach:** `control.approach` (e.g. `curvature_ff_ilos`, `stanley`, `smc_ilos`)
- **MPC mode:** `velocity` (default) or `spatial` for `H11_spatial_mpc`
- **Tuned:** path, Dubins, ILOS, PID, guidance, MPC weights

Validation plots from tuning:  
`experiments/results/tune_lemniscate_full/<H_id>/best_validation/plots.png`

## 8. Rankings (lemniscate validation RMSE)

See `experiments/results/tune_lemniscate_full/tune_summary.json` — best: **H9_slow_tight** (~0.109 m).
