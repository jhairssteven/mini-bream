# MPC trajectory follower (WAMV-16)

Pluggable guidance (`control_law.py`) + velocity MPC (default) or spatial MPC (`mpc.mode: spatial`).

## Architecture

- **Guidance**: ILOS + curvature feedforward / Stanley / pure pursuit (`control.approach`)
- **Control**: velocity MPC on `[u, v, r]` (default) or spatial MPC on `[x,y,ψ,u,v,r]`
- **Paths**: lemniscate, circle, rectangle, triangle, line, sine (`trajectories.yaml`)
- **Experiments**: hypothesis → test → log/plot → refine (`experiments/`)

See `experiments/README.md` for the scientific campaign workflow.

**Full technical report (LaTeX/PDF):** `report/main.tex` — build with `cd report && make pdf`.

## Open-water world

```bash
source /opt/ros/humble/setup.bash
source /workspace/codebase/vrx_ws/install/setup.bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash

ros2 launch blueboat_sim open_water.launch.py headless:=True
```

World file: `open_water_harner.sdf` (no shore/dock collisions). Rebuild after edits: `colcon build --packages-select blueboat_sim`.

## Run

```bash
cd .../molo_wpt_follower/mpc
python3 mpc.py
python3 wind_viz.py   # wind arrow field from /vrx/debug/wind/*
rviz2 -d molo_mpc.rviz
```

Set trajectory in `params.yaml` under `waypoints.trajectory` (or use `points:` polyline).

## Single-path evaluate

```bash
python3 evaluate_mpc.py --duration 60 --skip-initial 15
```

## Multi-trajectory robustness benchmark

Runs six paths in open-water Gazebo (restarts sim per trajectory):

```bash
python3 evaluate_trajectories.py
python3 evaluate_trajectories.py --names circle sine line_east
```

Suite definition: `trajectories.yaml`. Results: `trajectory_eval_results.json`.  
Pass criterion: XTE RMSE **< 0.1 m** (configurable via `target_rmse_m`).

### Latest clean benchmark (`benchmark_clean_v2`, circle + lemniscate)

| Hypothesis | Circle RMSE | Lemniscate RMSE | Mean |
|------------|-------------|-----------------|------|
| **H1b_tuned_ff** | **0.217 m** | 0.532 m | 0.375 |
| **H1_curvature_ff** | **0.223 m** | **0.504 m** | 0.363 |
| H9_slow_tight | 0.225 m | **0.447 m** | **0.336** |

**None pass < 0.1 m.** H1 remains best general-purpose; H1b best on circle.

```bash
python3 experiments/summarize_results.py experiments/results/benchmark_clean_v2
```

## Tune

```bash
python3 tune_mpc.py --n-calls 12 --duration 55
```

## Trajectory types (`waypoints.trajectory.type`)

- `lemniscate` — `scale_m`, `num_points`
- `circle` — `radius_m`, `num_points`
- `rectangle` — `width_m`, `height_m`, `n_per_edge`
- `triangle` — `size_m`
- `line` — `length_m`
- `sine` — `length_m`, `amplitude_m`, `waves`
- `points` — `vertices: [[x,y], ...]`, optional `closed: true`

Set `path.closed: true` for loops; `path.smooth_dubins: true` rounds corners via Dubins.
