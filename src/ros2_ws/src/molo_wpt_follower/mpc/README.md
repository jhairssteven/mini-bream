# MPC trajectory follower (WAMV-16)

ILOS path guidance + velocity MPC thrust allocation (Fossen 3-DOF hydrodynamics from LINC `NominalMPC`).

## Architecture

- **Guidance**: ILOS (`algorithms.py`) — cross-track error and desired heading
- **Control**: velocity MPC on `[u, v, r]` with inputs `[T_left, T_right]` (N)
- **Paths**: lemniscate, circle, rectangle, triangle, line, sine (see `trajectories.yaml`)

Spatial pose MPC (`spatial_mpc.py`) is kept for reference; `mpc.py` uses `velocity_mpc.py`.

## Open-water world

```bash
source /opt/ros/humble/setup.bash
source /workspace/codebase/vrx_ws/install/setup.bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash

ros2 launch /workspace/codebase/mini-bream/src/ros2_ws/install/linc_gz/share/linc_gz/launch/open_water.launch.py headless:=True
```

World file: `open_water_harner.sdf` (no shore/dock collisions). Rebuild after edits: `colcon build --packages-select linc_gz`.

## Run

```bash
cd .../molo_wpt_follower/mpc
python3 mpc.py
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

### Latest open-water results (GT pose, `use_ground_truth_pose: true`)

| Trajectory   | XTE RMSE (m) | Pass (< 0.1 m) |
|-------------|--------------|----------------|
| line_east   | ~0.002       | yes            |
| circle      | ~0.29–0.34   | no             |
| sine        | ~0.33        | no             |
| lemniscate  | ~0.37–0.50   | no             |
| rectangle   | ~0.59        | no             |
| triangle    | ~0.73–0.94   | no             |

Straight and gentle curves meet the target; tight corners need slower speeds or larger Dubins radius (per-trajectory overrides in `trajectories.yaml`).

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
