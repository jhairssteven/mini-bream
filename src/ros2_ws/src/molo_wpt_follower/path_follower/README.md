# path_follower — external path interface

Thin integration layer that connects **any path provider** (mission planner, rosbag replay, test publishers) to **any follower controller** in `molo_wpt_follower`.

## Contract

| Item | Value |
| --- | --- |
| Input topic | `/molo_mpc/reference_path` (`nav_msgs/Path`) |
| Frame | `map` (matches BlueBoat sim ground-truth odometry) |
| Controllers | `ilos`, `h0` / `mpc` |

Enable with `path.source: topic` in YAML (see `config/path_topic_overlay.yaml`). Controllers still run standalone with their existing experiment scripts when `path.source` is omitted.

## Quick start (sim + external planner path)

```bash
# Terminal 1 — simulation
ros2 launch blueboat_sim lidar_obstacle_course.launch.py headless:=True

# Terminal 2 — local planning (costmap + RRT*)
ros2 launch mission_planner molo_planning.launch.py platform:=blueboat_sim

# Terminal 3 — path follower (ILOS example)
cd src/ros2_ws/src/molo_wpt_follower/path_follower
python3 stack_runner.py --controller ilos --platform sim

# RViz (sim: fixed frame `map`, use_sim_time true)
rviz2 -d src/docker/config/molo_autonomy.rviz --ros-args -p use_sim_time:=true

# Or H0 MPC:
python3 stack_runner.py --controller h0 --platform sim
```

```bash
# RViz (sim: fixed frame map, use_sim_time true)
rviz2 -d src/docker/config/molo_autonomy.rviz --ros-args -p use_sim_time:=true
```

## Controller selection

| `--controller` | Node | Standalone experiment |
| --- | --- | --- |
| `ilos` | `ilos_boat/ilos_follower.py` | `ilos_boat/run_real_boat.sh` |
| `h0` / `mpc` | `mpc/mpc.py` via `h0_boat/stack_runner.py` | `h0_boat/run_real_boat.sh` |

Use `--internal-path` to keep the built-in YAML trajectory (lemniscate, etc.) for controller tuning without a planner.

## Adding a new controller

1. Implement a follower node that uses `PathActivationManager` from `mpc/path_activation.py`.
2. Register the controller name in `path_follower/config.py` (`CONTROLLERS` dict).
3. Add a branch in `stack_runner.py` if the new controller needs a custom process layout.

## Adding a new path provider

Publish `nav_msgs/Path` on `/molo_mpc/reference_path` in the `map` frame. No follower changes required.
