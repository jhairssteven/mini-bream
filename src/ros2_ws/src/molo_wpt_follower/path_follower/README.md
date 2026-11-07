# path_follower — external path interface

Thin integration layer that connects **any path provider** (Nav2, rosbag replay, test publishers) to **any follower controller** in `molo_wpt_follower`.

## Contract

| Item | Value |
| --- | --- |
| Input topic | `/plan` (`nav_msgs/Path`) from Nav2 |
| Frame | `map` |
| Controllers | `ilos`, `h0` / `mpc` |

Enable with `path.source: topic` in YAML (see `config/path_topic_overlay.yaml`). Controllers still run standalone with their existing experiment scripts when `path.source` is omitted.

## Quick start (sim + Nav2)

```bash
# Terminal 1 — simulation
cd src/docker && ./mini_bream_env.sh start sim

# Terminal 2 — autonomy container
./mini_bream_env.sh start autonomy
docker exec -it mini_bream_autonomy bash
source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
ros2 launch mission_planner molo_autonomy.launch.py platform:=blueboat_sim controller:=ilos

# RViz (fixed frame `map`, use_sim_time true)
rviz2 -d /workspace/docker/config/molo_autonomy.rviz --ros-args -p use_sim_time:=true
# Use "2D Goal Pose" to send a navigation goal; Nav2 plans to /plan, ILOS follows.
```

## Real boat

```bash
ros2 launch mission_planner molo_autonomy.launch.py platform:=blueboat controller:=ilos use_sim_time:=false
```

Requires LiDAR on `/rslidar_points` and odometry on `/odom` (TF bridge in `config/blueboat/tf_bridge.json`).

## Controller selection

| `--controller` | Node | Standalone experiment |
| --- | --- | --- |
| `ilos` | `ilos_boat/ilos_follower.py` | `ilos_boat/run_real_boat.sh` |
| `h0` / `mpc` | `mpc/mpc.py` via `h0_boat/stack_runner.py` | `h0_boat/run_real_boat.sh` |

Use `--internal-path` to keep the built-in YAML trajectory (lemniscate, etc.) for controller tuning without a planner.

## Adding a new path provider

Publish `nav_msgs/Path` on `/plan` in the `map` frame. No follower changes required if `path_topic_overlay.yaml` is merged.
