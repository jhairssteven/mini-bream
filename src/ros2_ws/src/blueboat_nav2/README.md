# blueboat_nav2

Nav2-based local planning for BlueBoat: LiDAR → costmap → Smac hybrid (Dubins) planner → `/plan`.

Low-level control is **not** handled by Nav2. The ILOS or H0 follower in `molo_wpt_follower` subscribes to `/plan`.

## Stack

```
/rslidar_points
  → pointcloud_to_laserscan → /scan
  → Nav2 global costmap (rolling, 60×60 m @ 0.25 m)
  → SmacPlannerHybrid (Dubins, min turn radius 3 m, footprint 1.3×0.66 m)
  → goal_path_planner (RViz /goal_pose → ComputePathToPose)
  → /plan
  → ILOS / H0 follower
```

## Launch

```bash
# Planning only
ros2 launch blueboat_nav2 nav2_planning.launch.py platform:=blueboat_sim

# Full autonomy (TF bridge + Nav2 + follower)
ros2 launch mission_planner molo_autonomy.launch.py platform:=blueboat_sim controller:=ilos
```

| Platform | `platform` arg | Base link frame | Odom |
|----------|----------------|-----------------|------|
| Simulation | `blueboat_sim` | `blueboat/base_link` | Gazebo GT odom (via TF bridge) |
| Real boat | `blueboat` | `blueboat/blueboat/base_link` | `/molo_boat/estimated_odometry` |

## Goals

In RViz, use **2D Goal Pose** (publishes `/goal_pose`). The planner replans at 1 Hz while a goal is active.

## Parameters

- `config/nav2_blueboat_sim.yaml` — simulation
- `config/nav2_blueboat_real.yaml` — field / real boat

Footprint matches hull envelope (~1.3 m × 0.66 m). Adjust `minimum_turning_radius` for your operating speed.

## Dependencies

Installed via `src/docker/install_ros2_ws_deps.sh` (`ros-humble-navigation2`, `nav2-smac-planner`, `pointcloud-to-laserscan`, etc.).
