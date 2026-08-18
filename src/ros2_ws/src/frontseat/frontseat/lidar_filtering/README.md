# LiDAR filtering pipeline

Three sequential nodes clean the LiDAR cloud before other modules consume
it. Raw points stay on `/rslidar_points`. The pipeline product is
`/rslidar_points/filtered` (`boat_hal` `lidar_points_filtered`).

```
rslidar_sdk
    │  /rslidar_points
    ▼
self_filter          drop hull + mast volumes
    │  /rslidar_points/self_filtered
    ▼
waterline_ransac     strip near-horizontal water / leftover planes
    │  /rslidar_points/waterline_removed
    ▼
pc_clustering        drop sparse spray (DBSCAN size gate)
    │  /rslidar_points/filtered
    ▼
other modules        (costmap, HAL, planning, …)
```

Published clouds keep the sensor `frame_id` (`rslidar`). Geometry that needs
the boat frame (`base_link`) is handled inside each node via TF.

## Stages

| Stage | What it removes | Details |
|-------|-----------------|--------|
| Self-filter | Boat hull and 25×25 mm sensor masts | [self_filter README](../../config/lidar_filtering/self_filter/README.md) |
| Waterline RANSAC | Near-horizontal planes (water first, then leftovers) | [ransac/README.md](ransac/README.md) |
| DBSCAN clustering | Sparse noise / spray inside a 4 m XY cylinder | [pc_clustering/README.md](pc_clustering/README.md) |

Run a single stage with `self_filter.launch.py`, `waterline_ransac.launch.py`,
or `pc_clustering.launch.py` when debugging. Topic names in the YAML already
assume the full chain.

## Topics

| Topic | Role |
|-------|------|
| `/rslidar_points` | Raw LiDAR |
| `/rslidar_points/self_filtered` | After hull/mast filter |
| `/rslidar_points/self_removed` | Debug: points inside filter volumes |
| `/rslidar_points/waterline_colored` | Debug: RGB by fitted plane |
| `/rslidar_points/waterline_removed` | After all RANSAC inliers are stripped |
| `/rslidar_points/filtered` | DBSCAN clustering and pipeline output for other modules |
| `/rslidar_points/clustered_removed` | Debug: clustering noise / rejected clusters |

Configs: `config/lidar_filtering/{self_filter,ransac,pc_clustering}/`.

## Launch

```bash
ros2 launch frontseat lidar_filtering.launch.py
ros2 launch frontseat lidar_filtering.launch.py use_sim_time:=true
ros2 launch frontseat lidar_filtering.launch.py use_sim_time:=true debug_enabled:=true
```

Bag replay (autonomy container), after TF + LiDAR are playing:

```bash
ros2 launch frontseat lidar_filtering.launch.py use_sim_time:=true debug_enabled:=true
ros2 bag play /workspace/field_tests/rosbags/<bag> \
  --clock --topics /rslidar_points /tf /tf_static
```
