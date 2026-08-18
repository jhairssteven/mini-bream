# DBSCAN clustering

Drops sparse LiDAR returns by clustering with DBSCAN and removing points that
land in clusters smaller than `min_cluster_size`. The filtered cloud is the
points that remain. There is no colored cluster cloud; RViz debug uses
axis-aligned bounding boxes.

Published clouds keep the input `frame_id` (i.e: `rslidar`).

## Pipeline

Each cloud is processed as:

1. Read XYZ (skip non-finite points).
2. Optional XY radius gate (`max_radius`, default 4 m in `radius_frame`).
   Points outside the cylinder skip DBSCAN and stay in the output.
   `max_radius <= 0` clusters the whole cloud.
3. Optional voxel downsample (`voxel_size`); DBSCAN runs on voxel centroids.
4. DBSCAN with `eps` / `min_samples`. Label `-1` is noise.
5. Labels are mapped back to the original points when voxelizing.
6. Size gate: clusters with fewer than `min_cluster_size` points (and, if set,
   more than `max_cluster_size`) become noise.
7. **Filtered cloud** = in-radius kept clusters plus all out-of-radius points.
8. **Removed cloud** (debug) = in-radius noise + rejected clusters.
9. **Markers** (debug) = AABB cube + wireframe + text per kept cluster.

```
input cloud
    │
    XY radius gate (max_radius; <=0 = whole cloud)
    │
    in-radius ── optional voxel ── DBSCAN ── size gate
    │                                    │
    │                                    ├─ noise / small ──► *_removed
    │                                    └─ kept clusters ──► output + AABB
    │
    out-of-radius ──────────────────────────────────────────► output (passthrough)
```

Chain after self-filter or waterline RANSAC by pointing `input_topic` at
`/rslidar_points/filtered` or `/rslidar_points/waterline_removed`.

## Files

| File | Role |
|------|------|
| `dbscan.py` | DBSCAN + size gate + AABB boxes |
| `PcClusteringNode.py` | ROS 2 node (filtered cloud, optional debug) |
| `config/pc_clustering/clustering.yaml` | Topics and knobs |
| `launch/pc_clustering.launch.py` | Node only; play bags separately |

## Topics

Defaults below match `clustering.yaml`. Change the YAML if you chain nodes.

| Direction | Topic | Contents |
|-----------|-------|----------|
| Input | `input_topic` | XYZ (optional intensity) PointCloud2 |
| Output | `output_topic` | Input minus small/noisy clusters |
| Debug | `removed_topic` | Points dropped by the size gate / noise |
| Debug | `markers_topic` | AABB boxes + `id=… n=…` labels |

Set `debug.enabled: false` (or omit `debug_enabled:=true` at launch) to skip
marker and removed-cloud publishing.

## Launch

```bash
ros2 launch frontseat pc_clustering.launch.py use_sim_time:=true
ros2 launch frontseat pc_clustering.launch.py use_sim_time:=true debug_enabled:=true
```

YAML is copied into the package share on `colcon build`. Restart the node after
edits. Optional `config_path:=/path/to/clustering.yaml` skips the share copy.

Bag replay (autonomy container):

```bash
ros2 launch frontseat pc_clustering.launch.py use_sim_time:=true debug_enabled:=true
ros2 bag play /workspace/field_tests/rosbags/jul_29_2026_lidar/rosbag2_1970_01_01-00_24_06 \
  --clock --topics /rslidar_points /tf /tf_static
```

## YAML knobs

| Key | Meaning |
|-----|---------|
| `max_radius` | XY cylinder in `radius_frame`; `<=0` clusters the whole cloud |
| `radius_frame` | Frame for the cylinder (`base_link`) |
| `cluster.eps` | DBSCAN neighborhood radius, meters |
| `cluster.min_samples` | Core-point density (includes the point itself). `1` merges everything within `eps` into one cluster |
| `cluster.min_cluster_size` | Drop clusters smaller than this |
| `cluster.max_cluster_size` | Drop clusters larger than this; `<=0` disables |
| `cluster.voxel_size` | Voxel size for DBSCAN; labels map back; `<=0` disables |
| `debug.enabled` | Master switch for debug topics |
| `debug.publish_markers` | AABB MarkerArray |
| `debug.publish_removed_cloud` | Removed-points PointCloud2 |

`debug_enabled:=true` at launch ORs with YAML `debug.enabled`.

## Library

`cluster_points(points, params=ClusterParams(...))` returns a `ClusterResult`
with `labels`, `keep_mask`, and `boxes`. `dbscan_labels` is the raw DBSCAN
step (no size gate). scikit-learn is used when installed (`python3-sklearn`);
otherwise a SciPy KD-tree fallback runs (slower on dense LiDAR).
