# Waterline RANSAC

Sequential RANSAC for near-horizontal planes in a LiDAR cloud. Plane 1 is for 
the water surface. Later planes are extra near-horizontal surfaces fit on
whatever points remain after earlier inliers are stripped.

The fit frame is REP-103 `+Z` up (`base_link`). Published clouds keep the input
`frame_id` (i.e: `rslidar`).

## Pipeline

Each cloud is processed as:

1. Transform points into `fit_frame`.
2. Keep candidates with `z_min <= z <= z_max` and XY range `<= max_radius`
   from the `base_link` origin. Points outside that cylinder never enter RANSAC
   and stay orange on the colored cloud.
3. **Pass 1** — fit plane 1 (water) with `ransac.first` on the candidate cloud.
4. **Pass k** — fit plane *k* with `ransac.second` / `ransac.third` / … on the
   **leftover** candidates: the previous pass's inliers are removed, then RANSAC
   runs again. The input cloud of pass *k* is the output leftover of pass *k-1*.
5. Stop after `num_planes` successful fits, or earlier if a pass fails
   (`min_inliers`, `min_inlier_ratio`, or tilt).

Inlier sets do not overlap. Walls and boat sides are rejected when the plane
normal tilts more than `max_tilt_deg` from `up_axis`.

The `*_removed` cloud is the leftover after **every** successful pass: all
fitted-plane inliers are stripped. Points that never entered RANSAC (outside
the z-band / radius cylinder) stay in that cloud. That leftover is the input
to clustering (stage 3). Overview: [`../README.md`](../README.md).

```
candidates
    │
    ├─ RANSAC first  ── inliers → water (blue)     ── stripped from *_removed
    │
    leftover
    │
    ├─ RANSAC second ── inliers → plane 2 (green)  ── stripped from *_removed
    │
    leftover
    │
    └─ RANSAC third  ── inliers → plane 3 (gold)   ── stripped from *_removed
                                                      leftover → *_removed
```

## Files

| File | Role |
|------|------|
| `plane.py` | RANSAC + sequential leftover-cloud fitting |
| `WaterlineRansacNode.py` | ROS 2 node (TF, colorize, strip all inliers, markers) |
| `config/lidar_filtering/ransac/waterline.yaml` | Topics and per-pass knobs |
| `launch/waterline_ransac.launch.py` | This node only |
| `launch/lidar_filtering.launch.py` | Full pipeline |

## Topics

Defaults below match `waterline.yaml`. Change the YAML if you chain nodes.

| Direction | Topic | Contents |
|-----------|-------|----------|
| Input | `input_topic` | XYZ (optional intensity) PointCloud2 |
| Output | `colored_topic` | Same points, RGB by plane |
| Output | `removed_topic` | Input minus inliers of every fitted plane |
| Output | `markers_topic` | Fitted planes, inliers, 4 m radius cylinder |

Colors: blue = water, green = plane 2, gold = plane 3, orange = other / outside
the candidate cylinder.

## Launch

```bash
ros2 launch frontseat lidar_filtering.launch.py use_sim_time:=true
ros2 launch frontseat waterline_ransac.launch.py use_sim_time:=true
```

YAML is copied into the package share on `colcon build`. Restart the node after
edits. Optional `config_path:=/path/to/waterline.yaml` skips the share copy.

## YAML knobs

Shared candidate gate:

| Key | Meaning |
|-----|---------|
| `fit_frame` | Frame used for RANSAC (`base_link`) |
| `up_axis` | Gravity / “up”; tilt is measured from this |
| `max_radius` | XY cylinder around `base_link`; `<=0` disables |
| `ransac.num_planes` | How many sequential leftover passes |
| `ransac.z_min` / `z_max` | Height band in `fit_frame` |
| `ransac.random_seed` | `<0` is nondeterministic |

Per pass (`first`, `second`, `third`):

| Key | Meaning |
|-----|---------|
| `distance_threshold` | Inlier band around the plane, meters |
| `max_iterations` | Random 3-point hypotheses |
| `max_tilt_deg` | Reject steeper-than-this planes |
| `min_inliers` | Absolute inlier floor on the leftover cloud |
| `min_inlier_ratio` | Inlier floor as a fraction of the leftover cloud |

Missing later blocks reuse the last defined pass. Extra `num_planes` beyond
`third` also reuse the last block.

## Library

`fit_sequential_planes(points, num_planes=3, first=..., second=..., third=...)`
returns a list of `PlaneFit` objects. Pass `params=[...]` to supply an arbitrary
per-pass list. `fit_waterline_plane` is the one-plane helper.
