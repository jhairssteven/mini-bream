# Boat self-filter

Removes the BlueBoat hull footprint and vertical 25×25 mm aluminum sensor masts
from the RoboSense LiDAR cloud. Filter volumes are defined in `base_link`; the
filtered cloud keeps the same `frame_id` as the input (typically `rslidar`).

`base_link` origin is the hull top surface (deck); the hull box extends downward (z < 0).

## Topics

| Input | Output | Debug (optional) |
|-------|--------|------------------|
| `/rslidar_points` | `/rslidar_points/filtered` | `/rslidar_points/removed` |

When `debug.enabled` is true in YAML (or `debug_enabled:=true` at launch):
- `/self_filter/debug_markers` — RViz cube markers for filter volumes
- `/rslidar_points/removed` — points removed by the filter (inverse of filtered)

Set `debug.enabled: false` in production to skip marker/removed-cloud overhead.

## Launch

```bash
ros2 launch frontseat self_filter.launch.py
ros2 launch frontseat self_filter.launch.py debug_enabled:=true
```

Volumes are defined in `blueboat_self_filter.yaml`. Tune `center` / `size` after field
measurements. Set `enabled: false` on individual volumes to disable them.

## YAML debug block

```yaml
debug:
  enabled: true
  publish_markers: true
  markers_topic: /self_filter/debug_markers
  publish_removed_cloud: true
```

## Parameters

| Parameter | Description |
|-----------|-------------|
| `config_path` | YAML path (default: package `blueboat_self_filter.yaml`) |
| `debug_enabled` | Force debug outputs on (overrides YAML `debug.enabled`) |
