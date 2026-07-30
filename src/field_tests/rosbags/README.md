# ROS 2 bag recordings

Bags recorded via `ros2 launch frontseat rosbag.launch.py` are written here
(inside containers: `/workspace/field_tests/rosbags`).

## Jul 29 2026 field test bags

Heavy ZED / LiDAR topics were filtered on the Jetson into `jul_29_2026_filtered/`.
Use `scripts/filter_rosbag.py` to reproduce filtering locally.

Heading debug visualization: see `heading_viz/README.md` (includes isolated Docker replay).

Example record:

```bash
ros2 launch frontseat rosbag.launch.py bag_suffix:=rtk_test
ros2 launch frontseat rosbag.launch.py record_perception:=false bag_suffix:=rtk_only
```

Manual recording: use `-s mcap` (Humble defaults to sqlite3), e.g. `ros2 bag record -s mcap -a`.

Bag folders are named `YYYY-MM-DD/rosbag_YYYY-MM-DD_HH-MM-SS[_suffix]`.
