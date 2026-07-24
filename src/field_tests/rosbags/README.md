# ROS 2 bag recordings

Bags recorded via `ros2 launch frontseat rosbag.launch.py` are written here
(inside containers: `/workspace/field_tests/rosbags`).

This directory is mounted into Pi, Jetson, and ground-station Docker services
so recordings persist after containers are removed.

H0 MPC experiment results are saved alongside under `field_tests/h0_boat/`
(see `molo_wpt_follower/h0_boat/run_real_boat.sh`).

Example:

```bash
ros2 launch frontseat rosbag.launch.py record_bag:=true bag_storage:=mcap bag_suffix:=rtk_test
```

Bag folders are named `YYYY-MM-DD/rosbag_YYYY-MM-DD_HH-MM-SS[_suffix]`.
