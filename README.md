# Mini-Bream

Source and docs for the **Mini-BREAM** ASV stack: ROS 2 Humble, Docker-based deployment across Raspberry Pi (motors / frontseat), Jetson (LiDAR / ZED), and a ground station (teleop + RViz).

**Start here:** [QUICKSTART.md](QUICKSTART.md)


### Requirements

- Ubuntu 22.04+
- Docker and Docker Compose

### Clone

```bash
git clone git@github.com:jhairssteven/mini-bream.git
cd mini-bream
git submodule update --init --recursive
```

---

## Layout

```
mini-bream/
├── QUICKSTART.md          # bring-up on Pi / Jetson / GS
├── docs/                  # misc notes (e.g. rosbag filtering)
├── scripts/               # host helper scripts
├── sync_ws.sh             # sync workspace to the Pi
└── src/
    ├── docker/            # images, compose, mini_bream_env.sh
    ├── teleop/            # radio teleop + PWM mux (non-ROS path)
    ├── field_tests/       # bags and experiment artifacts
    ├── ros2_ws/           # core ROS 2 packages
    └── vrx_ws/            # VRX / Gazebo simulation packages
```

---

## Documentation index

### Bring-up and ops

| Doc | Topic |
|-----|--------|
| [QUICKSTART.md](QUICKSTART.md) | Start / stop Pi, Jetson, and ground station |
| [src/docker/README.md](src/docker/README.md) | Images, compose files, architecture |
| [src/teleop/README.md](src/teleop/README.md) | Emergency radio teleop and PWM mux |

### Field tests and logs

| Doc | Topic |
|-----|--------|
| [src/field_tests/rosbags/README.md](src/field_tests/rosbags/README.md) | Bag layout and filtering notes |
| [src/field_tests/h0_boat/README.md](src/field_tests/h0_boat/README.md) | H0 experiment run artifacts |

### Simulation

| Doc | Topic |
|-----|--------|
| [src/ros2_ws/src/blueboat_sim/README.md](src/ros2_ws/src/blueboat_sim/README.md) | BlueBoat Gazebo / HAL sim |
| [src/vrx_ws/src/README.md](src/vrx_ws/src/README.md) | Upstream VRX environment |

### Navigation and control (`ros2_ws`)

| Doc | Topic |
|-----|--------|
| [blueboat_nav2](src/ros2_ws/src/blueboat_nav2/README.md) | Nav2 local planning (LiDAR → costmap → Dubins) |
| [path_follower](src/ros2_ws/src/molo_wpt_follower/path_follower/README.md) | `/plan` → follower interface |
| [ilos_boat](src/ros2_ws/src/molo_wpt_follower/ilos_boat/README.md) | ILOS + PID differential-thrust experiments |
| [h0_boat](src/ros2_ws/src/molo_wpt_follower/h0_boat/README.md) | H0 / velocity-MPC baseline experiments |
| [mpc](src/ros2_ws/src/molo_wpt_follower/mpc/README.md) | MPC trajectory follower |
| [mission_planner](src/ros2_ws/src/mission_planner/README.md) | Mission generation / planner package |
| [moloplanner](src/ros2_ws/src/mission_planner/mission_planner/moloplanner/README.md) | Molo perception / planning stack |

### Frontseat sensors and perception

| Doc | Topic |
|-----|--------|
| [heading](src/ros2_ws/src/frontseat/config/heading/README.md) | Moving-base RTK heading filters |
| [rslidar_airy](src/ros2_ws/src/frontseat/config/rslidar_airy/README.md) | RS-LiDAR-AIRY driver setup |
| [zed2i](src/ros2_ws/src/frontseat/config/zed2i/README.md) | ZED 2i on Jetson perception |
| [lidar_filtering](src/ros2_ws/src/frontseat/frontseat/lidar_filtering/README.md) | Self-filter → RANSAC → clustering |
| [self_filter](src/ros2_ws/src/frontseat/config/lidar_filtering/self_filter/README.md) | Hull / mast self-filter config |
| [blueboat mesh](src/ros2_ws/src/frontseat/meshes/blueboat/README.md) | Boat mesh assets |

### Vendored / third-party packages

| Doc | Topic |
|-----|--------|
| [rslidar_sdk](src/ros2_ws/src/rslidar_sdk/README.md) | RoboSense LiDAR SDK |
| [rslidar_msg](src/ros2_ws/src/rslidar_msg/README.md) | RoboSense packet messages |
| [ublox](src/ros2_ws/src/ublox/README.md) | u-blox GPS driver |


---
Licensed under the [MIT License](LICENSE).