# RoboSense RS-LiDAR-AIRY Setup (ROS 2 Humble)

Reproducible setup for the [RoboSense rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk) driver in the mini-bream ROS 2 workspace, using the `linc_project` Docker container.

Sensor configuration and launch files live in the `frontseat` package:

- `config/rslidar_airy/airy.yaml` — driver parameters
- `config/rslidar_airy/setup_network.sh` — host Ethernet setup
- `launch/airy_lidar.launch.py` — ROS 2 launch file

## Hardware and network

The RS-LiDAR-AIRY uses Ethernet with factory-default addressing:

| Device | IP address | Notes |
|--------|------------|-------|
| LiDAR | `192.168.1.200` | Fixed destination for point-cloud packets |
| Host (this PC) | `192.168.1.102/24` | Must be on the same subnet as the LiDAR |

Default UDP ports:

| Port | Purpose |
|------|---------|
| `6699` | MSOP (point cloud data) |
| `7788` | DIFOP (device info) |
| `6688` | IMU data |

### 1. Connect hardware

1. Connect the LiDAR Ethernet cable to the host Ethernet port (e.g. `enp4s0`).
2. Power on the LiDAR and wait for it to initialize.

### 2. Configure host network

From the host or inside the `linc_project` container (host network mode):

```bash
/workspace/codebase/mini-bream/src/ros2_ws/src/frontseat/config/rslidar_airy/setup_network.sh
```

Override the interface or IP if needed:

```bash
LIDAR_IFACE=eth0 LIDAR_HOST_IP=192.168.1.102 ./setup_network.sh
```

Verify connectivity:

```bash
ping -c 3 192.168.1.200
# Optional: confirm UDP traffic
tcpdump -i enp4s0 -c 5 -n udp port 6699
```

**Persistent network (recommended for bare metal):** create a Netplan file on the host, e.g. `/etc/netplan/99-robosense-airy.yaml`:

```yaml
network:
  version: 2
  ethernets:
    enp4s0:
      addresses:
        - 192.168.1.102/24
```

Apply with `sudo netplan apply`.

## Software dependencies

Inside the `linc_project` container (Ubuntu 22.04 + ROS 2 Humble):

```bash
apt-get update
apt-get install -y libpcap-dev libyaml-cpp-dev git iproute2 tcpdump
```

ROS 2 Humble is already provided by the container image (`osrf/ros:humble-desktop`).

## ROS workspace packages

Clone the upstream driver packages into `src/ros2_ws/src/`:

```bash
cd /workspace/codebase/mini-bream/src/ros2_ws/src
git clone --recursive https://github.com/RoboSense-LiDAR/rslidar_sdk.git
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
```

`rslidar_sdk` includes the `rs_driver` submodule. Always use `git clone --recursive` or run `git submodule update --init` after cloning.

Build `rslidar_sdk` with IMU parsing enabled (`ENABLE_IMU_DATA_PARSE=ON` in `rslidar_sdk/CMakeLists.txt`, already set in this project).

## Build

```bash
cd /workspace/codebase/mini-bream/src/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rslidar_msg rslidar_sdk frontseat --symlink-install
source install/setup.bash
```

## Configuration

Key settings in `airy.yaml`:

```yaml
lidar_type: RSAIRY
host_address: 192.168.1.102
msop_port: 6699
difop_port: 7788
imu_port: 6688
ros_send_point_cloud_topic: /rslidar_points
ros_send_imu_data_topic: /rslidar_imu_data
```

## Run and verify

1. Configure the network (see above).
2. Source the workspace and launch:

```bash
source /opt/ros/humble/setup.bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash
ros2 launch frontseat airy_lidar.launch.py
```

3. In another terminal, confirm publishing:

```bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash
ros2 topic list | grep rslidar
ros2 topic hz /rslidar_points
ros2 topic hz /rslidar_imu_data
```

Expected topics:

- `/rslidar_points` (`sensor_msgs/PointCloud2`)
- `/rslidar_imu_data` (`sensor_msgs/Imu`)

## Docker notes

The `linc_project` container uses `network_mode: host` and `privileged: true`, so the driver binds directly to host interfaces. No extra port mapping is required.

Ensure the Ethernet interface has `192.168.1.102/24` before launching the driver.

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No UDP packets | Host IP not set on Ethernet NIC; run `setup_network.sh` |
| Driver starts but no points | Wrong `lidar_type` (must be `RSAIRY`), or wrong ports |
| `Config loaded` error | YAML indentation in `airy.yaml` |
| Low frame rate on Humble | See [ROS2 Humble frame rate doc](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/13_how_to_solve_ROS2_humble_frame_rate_drop.md) |

## References

- [rslidar_sdk repository](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [rslidar_msg repository](https://github.com/RoboSense-LiDAR/rslidar_msg)
- [Online LiDAR parameter guide](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/06_how_to_decode_online_lidar.md)
