# RoboSense RS-LiDAR-AIRY (ROS 2 Humble)

Driver config for the Airy LiDAR. Runs in the Jetson `mini-bream:perception` container
(`perception` service in `src/docker/docker-compose.frontseat.yml`). Start only that
service on the Jetson; Pi runs `pwm_daemon` / `radio_rx` / `frontseat`.

| File | Role |
|------|------|
| `airy.yaml` | `rslidar_sdk` parameters |
| `setup_network.sh` | Host secondary-IP setup for factory LiDAR addressing |
| `../../launch/airy_lidar.launch.py` | ROS 2 launch (no RViz) |

## Why a secondary IP (not just changing `host_address`)

There are two different addresses:

1. **LiDAR destination IP** — programmed into the sensor firmware. Factory default: send MSOP/IMU UDP **to** `192.168.1.102`.
2. **`host_address` in `airy.yaml`** — where `rslidar_sdk` **binds** to receive those packets.

Changing only `host_address` to the Jetson’s DHCP address (`192.168.0.102`) does nothing useful unless you also reconfigure the LiDAR’s destination to match. Until then the sensor still transmits to `192.168.1.102`.

On a shared L2 LAN (LiDAR → router ← Jetson), the non-invasive approach is: keep the factory destination and add `192.168.1.102/24` as a **secondary** address on Jetson `eth0` beside `192.168.0.102`.

To drop the secondary IP later: reconfigure the LiDAR destination (web UI / RSView) to `192.168.0.102`, then set `host_address: 192.168.0.102` in `airy.yaml`.

| Device | Address | Notes |
|--------|---------|-------|
| Jetson primary | e.g. `192.168.0.102` | DHCP on router LAN |
| Jetson LiDAR recv | `192.168.1.102/24` | secondary on eth0 |
| LiDAR | `192.168.1.200` | factory device IP |

UDP ports: MSOP `6699`, DIFOP `7788`, IMU `6688`. Only one host may own `192.168.1.102`.

### Configure host network

```bash
sudo LIDAR_IFACE=eth0 \
  /home/orin-nano/mini-bream/src/ros2_ws/src/frontseat/config/rslidar_airy/setup_network.sh
```

```bash
ping -c 3 192.168.1.200
sudo tcpdump -i eth0 -c 5 -n udp port 6699
```

**Persistent (NetworkManager):**

```bash
CONN=$(nmcli -t -f NAME,DEVICE connection show --active | awk -F: '$2=="eth0"{print $1; exit}')
sudo nmcli connection modify "$CONN" +ipv4.addresses 192.168.1.102/24
sudo nmcli connection up "$CONN"
```

## Docker (Jetson)

```bash
cd /home/orin-nano/mini-bream/src/docker
docker compose -f docker-compose.frontseat.yml up --build perception
```

`start_perception.sh` builds `rslidar_msg` / `rslidar_sdk` (IMU parse on) / `frontseat` (for launch+config), then starts `airy_lidar`.

## Verify

```bash
docker exec mini_bream_perception bash -lc '
  source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
  ros2 topic hz /rslidar_points
  ros2 topic hz /rslidar_imu_data'
```

## Troubleshooting

| Symptom | Check |
|---------|-------|
| No UDP / ping fail | Secondary IP missing; run `setup_network.sh` |
| Driver up, no points | Another host owns `192.168.1.102`, or LiDAR dest ≠ `host_address` |
| No IMU topic | Rebuild with `-DENABLE_IMU_DATA_PARSE=ON` |
| Low frame rate | Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) |
| Nav2 `/scan observation buffer` warnings, no plan | `use_lidar_clock` must be `false` in `airy.yaml` for field use so LiDAR stamps match host/odometry time (see below) |

### Nav2 / costmap time sync

For real-boat operation (`use_sim_time:=false`), `airy.yaml` sets `use_lidar_clock: false`.
With `use_lidar_clock: true`, the Airy stamps point clouds with the sensor uptime clock while
`gps_map_odom` `/odom` and TF use host time. Nav2 then cannot transform
`/scan` into `map` and logs `The /scan observation buffer has not been updated`.

After changing this setting, restart the LiDAR tmux session in perception:

```bash
docker exec mini_bream_perception bash -lc 'tmux kill-session -t airy_lidar; tmux new-session -d -s airy_lidar "bash -lc \"source /opt/ros/humble/setup.bash && source /opt/zed_ws/install/setup.bash && source /workspace/ros2_ws/install/setup.bash && exec ros2 launch frontseat airy_lidar.launch.py\""'
```

Then restart Nav2 planning in the autonomy container (tmux session with `molo_autonomy.launch.py`).

## References

- [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg)
