# RoboSense RS-LiDAR-AIRY (ROS 2 Humble)

Driver config for the Airy LiDAR. Runs in the Jetson `mini-bream:lidar` container
(`lidar` service in `src/docker/docker-compose.frontseat.yml`). Start only that
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
docker compose -f docker-compose.frontseat.yml up --build lidar
```

`start_lidar.sh` builds `rslidar_msg` / `rslidar_sdk` (IMU parse on) / `frontseat` (for launch+config), then starts `airy_lidar`.

## Verify

```bash
docker exec mini_bream_lidar bash -lc '
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

## References

- [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)
- [rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg)
