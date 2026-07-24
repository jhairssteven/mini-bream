# H0 Boat Experiment

Runs the H0 lemniscate MPC baseline on the real boat (or a Pi→Jetson mock). Uses the tuned sim config from `mpc/experiments/results/lemniscate_validation/H0_baseline` plus a hardware overlay.

**Thrust path (real boat):** MPC → `/molo_boat/thrust_*` → `thrust_bridge` → `/pwm/*_thrust_cmd` → `motor_controller` → `pwm_daemon` (UDP). Release the radio deadman so `pwm_daemon` accepts ROS thrust.

**Results** persist on the host under `src/field_tests/h0_boat/<timestamp>/` (mounted at `/workspace/field_tests/h0_boat` in containers).

## Prerequisites

- Pi: `./src/docker/mini_bream_env.sh start pi` (`pwm_daemon`, `radio_rx`, `frontseat`)
- Jetson (mock only): `./mini_bream_env.sh start jetson`
- `ROS_DOMAIN_ID` must match on all hosts (default `0`)
- MPC Python deps (`dubins`, `osqp`, `scipy`, `matplotlib`, …) are baked into the `frontseat` and `perception` images via `install_ros2_ws_deps.sh`. After changing deps, rebuild:

```bash
cd src/docker
docker compose -f docker-compose.frontseat.yml build frontseat   # Pi
docker compose -f docker-compose.frontseat.yml build perception  # Jetson
```

## Mock test (Pi sensors → Jetson MPC)

From the Pi:

```bash
cd src/ros2_ws/src/molo_wpt_follower/h0_boat
./run_mock_test.sh
```

Syncs to Jetson, publishes mock GPS/IMU on `/mock/wamv/sensors/*`, runs MPC with `thrust_mode: log_only`. Success = `Mock integration test PASSED`.

Optional: `JETSON_IP=192.168.0.102 DURATION=30 ./run_mock_test.sh`

## Real boat

1. Start the Pi stack:

```bash
cd src/docker
./mini_bream_env.sh start pi          # motors live
# ./mini_bream_env.sh start pi --dry-run   # bench: pwm_daemon dry_run
```

2. Release radio deadman (arm=false).

3. Run from the **host/pi** (recommended):

```bash
cd src/ros2_ws/src/molo_wpt_follower/h0_boat
./run_real_boat.sh              # full run → field_tests/h0_boat/<timestamp>/
./run_real_boat.sh --bench      # real GPS/IMU, no motor output (smoke test)
./run_real_boat.sh --smoke      # shortened evaluate window
```

Or manually inside the container:

```bash
docker exec -it mini_bream_frontseat bash -lc \
  '/workspace/docker/start_h0_boat.sh --out /workspace/field_tests/h0_boat/manual_run'
```

Origin is taken from the first GPS fix unless you pass `--origin-lat` / `--origin-lon`. Default overlay: `config/h0_boat_overlay.yaml` (`thrust_mode: pwm_topics`).

Each run writes `config.yaml`, `log.csv`, `ref_path.csv`, `origin.json`, `result.json`, and plots under `H0_baseline/`.

## RViz

On a ground-station machine (while the experiment is running):

```bash
cd src/docker
./mini_bream_env.sh start gs --h0-boat
```

Topics: `/molo_mpc/ref_path`, `/molo_mpc/traversed_path`, `/molo_mpc/vehicle_pose`, `/molo_h0/recent_poses` (frame: `world`).
