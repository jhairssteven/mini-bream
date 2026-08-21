# H0 Boat Experiment

Runs the H0 lemniscate MPC baseline (`baseline_ilos_velocity`) with the same algorithm and artifacts on every platform. Platform differences are isolated to YAML overlays — start the platform topics, then run the experiment from inside `mini_bream_autonomy`.

The launcher waits for the required sensor topics (and `motor_controller` on the real boat) before starting.

| Platform | Prerequisite (ROS graph) | Launcher |
|----------|--------------------------|----------|
| **sim** | `/blueboat/sensors/gps/gps/fix`, IMU, GT odometry | `./run_real_boat.sh --platform sim` |
| **real** | `/wamv/sensors/gps/gps/fix`, IMU, `motor_controller` | `./run_real_boat.sh` |
| **bench** | GPS + IMU only (thrust to sink topics) | `./run_real_boat.sh --bench` |
| **mock** | Pi mock sensors + Jetson | `./run_mock_test.sh` |

**Results (sim):** `h0_boat/results/<timestamp>/`  
**Results (field):** `/workspace/field_tests/h0_boat/<timestamp>/` (host: `src/field_tests/h0_boat/`)

## Architecture

```
                    ┌─────────────────────────────────────┐
                    │     run_h0_experiment.py            │
                    │  (same algorithm, same artifacts)   │
                    └─────────────────┬───────────────────┘
                                      │
              ┌───────────────────────┼───────────────────────┐
              │                       │                       │
         --platform sim         --platform real        --platform mock
              │                       │                       │
    ┌─────────▼─────────┐   ┌────────▼────────┐    ┌────────▼────────┐
    │  blueboat_sim     │   │   frontseat     │    │ mock_frontseat  │
    │  /blueboat/*      │   │   /wamv/*       │    │ /mock/wamv/*    │
    └───────────────────┘   └─────────────────┘    └─────────────────┘
```

Platform overlays (`config/h0_*_overlay.yaml`) set topic names, helper nodes, and experiment timing. The stack runner starts only the helper nodes needed for each platform (velocity odom on real boat; MPC-only on sim).

## Simulation

1. Start Gazebo:

```bash
cd src/docker
./mini_bream_env.sh start sim
```

2. Start autonomy (if not already running):

```bash
cd src/docker
./mini_bream_env.sh start autonomy
```

3. Inside the autonomy container:

```bash
docker exec -it mini_bream_autonomy bash
cd /workspace/ros2_ws/src/molo_wpt_follower/h0_boat
./run_real_boat.sh --platform sim           # full run
./run_real_boat.sh --platform sim --smoke   # shortened evaluate window
./run_real_boat.sh --platform sim --dry-run # config + ref_path only
```

Sim uses ground-truth odometry and publishes thrust directly to `/blueboat/thrusters/*`. Target RMSE: 0.1 m.

## Tuning (Bayesian optimization)

Retune H0 for the BlueBoat platform using scikit-optimize (`gp_minimize`, EI acquisition), matching the methodology in `mpc/experiments/tune_lemniscate.py`.

**Simulation** (inside autonomy, with sim topics already up):

```bash
cd /workspace/ros2_ws/src/molo_wpt_follower/h0_boat
./run_real_boat.sh --platform sim --tune --install    # 28 BO trials → config/h0_tuned_overlay.yaml
./run_real_boat.sh --platform sim --tune --quick      # 8 trials (smoke test)
```

**Field fine-tuning:**

```bash
./run_real_boat.sh --platform real --tune --install --n-calls 20
./run_real_boat.sh --platform bench --tune            # sensors only, no motors
```

Tuned parameters are written to `config/h0_tuned_overlay.yaml` (with `--install`) and automatically merged into all platform profiles. Results land in `h0_boat/results/tune_<platform>_<timestamp>/`.

**Tuned parameters (14-dim search):** cruise speed, ILOS lookahead/conv, PID kp/kd, max yaw rate, lateral-v gain, MPC horizon, Q/R weights.

**Boat model:** `config/h0_blueboat_boat.yaml` (30 kg catamaran, mapped from `blueboat_sim` VRX hydrodynamics).

## Real boat

1. Start the Pi stack so GPS/IMU and `motor_controller` are on the domain:

```bash
cd src/docker
./mini_bream_env.sh start pi
```

2. Start autonomy on Jetson/dev:

```bash
cd src/docker
./mini_bream_env.sh start autonomy
```

3. Release radio deadman (`arm=false`).

4. Inside the autonomy container:

```bash
docker exec -it mini_bream_autonomy bash
cd /workspace/ros2_ws/src/molo_wpt_follower/h0_boat
./run_real_boat.sh              # full run
./run_real_boat.sh --bench      # real GPS/IMU, no motor output
./run_real_boat.sh --smoke      # shortened evaluate window
```

**Thrust path:** MPC → `/pwm/*_thrust_cmd` → `motor_controller` → `pwm_daemon`.

Target RMSE: 0.1 m.

## Mock test (Pi sensors → Jetson MPC)

```bash
cd src/ros2_ws/src/molo_wpt_follower/h0_boat
./run_mock_test.sh
```

## Configuration

Config layering:

```
H0 algorithm config (tuned in sim)
  └── platform overlay (--platform sim|real|bench|mock)
        └── optional extra overlay (--overlay path)
```

Platform overlays live in `config/`:

| File | Platform |
|------|----------|
| `h0_sim_hal_overlay.yaml` | Gazebo (`blueboat_sim`) |
| `h0_boat_overlay.yaml` | Real boat (`frontseat`) |
| `h0_bench_overlay.yaml` | Real boat, thrust to sink topics (no motors) |
| `h0_mock_overlay.yaml` | Mock sensor topics |
| `h0_tuned_overlay.yaml` | BO best params (optional) |

## RViz

On a ground-station machine (while the experiment is running):

```bash
cd src/docker
./mini_bream_env.sh start gs --h0-boat
```

Topics: `/molo_mpc/ref_path`, `/molo_mpc/traversed_path`, `/molo_mpc/vehicle_pose`, `/molo_h0/recent_poses` (frame: `world`).

## Artifacts per run

Written to `<out>/H0_baseline/`:

- `config.yaml` — merged config
- `log.csv` — MPC state log
- `ref_path.csv` — reference lemniscate path
- `origin.json` — path activation origin
- `result.json` — RMSE, pass/fail, platform metadata
- `plots/` — validation plots (unless `--no-plot`)
