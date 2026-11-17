# Ground-speed MPC → ILOS experiment

Implements `pipeline.tex` with the same experiment harness as `h0_boat` / `ilos_boat`.

**Cascade:** fixed mission path (lemniscate, …) → local **ground-speed kinematic MPC** (polyline + `U_MPC`) → **ILOS** (`ψ_d`, `U_d`) → surge **PI** / yaw **PD** → thruster allocation → sim: force **[N]** to Gazebo / real: `f^{-1}` → normalized PWM.

Unlike H0 (`baseline_ilos_velocity`), MPC here is a **path generator**, not a hydro thrust allocator. Unlike `ilos_boat`, a short-horizon GS-MPC sits above ILOS. RRT★ from the TeX notes is replaced by the same parametric trajectories as the other packages.

| Platform | Prerequisite (ROS graph) | Launcher |
|----------|--------------------------|----------|
| **sim** | `/blueboat/sensors/gps/gps/fix`, IMU, GT odometry | `./run_real_boat.sh --platform sim` |
| **real** | `/wamv/sensors/gps/gps/fix`, IMU, `motor_controller` | `./run_real_boat.sh` |
| **bench** | GPS + IMU only (thrust to sink topics) | `./run_real_boat.sh --bench` |

**Results (sim):** `ground_speed_mpc_ilos/results/<timestamp>/`  
**Results (field):** `/workspace/field_tests/ground_speed_mpc_ilos/<timestamp>/`

## Simulation

From the **host**, always start from a clean Gazebo (stops any existing sim first):

```bash
cd src/ros2_ws/src/molo_wpt_follower/ground_speed_mpc_ilos
./ensure_clean_sim.sh          # stop old sim → start fresh → wait for GPS
# autonomy must already be up:  ./mini_bream_env.sh start autonomy
```

Then inside autonomy:

```bash
docker exec -it mini_bream_autonomy bash
cd /workspace/ros2_ws/src/molo_wpt_follower/ground_speed_mpc_ilos
./run_real_boat.sh --platform sim           # full run
./run_real_boat.sh --platform sim --smoke   # shortened evaluate window
./run_real_boat.sh --platform sim --dry-run # config + ref_path only
```

`run_real_boat.sh` also kills leftover GS/ILOS/H0 stacks and zeros thrusters before a sim run.

## Configuration

```
gs_base.yaml
  └── platform overlay (--platform sim|real|bench)
        └── optional gs_tuned_overlay.yaml / --overlay
```

Package-local code: `ground_speed_mpc.py`, `inner_control.py`, `gs_mpc_ilos_follower.py`.  
Shared: `mpc.build_mission_path`, `path_activation`, `viz`, `algorithms` geometry helpers.

## Artifacts per run

Written to `<out>/GS_MPC_ILOS/`:

- `config.yaml`, `log.csv`, `ref_path.csv`, `origin.json`, `result.json`, `plots/`

## Staged constrained tuning (LHS → local)

Per phase:

1. Coarse **Latin hypercube** in tight gain bounds  
2. **Local trust-region** polish around best *valid* seed  

Phases under `results/tune_staged_<platform>_*/phase_<name>/` (or `tune_lhs_local_*`):

1. **inner** — surge PI / yaw PD (including `k_ff`; keep near plant gain, not 30–50)  
2. **ilos** — ILOS + speed schedule (inner frozen)  
3. **mpc** — GS-MPC weights/limits (inner+ILOS frozen)  
4. **global** — joint polish around staged seed  

Validity: lemniscate completion **≥ 80%** required (invalid → `rejected/`, not candidates). Target RMSE **0.1 m**.  
With honest surge tracking (no overdrive), a ~37 m path needs roughly **≥100 s** evaluate at ~0.3 m/s cruise — shorter windows will fail the gate even with good tracking.

### Simulation

Uses **open_water** Gazebo (no obstacle course):

```bash
# host: clean open_water sim
./ensure_clean_sim.sh

docker exec -it mini_bream_autonomy bash
cd /workspace/ros2_ws/src/molo_wpt_follower/ground_speed_mpc_ilos
./run_real_boat.sh --platform sim --tune --install
# quick: --quick  (4+6 / stage, 3+6 global)
```

### Real boat / new boat

Retune on the water for each hull. Do **not** treat a sim `gs_tuned_overlay.yaml` as final for a new boat (the tuner starts with `include_tuned=False`).

**1. ROS graph (frontseat + autonomy linked)**

| Need | Expected (see `config/gs_boat_overlay.yaml`) |
|------|-----------------------------------------------|
| GPS | `/wamv/sensors/gps/gps/fix` |
| IMU | `/wamv/sensors/imu/imu/data` |
| Motors | `motor_controller` → `/pwm/left_thrust_cmd`, `/pwm/right_thrust_cmd` |
| Optional odom | `/odom` |

If the new boat uses different topic names, edit `gs_boat_overlay.yaml` (or add a boat-specific overlay) before tuning. `./run_real_boat.sh` waits for these unless you pass `--skip-checks`.

**2. Plant params for this hull**

Set geometry / thrust map in `gs_base.yaml` or a boat overlay so the inner loops match the plant:

- `allocation.thruster_sep_m` — left/right thruster spacing  
- `thruster_map` (`c1`, `t_max`, …) — thrust ↔ PWM for **this** boat  
- `allocation.t_max` / `t_min`  
- Seed `surge_pi.k_ff` ≈ steady-state \(X/U\) (keep small; keep `ff_overspeed_fade_mps`)  
- Confirm mission size / open water: lemniscate `scale_m: 7` ≈ 37 m path  

**3. Timing / completion gate**

Ensure real-boat `evaluate_s` in `tune_gs.py` (`TIMING` / `VALIDATION_TIMING` for `real`) and `gs_boat_overlay.yaml` is long enough for ≥80% completion (~**100–120 s** at cruise ~0.3 m/s), or shrink the path / raise cruise. A ~55 s window on a ~37 m path will fail the gate under honest speed tracking.

**4. Field procedure**

1. Clear area, kill switch ready; prefer calm water for the first campaign.  
2. Start frontseat + autonomy (same ROS domain / DDS as usual).  
3. Inside autonomy:

```bash
cd /workspace/ros2_ws/src/molo_wpt_follower/ground_speed_mpc_ilos

# optional sanity
./run_real_boat.sh --platform real --dry-run
./run_real_boat.sh --platform real --smoke

# full staged tune (inner → ILOS → MPC → global); write best overlay
./run_real_boat.sh --platform real --tune --install

# first time on a new boat: fewer trials
./run_real_boat.sh --platform real --tune --install --quick
```

4. Artifacts: `/workspace/field_tests/ground_speed_mpc_ilos/tune_staged_real_<timestamp>/`  
5. With `--install`, best params land in `config/gs_tuned_overlay.yaml`.

Full campaigns are dozens of trials × (warmup ~20 s + evaluate ~100 s) — plan several hours of open water, battery, and operator attention.

**Avoid**

- Copying sim-tuned gains onto a new hull without retuning  
- Searching `k_ff` in the old 30–50 range  
- `--skip-checks` unless you know the graph is already up  
- Expecting ≥80% completion with a long path and a short evaluate window  
