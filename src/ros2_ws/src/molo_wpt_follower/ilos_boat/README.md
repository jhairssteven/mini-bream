# ILOS Boat Experiment

Closed-loop lemniscate tracking with **ILOS path guidance + PID heading control + differential thrust**. No MPC and no boat model — thrust is allocated by a simple left/right mix.

Same experiment pipeline as `h0_boat` (config overlays, scoring, artifacts, field-test launcher). Compare against H0 (`baseline_ilos_velocity`), which uses the same ILOS/PID guidance but feeds a velocity MPC for thrust allocation.

| Platform | Prerequisite | Launcher |
|----------|--------------|----------|
| **sim** | `blueboat_sim` (Gazebo) | `./run_real_boat.sh --platform sim` |
| **real** | `frontseat` on Pi | `./run_real_boat.sh` |
| **bench** | `frontseat`, motors idle | `./run_real_boat.sh --bench` |

**Results (sim):** `ilos_boat/results/<timestamp>/`  
**Results (field):** `src/field_tests/ilos_boat/<timestamp>/` (mounted at `/workspace/field_tests/ilos_boat` in containers)

---

## Controller (what it implements)

**Mission:** track a closed Dubins-smoothed lemniscate path in a local ENU frame (`world`), anchored at the boat’s first GPS fix.

**1. Cross-track error** (signed, w.r.t. path tangent \(\psi_p\)):

\[
e_y = -\sin\psi_p\,(x - x_p) + \cos\psi_p\,(y - y_p)
\]

**2. ILOS desired heading** with variable lookahead \(\Delta(e_y) \in [\Delta_{\min}, \Delta_{\max}]\):

\[
\psi_d = \atan\!\left(-\hat\beta - \frac{e_y}{\Delta}\right) + \psi_p
\]

(\(\hat\beta\) is an integral state; \(\gamma=0\) disables integral action in the default config.)

**3. PID yaw-rate command** (heading error \(\tilde\psi = \mathrm{wrap}(\psi_d - \psi)\)):

\[
r_{\mathrm{cmd}} = \mathrm{clip}\!\left(K_p \tilde\psi + K_d \dot{\tilde\psi},\,-r_{\max},\,r_{\max}\right)
\]

**4. Surge command** (cruise speed \(U_c\), reduced in turns / large heading error):

\[
U_{\mathrm{cmd}} = U_c \cdot s_{\mathrm{head}}(\tilde\psi) \cdot \frac{1}{1 + k_\kappa |\kappa|}
\]

**5. Differential thrust** (normalized, no hydrodynamic model):

\[
\tau_{\mathrm{surge}} = \mathrm{clip}\!\left(g_s \frac{U_{\mathrm{cmd}}}{U_{\mathrm{ref}}},\,\tau_{\min},\,1\right), \quad
\tau_{\mathrm{yaw}} = g_y \frac{r_{\mathrm{cmd}}}{r_{\max}}
\]

\[
\tau_L = \tau_{\mathrm{surge}} - \tau_{\mathrm{yaw}}, \qquad
\tau_R = \tau_{\mathrm{surge}} + \tau_{\mathrm{yaw}}
\]

On the real boat, \(\tau_{L,R}\) are published directly to `/pwm/left_thrust_cmd` and `/pwm/right_thrust_cmd` (configured in `topics` in the boat overlay).

**Scoring:** RMSE of \(|e_y|\) from `/molo_mpc/cross_track_error` over the evaluate window. Target: **0.1 m**.

---

## Simulation

1. Start Gazebo:

```bash
cd src/docker
./mini_bream_env.sh start sim
```

2. (Optional) RViz in another terminal:

```bash
cd src/docker
./mini_bream_env.sh start gs --h0-boat --sim-viz
```

3. Run the experiment:

```bash
cd src/ros2_ws/src/molo_wpt_follower/ilos_boat
./run_real_boat.sh --platform sim           # full run
./run_real_boat.sh --platform sim --smoke   # shortened evaluate window
./run_real_boat.sh --platform sim --dry-run # config + ref_path only
./run_stack.sh --platform sim               # controller only (Ctrl+C to stop; good for RViz)
```

Sim writes to `ilos_boat/results/<timestamp>/`. Uses ground-truth odometry and publishes thrust directly to `/blueboat/thrusters/*`.

---

## Real boat (field test)

1. Start the Pi stack:

```bash
cd src/docker
./mini_bream_env.sh start pi
```

2. Release radio deadman (`arm=false`).

3. Run from the ground-station laptop:

```bash
cd src/ros2_ws/src/molo_wpt_follower/ilos_boat
./run_real_boat.sh                    # full run (default: --platform real)
./run_real_boat.sh --bench            # GPS/IMU live, thrust to sink topics (no motors)
./run_real_boat.sh --smoke            # shortened evaluate window
```

**Thrust path:** ILOS → `/pwm/*_thrust_cmd` → `motor_controller` → `pwm_daemon`.

**RViz** (ground station, while the run is active):

```bash
cd src/docker
./mini_bream_env.sh start gs --h0-boat
```

No `--sim-viz` on the real boat. Fixed frame: `world`.

---

## Tuning (Bayesian optimization)

Tunes ILOS / PID / surge-yaw mix parameters (14-dim; no MPC weights). Uses scikit-optimize (`gp_minimize`, EI), same methodology as `h0_boat/tune_h0.py`.

**Simulation:**

```bash
cd src/docker && ./mini_bream_env.sh start sim

cd src/ros2_ws/src/molo_wpt_follower/ilos_boat
./run_real_boat.sh --platform sim --tune --install    # 28 trials → config/ilos_tuned_overlay.yaml
./run_real_boat.sh --platform sim --tune --quick      # 8 trials (smoke)
```

**Field fine-tuning:**

```bash
./run_real_boat.sh --platform real --tune --install --n-calls 20
./run_real_boat.sh --platform bench --tune            # sensors only, no motors
```

Refine around a prior result:

```bash
./run_real_boat.sh --platform sim --tune --refine results/tune_sim_20260727/tune_result.json
```

---

## Configuration

```
ilos_base.yaml          # algorithm + lemniscate mission
  └── platform overlay  (--platform sim|real|bench)
        └── ilos_tuned_overlay.yaml  (auto-merged if present)
```

| File | Platform |
|------|----------|
| `config/ilos_sim_overlay.yaml` | Gazebo (`blueboat_sim`) |
| `config/ilos_boat_overlay.yaml` | Real boat (`frontseat`) |
| `config/ilos_bench_overlay.yaml` | Real boat, thrust to sink topics (no motors) |
| `config/ilos_tuned_overlay.yaml` | BO best params (optional) |

---

## RViz topics

Uses the shared `h0_boat.rviz` config (`--h0-boat` / `--ilos-boat`):

| Display | Topic | ILOS publishes? |
|---------|-------|-----------------|
| Reference Path | `/molo_mpc/ref_path` | yes (once at path start) |
| Traversed Path | `/molo_mpc/traversed_path` | yes |
| Vehicle Pose | `/molo_mpc/vehicle_pose` | yes |
| Recent Poses | `/molo_h0/recent_poses` | yes |
| Predicted Path | `/molo_mpc/pred_path` | no (MPC only) |

---

## Artifacts per run

Written to `<out>/ILOS_PID/`:

- `config.yaml` — merged config
- `log.csv` — state / XTE / thrust log
- `ref_path.csv` — reference lemniscate
- `origin.json` — path activation origin (first GPS fix)
- `result.json` — RMSE, pass/fail, platform metadata
- `plots/` — validation plots (unless `--no-plot`)
