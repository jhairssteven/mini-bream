# ILOS Boat — Config Overlays

The overlay system is a **layered YAML merge**: one shared algorithm config (`ilos_base.yaml`) plus small platform-specific patches, rather than a full duplicate config per platform.

- **Base** = everything that's the same everywhere (lemniscate mission, ILOS/PID defaults, Dubins settings)
- **Platform overlay** = differences for sim / real / bench (topic names, sensors, timing)
- **Tuned overlay** = only the parameters Bayesian optimization changed
- **Extra overlay** = optional one-off patch (for example `docker/config/autonomy_overlay.yaml` when that file exists)

At runtime these layers become **one flattened config**. The controller never reads several YAML files itself — it loads the merged result written to each run's `config.yaml`.

Later layers win on conflicts. Keys **not mentioned** in an overlay are **left unchanged**. Nested maps are merged key-by-key (so you can override `pid.kp` without replacing the whole `pid` block).

---

## Example

**Base** (`ilos_base.yaml`):

```yaml
pid:
  kp: 1.6
  ki: 0.0
  kd: 0.45
path:
  cruise_speed_mps: 0.25
```

**Tuned overlay** (`ilos_tuned_overlay.yaml`):

```yaml
pid:
  kp: 1.759
  kd: 0.192
path:
  cruise_speed_mps: 0.379
```

**Result after merge:**

```yaml
pid:
  kp: 1.759      # from tuned
  ki: 0.0        # kept from base (tuned didn't mention it)
  kd: 0.192      # from tuned
path:
  cruise_speed_mps: 0.379
```

A real run's saved `config.yaml` typically has sim (or boat) topics from the platform overlay **and** tuned PID values from `ilos_tuned_overlay.yaml`.

---

## Merge order

| Step | File | When |
|------|------|------|
| 1 | `ilos_base.yaml` | Always |
| 2 | Platform overlay(s) | Depends on `--platform` |
| 3 | `ilos_tuned_overlay.yaml` | **Auto** if the file exists |
| 4 | Extra overlay (`--overlay`) | If passed on CLI |
| 5 | `--origin-lat/lon` | Runtime GPS override (not a file) |

### Per platform

**`--platform sim`**

```
ilos_base.yaml
  → ilos_sim_overlay.yaml
  → ilos_tuned_overlay.yaml (if present)
```

**`--platform real`**

```
ilos_base.yaml
  → ilos_boat_overlay.yaml
  → ilos_tuned_overlay.yaml (if present)
  → autonomy_overlay.yaml (if the file exists; added by run_real_boat.sh)
```

**`--platform bench`**

```
ilos_base.yaml
  → ilos_boat_overlay.yaml
  → ilos_bench_overlay.yaml    ← second platform overlay
  → ilos_tuned_overlay.yaml (if present)
  → autonomy_overlay.yaml (if the file exists; added by run_real_boat.sh)
```

Bench is "real boat sensors + safe thrust sinks": it starts from the boat overlay, then bench overrides **only the thrust topics**:

```yaml
# ilos_boat_overlay.yaml
topics:
  left_thrust: /pwm/left_thrust_cmd      # real motors

# ilos_bench_overlay.yaml (applied after)
topics:
  left_thrust: /molo_boat/thrust_left    # sink topic, no motors
```

Only `left_thrust` and `right_thrust` change; GPS/IMU topics stay from the boat overlay.

---

## What each overlay changes

| Overlay | Purpose | Main overrides |
|---------|---------|----------------|
| `ilos_base.yaml` | Algorithm + mission | ILOS/PID gains, lemniscate shape, Dubins, default experiment timing |
| `ilos_sim_overlay.yaml` | Gazebo | `sim_enable: true`, GT odometry, `/blueboat/*` topics, fixed origin lat/lon |
| `ilos_boat_overlay.yaml` | Real Pi | `sim_enable: false`, `/wamv/*` + `/pwm/*` topics, longer warmup, GPS origin=null (wait for fix) |
| `ilos_bench_overlay.yaml` | Safe testing | Thrust → sink topics, shorter evaluate window |
| `ilos_tuned_overlay.yaml` | BO results | Only tuned params: `path`, `ilos`, `pid`, `guidance`, `thrust`, `speed` |
| `autonomy_overlay.yaml` | Optional extra overlay | Ensures `velocity_odom_enabled: true` when the file is present |

Sim vs real is mostly **which ROS topics to subscribe/publish**, not different control math.

---

## When the merged config is used

1. YAML layers are merged in memory (see order above).
2. Run-specific fields (e.g. `log_csv` path) are added and the **final result** is written to `<run_dir>/config.yaml`.

That saved `config.yaml` is what `stack_runner.py` and `ilos_follower.py` load. To see what actually ran, open `results/<timestamp>/ILOS_PID/config.yaml` — that is the fully resolved config, not any single overlay file.

---

## How tuning fits in

When you run `--tune --install`:

1. Bayesian optimization tries many parameter sets in memory.
2. Best params are written as a **small patch** (`ilos_tuned_overlay.yaml`).
3. With `--install`, that patch is copied to `config/ilos_tuned_overlay.yaml`.
4. On **every future run**, that file is merged automatically (step 3 in the table above).

You do not pass `--tune` again for normal runs — the tuned overlay is picked up if the file exists.

---

## Mental model

```
┌─────────────────────────────────────────────────┐
│  ilos_base.yaml          (algorithm + mission)  │
├─────────────────────────────────────────────────┤
│  platform overlay        (sim / boat / bench)   │
├─────────────────────────────────────────────────┤
│  ilos_tuned_overlay.yaml (optional BO params)   │
├─────────────────────────────────────────────────┤
│  --overlay file          (optional extra patch) │
├─────────────────────────────────────────────────┤
│  runtime origin lat/lon  (first GPS fix)        │
└─────────────────────────────────────────────────┘
                    ↓
           one merged config dict
                    ↓
        written to run_dir/config.yaml
                    ↓
           stack_runner.py reads it
```

**Overlays are partial YAML files.** They never duplicate the whole config — they only specify keys they want to override. Later layers win on conflicts; untouched keys inherit from earlier layers.
