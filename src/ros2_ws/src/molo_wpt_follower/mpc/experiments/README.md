# MPC experiment campaign (scientific methodology)

## Workflow

1. **Hypothesis** — documented in `hypotheses.yaml` (prediction + config patch).
2. **Test** — open-water Gazebo, curved trajectories, logged CSV per run.
3. **Analyze** — `campaign_results.json`, per-run `plots.png`, heatmaps.
4. **Refine** — update default `params.yaml` from best hypothesis.

## Run full campaign (Docker)

```bash
source /opt/ros/humble/setup.bash
source /workspace/codebase/vrx_ws/install/setup.bash
source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash

cd .../molo_wpt_follower/mpc
python3 experiments/run_campaign.py
```

Subset:

```bash
python3 experiments/run_campaign.py --hypothesis H1_curvature_ff H3_aggressive_ff
```

Re-plot only:

```bash
python3 experiments/run_campaign.py --plot-only experiments/results/<timestamp>
```

## Outputs per run

```
experiments/results/<timestamp>/
  hypotheses.yaml
  campaign_results.json
  campaign_heatmap.png
  curved_comparison.png
  H1_curvature_ff/circle/
    config.yaml
    meta.json
    ref_path.csv
    log.csv
    plots.png
```

## Manual test (one tuned approach)

See **`experiments/MANUAL_TEST.md`** for step-by-step Gazebo + RViz + MPC.

```bash
./experiments/run_tuned_approach.sh --list
./experiments/run_tuned_approach.sh H9_slow_tight
```

## RViz (single run)

```bash
ros2 launch .../open_water.launch.py
python3 mpc.py --config experiments/results/tune_lemniscate_full/H9_slow_tight/best_config.yaml
rviz2 -d molo_mpc.rviz
```

## Benchmark suite (circle + lemniscate)

Default `hypotheses.yaml` runs **16 hypotheses** × 2 trajectories. See `approaches.yaml` for the full registry.

```bash
python3 experiments/run_campaign.py
python3 experiments/run_campaign.py --hypothesis H1_curvature_ff H12_smc_ilos
```

## Bayesian tuning (lemniscate only, fair comparison)

Tunes **all hypotheses** with the same open parameter set (path, Dubins, ILOS, PID, guidance, MPC) while keeping each `control.approach` fixed:

```bash
# Full (~28 BO trials × 16 hypotheses, hours in Docker)
python3 experiments/tune_lemniscate.py

# Smoke test
python3 experiments/tune_lemniscate.py --quick --hypothesis H1_curvature_ff

# Subset
python3 experiments/tune_lemniscate.py --hypothesis H9_slow_tight H10_lateral_vel H1_curvature_ff --n-calls 25
```

Outputs: `experiments/results/tune_lemniscate_<timestamp>/`
- `<H_id>/tune_result.json`, `best_config.yaml`, `best_validation/`
- `tune_summary.json` — ranked validation RMSE

Requires: `pip install scikit-optimize`

## Approaches (`control.approach`)

| Value | Description |
|-------|-------------|
| `baseline_ilos_velocity` | Legacy ILOS + tiled velocity refs |
| `curvature_ff_ilos` | r=uκ preview + ILOS heading PID |
| `stanley` | Stanley steering + preview |
| `aggressive_ff` | High gain + lateral v_ref |
| `smc_ilos` / `smc_geometric` | Sliding-mode guidance + velocity MPC |
| `frenet_ff` / `backstepping` / `contouring` | Geometric nonlinear laws + MPC |
