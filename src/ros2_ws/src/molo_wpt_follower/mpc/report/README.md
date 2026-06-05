# MPC experiment technical report (LaTeX)

Self-contained report of the circle/lemniscate benchmark and lemniscate Bayesian optimization.

## Build

```bash
cd molo_wpt_follower/mpc/report
python3 generate_figures.py   # reads ../experiments/results/
make pdf                      # produces main.pdf
```

Requires: `pdflatex`, `matplotlib`, experiment results under `experiments/results/`.

## Contents

- `main.tex` — full report (hypotheses, methodology, tables, discussion)
- `generate_figures.py` — figures from `benchmark_clean_v2` and `tune_lemniscate_full`
- `figures/` — generated PDF/PNG plots

## Key result

**H9_slow_tight** after BO: lemniscate validation RMSE **0.109 m** (target 0.1 m).
