#!/usr/bin/env python3
"""Repeat A/B sim trials (legacy vs patched structure) and average metrics."""
from __future__ import annotations

import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(PKG))

from config import DEFAULT_RESULTS  # noqa: E402
from trial_runner import score_trial_from_artifacts  # noqa: E402

N = 7
OVERLAYS = {
    "baseline_legacy": PKG / "config" / "gs_ab_baseline_legacy.yaml",
    "patched": PKG / "config" / "gs_ab_patched_structure.yaml",
}
OUT_ROOT = PKG / "results" / "ab_repeat7"
SKIP_S = 5.0


def _kill_stack() -> None:
    subprocess.run(
        ["bash", "-lc", 'for p in $(pgrep -f "/ground_speed_mpc_ilos/stack_runner.py" || true); do kill "$p" 2>/dev/null || true; done'],
        check=False,
    )
    time.sleep(1.0)


def _newest_field_run(before: set[str]) -> Path:
    root = DEFAULT_RESULTS
    dirs = sorted(root.glob("20*/GS_MPC_ILOS"), key=lambda p: p.stat().st_mtime, reverse=True)
    for d in dirs:
        ts = d.parent.name
        if ts not in before:
            return d
    if not dirs:
        raise RuntimeError(f"no runs under {root}")
    return dirs[0]


def _extras(run: Path) -> dict:
    rows = list(csv.DictReader(open(run / "log.csv", encoding="utf-8")))
    t0 = float(rows[0]["t"])
    rows = [r for r in rows if float(r["t"]) - t0 >= SKIP_S]
    ug = np.array([float(r["u_g"]) for r in rows])
    ud = np.array([float(r["u_cmd"]) for r in rows])
    X = np.array([float(r["X"]) for r in rows])
    u = np.array([float(r["u"]) for r in rows])
    xte = np.array([float(r["xte"]) for r in rows])
    return {
        "ug_std": float(ug.std()),
        "frac_ug_neg": float((ug < 0).mean()),
        "frac_overspeed": float((ug > ud + 0.15).mean()),
        "frac_X_neg_fwd": float(((X < 0) & (ud > 0.05)).mean()),
        "x_sign_flips": int((np.diff(np.sign(X)) != 0).sum()),
        "xte_mean": float(np.abs(xte).mean()),
        "X_min": float(X.min()),
        "frac_u_neg": float((u < 0).mean()),
    }


def run_one(label: str, overlay: Path, trial_i: int) -> dict:
    _kill_stack()
    before = {p.parent.name for p in DEFAULT_RESULTS.glob("20*/GS_MPC_ILOS")}
    print(f"\n===== {label} trial {trial_i}/{N} =====", flush=True)
    rc = subprocess.call(
        [
            sys.executable,
            str(PKG / "run_gs_experiment.py"),
            "--platform",
            "sim",
            "--overlay",
            str(overlay),
            "--no-plot",
        ],
        cwd=str(PKG),
    )
    run = _newest_field_run(before)
    m = score_trial_from_artifacts(run, skip_initial_s=SKIP_S, min_completion=0.8)
    e = _extras(run)
    row = {
        "label": label,
        "trial": trial_i,
        "run": str(run),
        "exit_code": rc,
        **m,
        **e,
    }
    print(
        f"  -> rmse={row['rmse']:.4f} completion={row['completion']:.1%} "
        f"reverse={row['reverse_frac']:.1%} valid={row['valid']} ug_std={row['ug_std']:.3f}",
        flush=True,
    )
    return row


def summarize(rows: list[dict]) -> dict:
    keys = [
        "rmse",
        "completion",
        "reverse_frac",
        "ug_std",
        "frac_overspeed",
        "frac_X_neg_fwd",
        "x_sign_flips",
        "xte_mean",
        "X_min",
    ]
    out: dict = {"n": len(rows), "n_valid": sum(1 for r in rows if r.get("valid"))}
    for k in keys:
        vals = np.array([float(r[k]) for r in rows], dtype=float)
        out[k] = {
            "mean": float(vals.mean()),
            "std": float(vals.std(ddof=1)) if len(vals) > 1 else 0.0,
            "min": float(vals.min()),
            "max": float(vals.max()),
        }
    return out


def main() -> int:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict] = []
    # Interleave: baseline, patched, ... to reduce time-correlated bias
    for i in range(1, N + 1):
        for label, overlay in OVERLAYS.items():
            all_rows.append(run_one(label, overlay, i))
            with open(OUT_ROOT / "trials.jsonl", "a", encoding="utf-8") as f:
                f.write(json.dumps(all_rows[-1]) + "\n")

    summary = {
        label: summarize([r for r in all_rows if r["label"] == label])
        for label in OVERLAYS
    }
    # deltas patched - baseline means
    b, p = summary["baseline_legacy"], summary["patched"]
    deltas = {}
    for k in ("rmse", "completion", "reverse_frac", "ug_std", "frac_overspeed", "frac_X_neg_fwd", "x_sign_flips", "xte_mean"):
        deltas[k] = float(p[k]["mean"] - b[k]["mean"])

    report = {
        "n_repeats": N,
        "overlays": {k: str(v) for k, v in OVERLAYS.items()},
        "summary": summary,
        "delta_patched_minus_baseline": deltas,
        "worked": bool(
            p["reverse_frac"]["mean"] < b["reverse_frac"]["mean"]
            and p["completion"]["mean"] >= b["completion"]["mean"]
            and p["ug_std"]["mean"] < b["ug_std"]["mean"]
        ),
        "trials": all_rows,
    }
    out_path = OUT_ROOT / "summary.json"
    out_path.write_text(json.dumps(report, indent=2))
    print("\n========== REPEAT-7 SUMMARY ==========", flush=True)
    for label in OVERLAYS:
        s = summary[label]
        print(
            f"{label}: valid={s['n_valid']}/{s['n']}  "
            f"rmse={s['rmse']['mean']:.4f}±{s['rmse']['std']:.4f}  "
            f"completion={s['completion']['mean']:.1%}±{s['completion']['std']:.1%}  "
            f"reverse={s['reverse_frac']['mean']:.2%}±{s['reverse_frac']['std']:.2%}  "
            f"ug_std={s['ug_std']['mean']:.3f}±{s['ug_std']['std']:.3f}",
            flush=True,
        )
    print(f"deltas (patched-baseline): {deltas}", flush=True)
    print(f"worked={report['worked']}  -> {out_path}", flush=True)
    return 0 if report["worked"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
