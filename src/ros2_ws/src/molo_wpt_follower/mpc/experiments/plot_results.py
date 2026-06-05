#!/usr/bin/env python3
"""Plot experiment logs and campaign summaries (matplotlib, Agg backend)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore


def load_csv_log(path: Path) -> Optional[Dict[str, np.ndarray]]:
    if path.stat().st_size < 80:
        return None
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        return None
    return {n: np.asarray(data[n], dtype=float) for n in data.dtype.names}


def load_ref_path(path: Path) -> tuple[np.ndarray, np.ndarray]:
    arr = np.genfromtxt(path, delimiter=",", skip_header=1)
    return arr[:, 0], arr[:, 1]


def _thrust_max_n(run_dir: Path) -> float:
    cfg = run_dir / "config.yaml"
    if yaml is not None and cfg.is_file():
        boat = yaml.safe_load(cfg.read_text(encoding="utf-8")).get("boat", {})
        return float(boat.get("max_thrust_N", 250.0))
    return 250.0


def _has_mpc_tracking_cols(d: Dict[str, np.ndarray]) -> bool:
    return all(k in d for k in ("u_mpc_ref", "v_mpc_ref", "r_mpc_ref", "thrust_l_N", "thrust_r_N"))


def plot_run(
    run_dir: Path,
    hypothesis_id: str,
    trajectory: str,
    rmse: float,
    target: float,
) -> None:
    log = run_dir / "log.csv"
    ref = run_dir / "ref_path.csv"
    if not log.exists():
        return
    d = load_csv_log(log)
    if d is None:
        return

    if _has_mpc_tracking_cols(d):
        _plot_run_extended(run_dir, hypothesis_id, trajectory, rmse, target, d, ref)
    else:
        _plot_run_legacy(hypothesis_id, trajectory, rmse, target, d, ref, run_dir)


def _plot_run_extended(
    run_dir: Path,
    hypothesis_id: str,
    trajectory: str,
    rmse: float,
    target: float,
    d: Dict[str, np.ndarray],
    ref: Path,
) -> None:
    tmax = _thrust_max_n(run_dir)
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(f"{hypothesis_id} / {trajectory}  XTE RMSE={rmse:.4f} m (target {target} m)")

    ax = axes[0, 0]
    if ref.exists():
        rx, ry = load_ref_path(ref)
        ax.plot(rx, ry, "g-", linewidth=2, label="reference")
    ax.plot(d["x"], d["y"], "b-", alpha=0.8, label="traversed")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory (world)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(d["t"], d["xte"], "r-", linewidth=1)
    ax.axhline(target, color="k", linestyle="--", label="target")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|XTE| [m]")
    ax.set_title("Cross-track error")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    vel_axes = [axes[1, 0], axes[1, 1], axes[2, 0]]
    labels = ("u", "v", "r")
    meas = ("u", "v", "r")
    refs = ("u_mpc_ref", "v_mpc_ref", "r_mpc_ref")
    units = ("m/s", "m/s", "rad/s")
    for ax, lbl, m, r, unit in zip(vel_axes, labels, meas, refs, units):
        ax.plot(d["t"], d[r], "--", label=f"{lbl}_mpc_ref", linewidth=1.2)
        ax.plot(d["t"], d[m], label=lbl, linewidth=1)
        ax.set_xlabel("t [s]")
        ax.set_ylabel(unit)
        ax.set_title(f"{lbl}: MPC ref vs measured")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(d["t"], d["thrust_l_N"], label="T_left [N]")
    ax.plot(d["t"], d["thrust_r_N"], label="T_right [N]")
    ax.axhline(tmax, color="r", linestyle="--", linewidth=1, label=f"+{tmax:.0f} N limit")
    ax.axhline(-tmax, color="r", linestyle="--", linewidth=1)
    sat_l = np.sum(np.abs(d["thrust_l_N"]) >= 0.99 * tmax)
    sat_r = np.sum(np.abs(d["thrust_r_N"]) >= 0.99 * tmax)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Thrust [N]")
    ax.set_title(f"Thruster commands (sat steps: L={sat_l}, R={sat_r})")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(run_dir / "plots.png", dpi=120)
    plt.close(fig)


def _plot_run_legacy(
    hypothesis_id: str,
    trajectory: str,
    rmse: float,
    target: float,
    d: Dict[str, np.ndarray],
    ref: Path,
    run_dir: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    fig.suptitle(f"{hypothesis_id} / {trajectory}  RMSE={rmse:.4f} m (target {target} m)")

    ax = axes[0, 0]
    if ref.exists():
        rx, ry = load_ref_path(ref)
        ax.plot(rx, ry, "g-", linewidth=2, label="reference")
    ax.plot(d["x"], d["y"], "b-", alpha=0.8, label="traversed")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory (world)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(d["t"], d["xte"], "r-", linewidth=1)
    ax.axhline(target, color="k", linestyle="--", label="target")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|XTE| [m]")
    ax.set_title("Cross-track error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    if "r_ref" in d:
        ax.plot(d["t"], d["r_ref"], label="r_ref")
    ax.plot(d["t"], d["r"], label="r")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("rad/s")
    ax.set_title("Yaw rate ref vs actual")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    if "v_ref" in d:
        ax.plot(d["t"], d["v_ref"], label="v_ref")
    ax.plot(d["t"], d["v"], label="v")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m/s")
    ax.set_title("Sway ref vs actual")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(run_dir / "plots.png", dpi=120)
    plt.close(fig)


def plot_campaign_summary(campaign_dir: Path, results: Dict[str, Any], target: float) -> None:
    trajs = results.get("runs", [])
    if not trajs:
        return
    by_h: Dict[str, List[dict]] = {}
    for r in trajs:
        by_h.setdefault(r["hypothesis_id"], []).append(r)

    hypo_ids = list(by_h.keys())
    traj_names = sorted({r["trajectory"] for r in trajs})
    mat = np.full((len(hypo_ids), len(traj_names)), np.nan)
    for i, hid in enumerate(hypo_ids):
        for r in by_h[hid]:
            j = traj_names.index(r["trajectory"])
            mat[i, j] = r.get("rmse", np.nan)

    fig, ax = plt.subplots(figsize=(max(8, len(traj_names) * 1.2), max(4, len(hypo_ids) * 0.5)))
    im = ax.imshow(mat, aspect="auto", cmap="RdYlGn_r", vmin=0, vmax=max(target * 3, 0.5))
    ax.set_xticks(range(len(traj_names)))
    ax.set_xticklabels(traj_names, rotation=35, ha="right")
    ax.set_yticks(range(len(hypo_ids)))
    ax.set_yticklabels(hypo_ids)
    ax.set_title(f"XTE RMSE [m] (target {target} m)")
    for i in range(len(hypo_ids)):
        for j in range(len(traj_names)):
            v = mat[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8)
    fig.colorbar(im, ax=ax, label="RMSE [m]")
    fig.tight_layout()
    fig.savefig(campaign_dir / "campaign_heatmap.png", dpi=130)
    plt.close(fig)

    curved = results.get("curved_trajectories", traj_names)
    fig, ax = plt.subplots(figsize=(10, 5))
    width = 0.8 / max(len(hypo_ids), 1)
    for i, hid in enumerate(hypo_ids):
        vals = []
        for tn in curved:
            row = next((x for x in by_h[hid] if x["trajectory"] == tn), None)
            vals.append(row["rmse"] if row else np.nan)
        x = np.arange(len(curved)) + i * width
        ax.bar(x, vals, width=width, label=hid)
    ax.axhline(target, color="k", linestyle="--", label="target")
    ax.set_xticks(np.arange(len(curved)) + width * (len(hypo_ids) - 1) / 2)
    ax.set_xticklabels(curved, rotation=20)
    ax.set_ylabel("RMSE [m]")
    ax.set_title("Curved trajectories — XTE RMSE by hypothesis")
    ax.legend(fontsize=8)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(campaign_dir / "curved_comparison.png", dpi=130)
    plt.close(fig)


def plot_all(campaign_dir: Path) -> None:
    summary_path = campaign_dir / "campaign_results.json"
    if not summary_path.exists():
        return
    with open(summary_path, encoding="utf-8") as f:
        results = json.load(f)
    target = float(results.get("target_rmse_m", 0.1))
    for run in results.get("runs", []):
        run_dir = Path(run["run_dir"])
        plot_run(
            run_dir,
            run["hypothesis_id"],
            run["trajectory"],
            float(run.get("rmse", float("nan"))),
            target,
        )
    plot_campaign_summary(campaign_dir, results, target)


def plot_validation_summary(validation_dir: Path, summary: Dict[str, Any], target: float) -> None:
    ranked = summary.get("ranked", [])
    if not ranked:
        return
    ids = [r["hypothesis_id"] for r in ranked]
    rmses = [float(r["validation_rmse"]) for r in ranked]

    fig, ax = plt.subplots(figsize=(10, max(4, len(ids) * 0.35)))
    y = np.arange(len(ids))
    colors = ["#2ca02c" if v < target else "#d62728" for v in rmses]
    ax.barh(y, rmses, color=colors, alpha=0.85)
    ax.axvline(target, color="k", linestyle="--", label=f"target {target} m")
    ax.set_yticks(y)
    ax.set_yticklabels(ids)
    ax.invert_yaxis()
    ax.set_xlabel("XTE RMSE [m]")
    ax.set_title("Lemniscate validation (fixed reference path)")
    for i, v in enumerate(rmses):
        ax.text(v + 0.01, i, f"{v:.3f}", va="center", fontsize=8)
    ax.legend(loc="lower right")
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(validation_dir / "validation_bar.png", dpi=130)
    plt.close(fig)


def plot_validation_all(validation_dir: Path) -> None:
    """Plot per-hypothesis runs from validation_summary.json layout."""
    summary_path = validation_dir / "validation_summary.json"
    if not summary_path.exists():
        return
    with open(summary_path, encoding="utf-8") as f:
        summary = json.load(f)
    target = float(summary.get("target_rmse_m", 0.1))
    for run in summary.get("runs", []):
        hid = run["hypothesis_id"]
        run_dir = validation_dir / hid
        if not run_dir.is_dir():
            continue
        rmse = float(run.get("rmse", float("nan")))
        plot_run(run_dir, hid, "lemniscate", rmse, target)
    plot_validation_summary(validation_dir, summary, target)


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("campaign_dir", type=Path)
    p.add_argument(
        "--validation",
        action="store_true",
        help="Use validation_summary.json layout (lemniscate_validation)",
    )
    args = p.parse_args()
    if args.validation:
        plot_validation_all(args.campaign_dir)
    else:
        plot_all(args.campaign_dir)
