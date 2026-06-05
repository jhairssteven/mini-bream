#!/usr/bin/env python3
"""Generate publication figures for mpc/report from experiment JSON and logs."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
MPC = ROOT.parent
FIG = ROOT / "figures"
VAL = MPC / "experiments/results/lemniscate_validation/validation_summary.json"
TARGET = 0.1


def load_json(p: Path) -> dict:
    return json.loads(p.read_text(encoding="utf-8"))


def fig_validation_bar(top_n: int = 6):
    d = load_json(VAL)
    ranked = d["ranked"][:top_n]
    ids = [r["hypothesis_id"] for r in ranked]
    rmses = [r["validation_rmse"] for r in ranked]
    y = np.arange(len(ids))
    colors = ["#2ca02c" if v < TARGET else "#d62728" for v in rmses]
    fig, ax = plt.subplots(figsize=(8, max(4, len(ids) * 0.45)))
    ax.barh(y, rmses, color=colors, alpha=0.85)
    ax.axvline(TARGET, color="k", ls="--", label=f"Target {TARGET} m")
    ax.set_yticks(y)
    ax.set_yticklabels(ids, fontsize=9)
    ax.invert_yaxis()
    ax.set_xlabel("XTE RMSE [m]")
    ax.set_title(f"Lemniscate validation: top {len(ids)} (fixed reference path)")
    for i, v in enumerate(rmses):
        ax.text(v + 0.008, i, f"{v:.3f}", va="center", fontsize=8)
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(True, axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / "validation_bar.pdf")
    fig.savefig(FIG / "validation_bar.png", dpi=150)
    plt.close(fig)


def export_run_plots(hid: str, stem: str | None = None) -> bool:
    """Copy per-run plots.png (3x2 diagnostics) into report/figures as PDF+PNG."""
    src = MPC / f"experiments/results/lemniscate_validation/{hid}/plots.png"
    if not src.exists():
        return False
    out = stem or f"best_{hid}"
    img = plt.imread(src)
    h, w = img.shape[:2]
    fig, ax = plt.subplots(figsize=(w / 120, h / 120))
    ax.imshow(img)
    ax.axis("off")
    fig.savefig(FIG / f"{out}.pdf", bbox_inches="tight", pad_inches=0.02)
    fig.savefig(FIG / f"{out}.png", dpi=150, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    return True


def fig_compare_top5(exclude: frozenset[str] = frozenset({"H5_geometric_ff"}), top_n: int = 6):
    """Plot top-N ranked approaches minus exclusions (no backfill)."""
    d = load_json(VAL)
    ranked_top = [r["hypothesis_id"] for r in d["ranked"][:top_n]]
    hypos = [h for h in ranked_top if h not in exclude]
    fig, ax = plt.subplots(figsize=(6, 4))
    for hid in hypos:
        log = MPC / f"experiments/results/lemniscate_validation/{hid}/log.csv"
        if not log.exists():
            continue
        data = np.genfromtxt(log, delimiter=",", names=True)
        ax.plot(data["t"], data["xte"], label=hid)
    ax.axhline(TARGET, color="k", ls="--", label="Target")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|XTE| [m]")
    ax.set_title(f"Top-{len(hypos)} approaches: XTE time series (fixed lemniscate)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / "xte_top6.pdf")
    fig.savefig(FIG / "xte_top6.png", dpi=150)
    plt.close(fig)


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    if not VAL.exists():
        print(f"Missing {VAL}")
        return
    summary = load_json(VAL)
    best = summary.get("best", "H0_baseline")
    fig_validation_bar(top_n=6)
    export_run_plots(best)
    for hid in ("H1_curvature_ff", "H1b_tuned_ff"):
        export_run_plots(hid, stem=f"diag_{hid}")
    fig_compare_top5()
    src_bar = MPC / "experiments/results/lemniscate_validation/validation_bar.png"
    if src_bar.exists():
        shutil.copy2(src_bar, FIG / "validation_bar_full.png")
    print(f"Figures written to {FIG} (best={best})")


if __name__ == "__main__":
    main()
