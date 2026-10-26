#!/usr/bin/env python3
"""Plot GS-MPC+ILOS run logs (trajectory, XTE, speed, thrust, inner loops)."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path) -> Optional[Dict[str, np.ndarray]]:
    if not path.is_file() or path.stat().st_size < 80:
        return None
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if raw.size == 0:
        return None
    out: Dict[str, np.ndarray] = {}
    for name in raw.dtype.names or ():
        col = np.asarray(raw[name])
        try:
            out[name] = col.astype(float)
        except (ValueError, TypeError):
            continue
    return out if "t" in out else None


def load_ref(path: Path):
    if not path.is_file():
        return None, None
    arr = np.genfromtxt(path, delimiter=",", skip_header=1)
    return arr[:, 0], arr[:, 1]


def _unwrap(angles: np.ndarray) -> np.ndarray:
    return np.unwrap(np.asarray(angles, dtype=float))


def _angdiff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def plot_inner_loops(run_dir: Path, d: Dict[str, np.ndarray]) -> Path:
    """Dedicated surge PI + yaw PD performance figure."""
    out_dir = run_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    t = d["t"]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle("Inner loops: surge PI (top) / yaw PD (bottom)", fontsize=12)

    # --- Surge: commands vs measured ---
    ax = axes[0, 0]
    if "u_cmd" in d:
        ax.plot(t, d["u_cmd"], "--", lw=1.3, label="U_d (ILOS)")
    if "u_mpc" in d:
        ax.plot(t, d["u_mpc"], ":", lw=1.2, label="U_MPC")
    if "u_g" in d:
        ax.plot(t, d["u_g"], lw=1.0, label="U_g measured")
    ax.axhline(0.0, color="0.5", lw=0.6)
    ax.set_ylabel("m/s")
    ax.set_title("Surge tracking")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)

    # --- Surge: error + force ---
    ax = axes[0, 1]
    if "u_cmd" in d and "u_g" in d:
        e_u = d["u_cmd"] - d["u_g"]
        ax.plot(t, e_u, "C0-", lw=1.0, label="e_u = U_d − U_g")
        ax.axhline(0.0, color="k", ls="--", lw=0.8)
        ax.set_ylabel("speed err [m/s]", color="C0")
        ax.tick_params(axis="y", labelcolor="C0")
        rmse_u = float(np.sqrt(np.mean(e_u * e_u)))
        ax.set_title(f"Surge error / force  (RMSE_u={rmse_u:.3f} m/s)")
    if "X" in d:
        ax2 = ax.twinx()
        ax2.plot(t, d["X"], "C3-", lw=1.0, alpha=0.85, label="X surge [N]")
        ax2.set_ylabel("X [N]", color="C3")
        ax2.tick_params(axis="y", labelcolor="C3")
        lines1, lab1 = ax.get_legend_handles_labels()
        lines2, lab2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc="best")
    else:
        ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)

    # --- Yaw: heading command vs measured ---
    ax = axes[1, 0]
    if "psi_d" in d and "psi" in d:
        psi_u = _unwrap(d["psi"])
        psid_u = _unwrap(d["psi_d"])
        # align absolute offset for readability (same start)
        off = psi_u[0] - psid_u[0]
        ax.plot(t, np.degrees(psid_u + off), "--", lw=1.3, label="ψ_d (ILOS)")
        ax.plot(t, np.degrees(psi_u), lw=1.0, label="ψ measured")
        if "r" in d:
            ax2 = ax.twinx()
            ax2.plot(t, d["r"], "C2:", lw=1.0, alpha=0.8, label="r [rad/s]")
            ax2.set_ylabel("r [rad/s]", color="C2")
            ax2.tick_params(axis="y", labelcolor="C2")
            lines1, lab1 = ax.get_legend_handles_labels()
            lines2, lab2 = ax2.get_legend_handles_labels()
            ax.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc="best")
        else:
            ax.legend(fontsize=8, loc="best")
    ax.set_ylabel("heading [deg]")
    ax.set_xlabel("t [s]")
    ax.set_title("Yaw tracking")
    ax.grid(True, alpha=0.3)

    # --- Yaw: heading error + moment ---
    ax = axes[1, 1]
    if "psi_d" in d and "psi" in d:
        e_psi = _angdiff(d["psi_d"], d["psi"])
        ax.plot(t, np.degrees(e_psi), "C0-", lw=1.0, label="e_ψ = ψ_d − ψ")
        ax.axhline(0.0, color="k", ls="--", lw=0.8)
        ax.set_ylabel("heading err [deg]", color="C0")
        ax.tick_params(axis="y", labelcolor="C0")
        rmse_yaw = float(np.sqrt(np.mean(e_psi * e_psi)))
        ax.set_title(f"Yaw error / moment  (RMSE_ψ={math.degrees(rmse_yaw):.2f}°)")
    if "N" in d:
        ax2 = ax.twinx()
        ax2.plot(t, d["N"], "C3-", lw=1.0, alpha=0.85, label="N yaw [N·m]")
        ax2.set_ylabel("N [N·m]", color="C3")
        ax2.tick_params(axis="y", labelcolor="C3")
        lines1, lab1 = ax.get_legend_handles_labels()
        lines2, lab2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc="best")
    else:
        ax.legend(fontsize=8, loc="best")
    ax.set_xlabel("t [s]")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = out_dir / "inner_loops.png"
    fig.savefig(out, dpi=140)
    fig.savefig(run_dir / "inner_loops.png", dpi=140)
    plt.close(fig)
    return out


def plot_run(run_dir: Path, rmse: float, target: float = 0.1) -> Path:
    d = load_csv(run_dir / "log.csv")
    if d is None:
        raise FileNotFoundError(f"no usable log in {run_dir}")

    rx, ry = load_ref(run_dir / "ref_path.csv")
    out_dir = run_dir / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Metrics for titles
    surge_rmse = float("nan")
    yaw_rmse_deg = float("nan")
    if "u_cmd" in d and "u_g" in d:
        e_u = d["u_cmd"] - d["u_g"]
        surge_rmse = float(np.sqrt(np.mean(e_u * e_u)))
    if "psi_d" in d and "psi" in d:
        e_psi = _angdiff(d["psi_d"], d["psi"])
        yaw_rmse_deg = float(math.degrees(np.sqrt(np.mean(e_psi * e_psi))))

    fig, axes = plt.subplots(3, 2, figsize=(12, 12))
    fig.suptitle(
        f"GS_MPC_ILOS lemniscate  XTE RMSE={rmse:.4f} m  (target {target} m)\n"
        f"surge RMSE={surge_rmse:.3f} m/s   yaw RMSE={yaw_rmse_deg:.2f}°",
        fontsize=12,
    )

    ax = axes[0, 0]
    if rx is not None:
        ax.plot(rx, ry, "g-", lw=2, label="reference")
    ax.plot(d["x"], d["y"], "b-", alpha=0.85, label="traversed")
    ax.plot(d["x"][0], d["y"][0], "ko", ms=6, label="start")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory (world)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(d["t"], np.abs(d["xte"]), "r-", lw=1)
    ax.axhline(target, color="k", ls="--", label=f"target {target} m")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|XTE| [m]")
    ax.set_title("Cross-track error")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Surge inner loop
    ax = axes[1, 0]
    if "u_cmd" in d:
        ax.plot(d["t"], d["u_cmd"], "--", lw=1.2, label="U_d (ILOS)")
    if "u_mpc" in d:
        ax.plot(d["t"], d["u_mpc"], ":", lw=1.2, label="U_MPC")
    if "u_g" in d:
        ax.plot(d["t"], d["u_g"], lw=1, label="U_g measured")
    elif "u" in d:
        ax.plot(d["t"], d["u"], lw=1, label="u body")
    ax.axhline(0.0, color="0.5", lw=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m/s")
    ax.set_title("Inner surge: command vs measured")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    if "u_cmd" in d and "u_g" in d:
        ax.plot(d["t"], d["u_cmd"] - d["u_g"], "C0-", lw=1, label="e_u = U_d−U_g")
        ax.axhline(0.0, color="k", ls="--", lw=0.8)
    if "X" in d:
        ax2 = ax.twinx()
        ax2.plot(d["t"], d["X"], "C3-", lw=1.0, alpha=0.8, label="X [N]")
        ax2.set_ylabel("X [N]", color="C3")
        ax2.tick_params(axis="y", labelcolor="C3")
        lines1, lab1 = ax.get_legend_handles_labels()
        lines2, lab2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc="best")
    else:
        ax.legend(fontsize=8)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("speed err [m/s]")
    ax.set_title("Inner surge: error + force X")
    ax.grid(True, alpha=0.3)

    # Yaw inner loop
    ax = axes[2, 0]
    if "psi_d" in d and "psi" in d:
        psi_u = _unwrap(d["psi"])
        psid_u = _unwrap(d["psi_d"])
        off = psi_u[0] - psid_u[0]
        ax.plot(d["t"], np.degrees(psid_u + off), "--", lw=1.2, label="ψ_d")
        ax.plot(d["t"], np.degrees(psi_u), lw=1, label="ψ")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("heading [deg]")
    ax.set_title("Inner yaw: command vs measured")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    if "psi_d" in d and "psi" in d:
        e_psi = _angdiff(d["psi_d"], d["psi"])
        ax.plot(d["t"], np.degrees(e_psi), "C0-", lw=1, label="e_ψ [deg]")
        ax.axhline(0.0, color="k", ls="--", lw=0.8)
    if "N" in d:
        ax2 = ax.twinx()
        ax2.plot(d["t"], d["N"], "C3-", lw=1.0, alpha=0.8, label="N [N·m]")
        ax2.set_ylabel("N [N·m]", color="C3")
        ax2.tick_params(axis="y", labelcolor="C3")
        lines1, lab1 = ax.get_legend_handles_labels()
        lines2, lab2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, lab1 + lab2, fontsize=8, loc="best")
    else:
        ax.legend(fontsize=8)
    if "thrust_l_N" in d:
        # light thruster overlay on a third feel — keep yaw panel clean; thrusters below title note
        pass
    ax.set_xlabel("t [s]")
    ax.set_ylabel("heading err [deg]")
    ax.set_title("Inner yaw: error + moment N")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = out_dir / "overview.png"
    fig.savefig(out, dpi=140)
    fig.savefig(run_dir / "plots.png", dpi=140)
    plt.close(fig)

    # Also thrusters-only small companion in overview folder
    if "thrust_l_N" in d:
        fig2, ax = plt.subplots(figsize=(10, 3.2))
        ax.plot(d["t"], d["thrust_l_N"], label="left cmd (norm×1000)")
        ax.plot(d["t"], d["thrust_r_N"], label="right cmd (norm×1000)")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("thrust cmd")
        ax.set_title("Thruster commands")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        fig2.tight_layout()
        fig2.savefig(out_dir / "thrusters.png", dpi=120)
        plt.close(fig2)

    plot_inner_loops(run_dir, d)
    return out


def main() -> None:
    p = argparse.ArgumentParser(description="Plot GS-MPC+ILOS results")
    p.add_argument(
        "results_root",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parent / "results",
    )
    p.add_argument("--only", default=None, help="Only plot this timestamp dir name")
    args = p.parse_args()

    root = args.results_root
    dirs = sorted(root.iterdir()) if root.is_dir() else []
    for d in dirs:
        if not d.is_dir():
            continue
        if args.only and d.name != args.only:
            continue
        run = d / "GS_MPC_ILOS"
        if not (run / "log.csv").is_file():
            continue
        meta_path = run / "result.json"
        rmse = float("nan")
        target = 0.1
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text())
            rmse = float(meta.get("rmse", float("nan")))
            target = float(meta.get("target_rmse_m", 0.1))
        out = plot_run(run, rmse, target)
        print(f"{d.name}: RMSE={rmse:.4f} m -> {out}")
        print(f"  inner loops -> {run / 'inner_loops.png'}")


if __name__ == "__main__":
    main()
