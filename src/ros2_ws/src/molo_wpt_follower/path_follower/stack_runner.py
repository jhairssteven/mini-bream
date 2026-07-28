#!/usr/bin/env python3
"""Unified path-follower stack: configurable controller + optional helpers."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
for path in (str(PKG_DIR), str(MOLO_DIR), str(MPC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import build_path_follower_config, controller_stack_module  # noqa: E402


def _write_temp_config(cfg: dict) -> Path:
    tmp = Path(tempfile.mkdtemp(prefix="path_follower_"))
    out = tmp / "config.yaml"
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a path follower with a pluggable controller")
    parser.add_argument(
        "--controller",
        choices=["ilos", "h0", "mpc"],
        default="ilos",
        help="Controller backend (h0 and mpc both use MpcFollowerNode)",
    )
    parser.add_argument(
        "--platform",
        choices=["sim", "real", "bench", "mock"],
        default="sim",
        help="Platform profile (HAL topic overlays)",
    )
    parser.add_argument(
        "--config",
        default="",
        help="Optional extra YAML overlay merged after defaults",
    )
    parser.add_argument(
        "--internal-path",
        action="store_true",
        help="Use YAML trajectory instead of external /plan topic",
    )
    parser.add_argument("--config-out", default="", help="Write merged config and exit")
    args = parser.parse_args()

    overlay = args.config or None
    cfg = build_path_follower_config(
        controller=args.controller,
        platform=args.platform,
        use_external_path=not args.internal_path,
        overlay_path=overlay,
    )

    if args.config_out:
        out = Path(args.config_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False)
        print(f"Wrote {out}")
        return

    config_path = _write_temp_config(cfg)
    stack = controller_stack_module(args.controller)
    stack_script = MOLO_DIR / stack / "stack_runner.py"
    if not stack_script.is_file():
        raise FileNotFoundError(f"stack runner not found: {stack_script}")

    cmd = [sys.executable, str(stack_script), "--config", str(config_path)]
    env = os.environ.copy()
    if os.environ.get("MOLO_USE_SIM_TIME", "").lower() in ("1", "true", "yes"):
        env["MOLO_USE_SIM_TIME"] = "1"
    raise SystemExit(subprocess.call(cmd, env=env))


if __name__ == "__main__":
    main()
