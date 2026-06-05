#!/usr/bin/env python3
"""Print ranked RMSE table from campaign_results.json."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("campaign_dir", type=Path)
    args = p.parse_args()
    data = json.loads((args.campaign_dir / "campaign_results.json").read_text(encoding="utf-8"))
    target = float(data.get("target_rmse_m", 0.1))
    runs = data.get("runs", [])
    print(f"Target RMSE < {target} m\n")
    print(f"{'Hypothesis':<22} {'Trajectory':<12} {'RMSE':>8} {'PASS':>6}")
    print("-" * 52)
    for r in sorted(runs, key=lambda x: (x.get("rmse", 1e9), x["hypothesis_id"])):
        rmse = r.get("rmse", float("nan"))
        ok = "yes" if r.get("pass") else "no"
        print(f"{r['hypothesis_id']:<22} {r['trajectory']:<12} {rmse:8.4f} {ok:>6}")
    print("\n--- Mean RMSE per hypothesis (circle + lemniscate) ---")
    by_h: dict = {}
    for r in runs:
        hid = r["hypothesis_id"]
        by_h.setdefault(hid, []).append(r.get("rmse", float("nan")))
    import math

    ranked = []
    for hid, vals in by_h.items():
        finite = [v for v in vals if math.isfinite(v)]
        mean = sum(finite) / len(finite) if finite else float("nan")
        ranked.append((mean, hid))
    for mean, hid in sorted(ranked):
        print(f"  {hid:<22} mean={mean:.4f}")


if __name__ == "__main__":
    main()
