#!/usr/bin/env python3
"""Unit tests for tune_seed warm-start helpers (no ROS required)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import yaml

from tune_seed import (
    gs_params_from_overlay,
    h0_params_from_overlay,
    ilos_params_from_overlay,
    load_seed_params,
    resolve_seed_artifact,
)


class TuneSeedTests(unittest.TestCase):
    def test_gs_overlay_roundtrip_keys(self):
        overlay = {
            "path": {"cruise_speed_mps": 0.3},
            "surge_pi": {"kp": 40.0, "ki": 6.0, "k_ff": 35.0},
            "yaw_pd": {"kp": 30.0, "kd": 10.0, "n_max": 40.0},
            "ilos": {"delta_m": 1.5, "sigma": 0.05, "e_i_limit": 2.0},
            "gs_mpc": {"horizon": 12, "w_path": 50.0, "u_max": 0.4},
            "speed": {
                "angle_threshold_fast_rad": 0.15,
                "angle_threshold_slow_rad": 0.5,
                "min_speed_scale": 0.4,
                "u_d_floor_frac_of_cruise": 0.75,
            },
        }
        p = gs_params_from_overlay(overlay)
        self.assertEqual(p["cruise"], 0.3)
        self.assertEqual(p["surge_kp"], 40.0)
        self.assertEqual(p["surge_kff"], 35.0)
        self.assertEqual(p["horizon"], 12)
        self.assertEqual(p["ilos_delta"], 1.5)

    def test_load_from_staged_summary_dir(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = {
                "phases": {
                    "inner": {"best_params": {"surge_kp": 41.0, "cruise": 0.28}},
                    "global": {"best_params": {"w_path": 55.0, "horizon": 11}},
                }
            }
            (root / "staged_summary.json").write_text(json.dumps(summary), encoding="utf-8")
            p = load_seed_params(root, "gs")
            self.assertEqual(p["surge_kp"], 41.0)
            self.assertEqual(p["w_path"], 55.0)
            self.assertEqual(p["horizon"], 11)
            self.assertIn("staged_summary.json", p["_seed_artifact"])

    def test_h0_resolve_prefers_tune_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "tune_result.json").write_text(
                json.dumps({"best_params": {"cruise": 0.22, "pid_kp": 1.5, "horizon": 14}}),
                encoding="utf-8",
            )
            (root / "h0_tuned_overlay.yaml").write_text(
                yaml.safe_dump({"path": {"cruise_speed_mps": 0.99}}),
                encoding="utf-8",
            )
            art = resolve_seed_artifact(root, "h0")
            self.assertEqual(art.name, "tune_result.json")
            p = load_seed_params(root, "h0")
            self.assertEqual(p["cruise"], 0.22)
            self.assertEqual(p["horizon"], 14)

    def test_ilos_from_overlay_yaml(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ilos_tuned_overlay.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "path": {"cruise_speed_mps": 0.25},
                        "ilos": {
                            "lookahead_min_m": 0.5,
                            "lookahead_max_m": 1.5,
                            "conv_rate": 8.0,
                        },
                        "pid": {"kp": 1.2, "kd": 0.4},
                        "thrust": {
                            "yaw_mix_gain": 0.8,
                            "surge_gain": 1.0,
                            "min_surge_norm": 0.1,
                        },
                        "speed": {
                            "angle_threshold_fast_rad": 0.1,
                            "angle_threshold_slow_rad": 0.4,
                            "min_speed_scale": 0.3,
                        },
                        "guidance": {
                            "max_yaw_rate_rad_s": 0.6,
                            "kappa_speed_scale": 2.0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            p = load_seed_params(path, "ilos")
            self.assertEqual(p["cruise"], 0.25)
            self.assertEqual(p["ilos_min"], 0.5)
            self.assertEqual(p["kappa_scale"], 2.0)
            self.assertEqual(
                ilos_params_from_overlay(yaml.safe_load(path.read_text()))["yaw_mix"], 0.8
            )
            self.assertEqual(h0_params_from_overlay({}), {})


if __name__ == "__main__":
    unittest.main()
