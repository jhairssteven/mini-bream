"""Resolve previous-tuning artifacts into flat BO seed params.

Accepted inputs for ``--seed-from`` / ``--refine``:
  - a prior tune directory (auto-picks the best known artifact)
  - ``tune_result.json`` / ``staged_summary.json``
  - a tuned overlay or best_config YAML
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import yaml

# Preferred artifacts when --seed-from points at a directory.
CONTROLLER_CANDIDATES: Dict[str, List[str]] = {
    "gs": [
        "staged_summary.json",
        "phase_global/tune_result.json",
        "phase_global/global_overlay.yaml",
        "gs_tuned_overlay.yaml",
        "tune_result.json",
    ],
    "h0": [
        "tune_result.json",
        "h0_tuned_overlay.yaml",
        "best_config.yaml",
    ],
    "ilos": [
        "tune_result.json",
        "ilos_tuned_overlay.yaml",
        "best_config.yaml",
    ],
}


def _load_data(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        return json.loads(text)
    return yaml.safe_load(text) or {}


def resolve_seed_artifact(raw: Path | str, controller: str) -> Path:
    """Return a concrete seed file from a path or prior-tune directory."""
    path = Path(raw).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"seed-from path not found: {path}")
    if path.is_file():
        return path

    key = controller.lower().strip()
    candidates = CONTROLLER_CANDIDATES.get(key, [])
    for rel in candidates:
        cand = path / rel
        if cand.is_file():
            return cand

    # Fall back: any tune_result.json / *tuned_overlay.yaml one level down.
    for pattern in ("tune_result.json", "*_overlay.yaml", "best_config.yaml", "staged_summary.json"):
        hits = sorted(path.glob(pattern)) + sorted(path.glob(f"*/{pattern}"))
        if hits:
            return hits[0]

    raise FileNotFoundError(
        f"no seed artifact found under {path} for controller={controller!r}; "
        f"looked for {candidates}"
    )


def flat_params_from_jsonish(data: Any) -> Dict[str, Any]:
    """Pull flat best_params from tune_result / staged_summary structures."""
    if not isinstance(data, dict):
        return {}
    if isinstance(data.get("best_params"), dict) and data["best_params"]:
        return {str(k): v for k, v in data["best_params"].items()}

    phases = data.get("phases")
    if isinstance(phases, dict):
        out: Dict[str, Any] = {}
        # Prefer later phases (global overwrites earlier).
        for name in ("inner", "ilos", "mpc", "global"):
            block = phases.get(name) or {}
            bp = block.get("best_params") if isinstance(block, dict) else None
            if isinstance(bp, dict):
                out.update(bp)
        if out:
            return out
    return {}


def gs_params_from_overlay(cfg: dict) -> Dict[str, Any]:
    """Map a GS nested overlay/config into flat tune_gs param names."""
    out: Dict[str, Any] = {}
    path = cfg.get("path") or {}
    if path.get("cruise_speed_mps") is not None:
        out["cruise"] = float(path["cruise_speed_mps"])

    surge = cfg.get("surge_pi") or {}
    mapping_surge = {
        "kp": "surge_kp",
        "ki": "surge_ki",
        "k_ff": "surge_kff",
        "dx_max": "surge_dx_max",
        "u_lpf_tau_s": "surge_lpf",
        "x_max": "surge_xmax",
    }
    for src, dst in mapping_surge.items():
        if surge.get(src) is not None:
            out[dst] = float(surge[src])

    yaw = cfg.get("yaw_pd") or {}
    if yaw.get("kp") is not None:
        out["yaw_kp"] = float(yaw["kp"])
    if yaw.get("kd") is not None:
        out["yaw_kd"] = float(yaw["kd"])
    if yaw.get("n_max") is not None:
        out["yaw_nmax"] = float(yaw["n_max"])

    speed = cfg.get("speed") or {}
    if speed.get("u_d_floor_frac_of_cruise") is not None:
        out["u_d_floor"] = float(speed["u_d_floor_frac_of_cruise"])
    if speed.get("angle_threshold_fast_rad") is not None:
        out["angle_fast"] = float(speed["angle_threshold_fast_rad"])
    if speed.get("angle_threshold_slow_rad") is not None:
        out["angle_slow"] = float(speed["angle_threshold_slow_rad"])
    if speed.get("min_speed_scale") is not None:
        out["min_speed_scale"] = float(speed["min_speed_scale"])

    ilos = cfg.get("ilos") or {}
    if ilos.get("delta_m") is not None:
        out["ilos_delta"] = float(ilos["delta_m"])
    if ilos.get("sigma") is not None:
        out["ilos_sigma"] = float(ilos["sigma"])
    if ilos.get("e_i_limit") is not None:
        out["ilos_ei_lim"] = float(ilos["e_i_limit"])

    mpc = cfg.get("gs_mpc") or {}
    for src, dst in (
        ("horizon", "horizon"),
        ("w_path", "w_path"),
        ("w_u", "w_u"),
        ("w_r", "w_r"),
        ("w_du", "w_du"),
        ("w_dr", "w_dr"),
        ("u_max", "u_max"),
        ("r_max", "r_max"),
    ):
        if mpc.get(src) is not None:
            out[dst] = int(mpc[src]) if dst == "horizon" else float(mpc[src])
    return out


def h0_params_from_overlay(cfg: dict) -> Dict[str, Any]:
    """Map H0 nested overlay/config into flat tune_h0 param names."""
    out: Dict[str, Any] = {}
    path = cfg.get("path") or {}
    if path.get("cruise_speed_mps") is not None:
        out["cruise"] = float(path["cruise_speed_mps"])
    ilos = cfg.get("ilos") or {}
    if ilos.get("lookahead_min_m") is not None:
        out["ilos_min"] = float(ilos["lookahead_min_m"])
    if ilos.get("lookahead_max_m") is not None:
        out["ilos_max"] = float(ilos["lookahead_max_m"])
    if ilos.get("conv_rate") is not None:
        out["ilos_conv"] = float(ilos["conv_rate"])
    pid = cfg.get("pid") or {}
    if pid.get("kp") is not None:
        out["pid_kp"] = float(pid["kp"])
    if pid.get("kd") is not None:
        out["pid_kd"] = float(pid["kd"])
    guidance = cfg.get("guidance") or {}
    if guidance.get("max_yaw_rate_rad_s") is not None:
        out["max_yaw_r"] = float(guidance["max_yaw_rate_rad_s"])
    control = cfg.get("control") or {}
    if control.get("lateral_v_gain") is not None:
        out["lateral_v"] = float(control["lateral_v_gain"])
    mpc = cfg.get("mpc") or {}
    if mpc.get("horizon") is not None:
        out["horizon"] = int(mpc["horizon"])
    q = mpc.get("Q_vel_diag") or []
    if len(q) >= 3:
        out["Q_u"], out["Q_v"], out["Q_r"] = float(q[0]), float(q[1]), float(q[2])
    r = mpc.get("R_diag") or []
    if r:
        out["R_in"] = float(r[0])
    if mpc.get("Q_terminal_scale") is not None:
        out["Q_term"] = float(mpc["Q_terminal_scale"])
    return out


def ilos_params_from_overlay(cfg: dict) -> Dict[str, Any]:
    """Map ILOS nested overlay/config into flat tune_ilos param names."""
    out: Dict[str, Any] = {}
    path = cfg.get("path") or {}
    if path.get("cruise_speed_mps") is not None:
        out["cruise"] = float(path["cruise_speed_mps"])
    ilos = cfg.get("ilos") or {}
    if ilos.get("lookahead_min_m") is not None:
        out["ilos_min"] = float(ilos["lookahead_min_m"])
    if ilos.get("lookahead_max_m") is not None:
        out["ilos_max"] = float(ilos["lookahead_max_m"])
    if ilos.get("conv_rate") is not None:
        out["ilos_conv"] = float(ilos["conv_rate"])
    pid = cfg.get("pid") or {}
    if pid.get("kp") is not None:
        out["pid_kp"] = float(pid["kp"])
    if pid.get("kd") is not None:
        out["pid_kd"] = float(pid["kd"])
    guidance = cfg.get("guidance") or {}
    if guidance.get("max_yaw_rate_rad_s") is not None:
        out["max_yaw_r"] = float(guidance["max_yaw_rate_rad_s"])
    if guidance.get("kappa_speed_scale") is not None:
        out["kappa_scale"] = float(guidance["kappa_speed_scale"])
    thrust = cfg.get("thrust") or {}
    if thrust.get("yaw_mix_gain") is not None:
        out["yaw_mix"] = float(thrust["yaw_mix_gain"])
    if thrust.get("surge_gain") is not None:
        out["surge_gain"] = float(thrust["surge_gain"])
    if thrust.get("min_surge_norm") is not None:
        out["min_surge"] = float(thrust["min_surge_norm"])
    speed = cfg.get("speed") or {}
    if speed.get("angle_threshold_fast_rad") is not None:
        out["angle_fast"] = float(speed["angle_threshold_fast_rad"])
    if speed.get("angle_threshold_slow_rad") is not None:
        out["angle_slow"] = float(speed["angle_threshold_slow_rad"])
    if speed.get("min_speed_scale") is not None:
        out["min_speed_scale"] = float(speed["min_speed_scale"])
    return out


def load_seed_params(
    raw: Path | str,
    controller: str,
    *,
    required_keys: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
    """Load flat BO seed params from a prior tune path.

    ``controller`` is one of: ``gs``, ``h0``, ``ilos``.
    """
    artifact = resolve_seed_artifact(raw, controller)
    data = _load_data(artifact)
    params = flat_params_from_jsonish(data)

    if not params and isinstance(data, dict):
        # Treat as nested overlay / best_config.
        if controller == "gs":
            params = gs_params_from_overlay(data)
        elif controller == "h0":
            params = h0_params_from_overlay(data)
        elif controller == "ilos":
            params = ilos_params_from_overlay(data)

    if required_keys:
        missing = [k for k in required_keys if k not in params]
        if missing and not params:
            raise ValueError(
                f"seed artifact {artifact} produced no params for {controller}; "
                f"missing keys include {missing[:5]}"
            )

    if not params:
        raise ValueError(f"could not extract seed params from {artifact}")

    params["_seed_artifact"] = str(artifact)
    return params
