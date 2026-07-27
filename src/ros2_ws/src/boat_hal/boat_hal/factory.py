"""Factory for HAL configuration (dependency inversion)."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from boat_hal.config import HalConfig


def default_config_path(mode: str = "simulation") -> str:
    share = get_package_share_directory("boat_hal")
    filename = "sim.yaml" if mode == "simulation" else "real.yaml"
    return str(Path(share) / "config" / filename)


def load_hal_config(mode: str | None = None, config_path: str | None = None) -> HalConfig:
    if config_path:
        return HalConfig.from_yaml(config_path)
    resolved_mode = mode or "simulation"
    return HalConfig.from_yaml(default_config_path(resolved_mode))
