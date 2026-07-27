"""HAL configuration loader."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml

from boat_hal.contracts.topics import TopicContract


@dataclass(frozen=True)
class HalConfig:
    mode: str
    use_sim_time: bool
    topics: TopicContract
    origin_lat: float
    origin_lon: float
    model_name: str
    max_thrust_N: float
    sim_thrust_scale: float

    @classmethod
    def from_yaml(cls, path: str | Path) -> "HalConfig":
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "HalConfig":
        topics = TopicContract.from_dict(data.get("topics", {}))
        origin = data.get("origin", {})
        boat = data.get("boat", {})
        lat = origin.get("lat")
        lon = origin.get("lon")
        return cls(
            mode=str(data.get("mode", "real")),
            use_sim_time=bool(data.get("use_sim_time", data.get("mode") == "simulation")),
            topics=topics,
            origin_lat=float(lat if lat is not None else 40.448417),
            origin_lon=float(lon if lon is not None else -86.86775),
            model_name=str(data.get("model_name", "blueboat")),
            max_thrust_N=float(boat.get("max_thrust_N", 250.0)),
            sim_thrust_scale=float(boat.get("sim_thrust_scale", 1000.0)),
        )

    @property
    def is_simulation(self) -> bool:
        return self.mode == "simulation"
