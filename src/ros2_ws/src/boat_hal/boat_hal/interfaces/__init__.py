"""Abstract sensor and actuator interfaces (SOLID: interface segregation)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Protocol


class GpsProvider(ABC):
    @abstractmethod
    def topic(self) -> str:
        """Return the canonical GPS topic name."""


class ImuProvider(ABC):
    @abstractmethod
    def topic(self) -> str:
        """Return the canonical IMU topic name."""


class LidarProvider(ABC):
    @abstractmethod
    def points_topic(self) -> str:
        """Return the canonical lidar point cloud topic name."""


class CameraProvider(ABC):
    @abstractmethod
    def image_topic(self) -> str:
        """Return the canonical RGB image topic name."""


class ThrusterProvider(ABC):
    @abstractmethod
    def left_topic(self) -> str:
        """Return the canonical left thrust command topic."""

    @abstractmethod
    def right_topic(self) -> str:
        """Return the canonical right thrust command topic."""


class HalProvider(Protocol):
    """Marker protocol for HAL backend implementations."""

    mode: str
