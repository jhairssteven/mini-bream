"""Exponential low-pass filter for heading angles."""

from __future__ import annotations

from frontseat.heading.angles import normalize_angle


class LowPassHeadingFilter:
    """First-order low-pass on yaw with angle wrapping."""

    def __init__(self, alpha: float) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha must be in (0, 1], got {alpha}')
        self._alpha = alpha
        self._yaw_rad: float | None = None

    @property
    def initialized(self) -> bool:
        return self._yaw_rad is not None

    def reset(self) -> None:
        self._yaw_rad = None

    def update(self, yaw_rad: float) -> float:
        if self._yaw_rad is None:
            self._yaw_rad = yaw_rad
            return self._yaw_rad

        delta = normalize_angle(yaw_rad - self._yaw_rad)
        self._yaw_rad = normalize_angle(self._yaw_rad + self._alpha * delta)
        return self._yaw_rad
