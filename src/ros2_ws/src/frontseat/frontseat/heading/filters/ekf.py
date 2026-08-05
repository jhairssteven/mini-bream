"""1D extended Kalman filter for heading fusion (GPS yaw + IMU gyro)."""

from __future__ import annotations

from dataclasses import dataclass

from frontseat.heading.angles import normalize_angle


@dataclass
class HeadingEkfState:
    yaw_rad: float
    variance_rad2: float


class HeadingEkf:
    """
    Fuse low-rate GPS heading with high-rate gyro z.

    State: yaw about +Z (ENU). Prediction uses gyro; update uses GPS yaw.
    """

    def __init__(
        self,
        process_noise_var: float,
        initial_variance_rad2: float = 1.0,
        max_variance_rad2: float = 0.05,
        gps_variance_scale: float = 1.0,
    ) -> None:
        if process_noise_var <= 0.0:
            raise ValueError(f'process_noise_var must be > 0, got {process_noise_var}')
        self._process_noise_var = process_noise_var
        self._max_variance_rad2 = max_variance_rad2
        self._gps_variance_scale = gps_variance_scale
        self._state: HeadingEkfState | None = None
        self._initial_variance_rad2 = initial_variance_rad2

    @property
    def initialized(self) -> bool:
        return self._state is not None

    def reset(self) -> None:
        self._state = None

    def predict(self, gyro_z_rad_s: float, dt_s: float) -> HeadingEkfState | None:
        if self._state is None or dt_s <= 0.0:
            return self._state

        yaw = normalize_angle(self._state.yaw_rad + gyro_z_rad_s * dt_s)
        variance = min(
            self._state.variance_rad2 + self._process_noise_var * dt_s,
            self._max_variance_rad2,
        )
        self._state = HeadingEkfState(yaw_rad=yaw, variance_rad2=variance)
        return self._state

    def update(self, measurement_yaw_rad: float, measurement_variance_rad2: float) -> HeadingEkfState:
        if measurement_variance_rad2 <= 0.0:
            raise ValueError(
                f'measurement_variance_rad2 must be > 0, got {measurement_variance_rad2}'
            )

        if self._state is None:
            self._state = HeadingEkfState(
                yaw_rad=measurement_yaw_rad,
                variance_rad2=max(measurement_variance_rad2, self._initial_variance_rad2),
            )
            return self._state

        meas_var = max(measurement_variance_rad2 * self._gps_variance_scale, 1e-8)
        innovation = normalize_angle(measurement_yaw_rad - self._state.yaw_rad)
        innovation_variance = self._state.variance_rad2 + meas_var
        kalman_gain = self._state.variance_rad2 / innovation_variance
        yaw = normalize_angle(self._state.yaw_rad + kalman_gain * innovation)
        variance = (1.0 - kalman_gain) * self._state.variance_rad2
        # Keep posterior uncertainty bounded so the next GPS update stays trusted.
        variance = min(variance, meas_var)
        self._state = HeadingEkfState(yaw_rad=yaw, variance_rad2=variance)
        return self._state

    def state(self) -> HeadingEkfState | None:
        return self._state
