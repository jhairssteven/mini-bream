"""Thrust → ESC PWM helpers and backends (dry-run / pigpio / jetson)."""

from __future__ import annotations

import logging
from typing import Protocol

logger = logging.getLogger("teleop.motors")

# BlueRobotics T200 / Basic ESC style mapping (matches frontseat BlueRoboticsT200)
PWM_FREQ_HZ = 340
SENSITIVITY = 0.7


def thrust_to_pulse_us(thrust: float) -> float:
    """Map thrust in [-1, 1] to pulse width µs with deadband around neutral."""
    thrust = max(-1.0, min(1.0, float(thrust)))
    if thrust < 0.0:
        # [-1, 0) → [1000, 1440]
        return 1000.0 + (thrust + 1.0) * (1440.0 - 1000.0)
    if thrust > 0.0:
        # (0, 1] → [1560, 2000]
        return 1560.0 + thrust * (2000.0 - 1560.0)
    return 1500.0


def pulse_us_to_duty_percent(pulse_us: float, freq_hz: float = PWM_FREQ_HZ) -> float:
    period_us = 1e6 / freq_hz
    return (pulse_us / period_us) * 100.0


def thrust_to_duty_percent(thrust: float, sensibility: float = SENSITIVITY) -> float:
    return pulse_us_to_duty_percent(thrust_to_pulse_us(thrust * sensibility))


class MotorPair(Protocol):
    def set_thrust(self, left: float, right: float) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class DryRunMotors:
    def __init__(self) -> None:
        self._last = (0.0, 0.0)

    def set_thrust(self, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        if (left, right) != self._last:
            logger.info(
                "dry-run thrust L=%.3f R=%.3f (duty L=%.2f%% R=%.2f%%)",
                left,
                right,
                thrust_to_duty_percent(left),
                thrust_to_duty_percent(right),
            )
            self._last = (left, right)

    def stop(self) -> None:
        self.set_thrust(0.0, 0.0)

    def close(self) -> None:
        self.stop()


class PigpioMotors:
    """Hardware PWM via pigpio (Raspberry Pi). Requires pigpiod."""

    def __init__(self, left_pin: int, right_pin: int, freq_hz: int = PWM_FREQ_HZ) -> None:
        import pigpio

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio not connected — is pigpiod running?")
        self.left_pin = left_pin
        self.right_pin = right_pin
        self.freq_hz = freq_hz
        self.stop()

    def _write(self, pin: int, thrust: float) -> None:
        duty_pct = thrust_to_duty_percent(thrust)
        duty_ppm = int(max(0, min(1_000_000, duty_pct * 10_000)))
        self.pi.hardware_PWM(pin, self.freq_hz, duty_ppm)

    def set_thrust(self, left: float, right: float) -> None:
        self._write(self.left_pin, max(-1.0, min(1.0, left)))
        self._write(self.right_pin, max(-1.0, min(1.0, right)))

    def stop(self) -> None:
        self.set_thrust(0.0, 0.0)

    def close(self) -> None:
        self.stop()
        self.pi.hardware_PWM(self.left_pin, 0, 0)
        self.pi.hardware_PWM(self.right_pin, 0, 0)
        self.pi.stop()


class JetsonMotors:
    """Jetson.GPIO software PWM (legacy Mini-Bream Orin pins)."""

    def __init__(self, left_pin: int, right_pin: int, freq_hz: int = PWM_FREQ_HZ) -> None:
        import Jetson.GPIO as GPIO

        self.GPIO = GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(left_pin, GPIO.OUT)
        GPIO.setup(right_pin, GPIO.OUT)
        zero = thrust_to_duty_percent(0.0)
        self.left = GPIO.PWM(left_pin, freq_hz)
        self.right = GPIO.PWM(right_pin, freq_hz)
        self.left.start(zero)
        self.right.start(zero)

    def set_thrust(self, left: float, right: float) -> None:
        self.left.ChangeDutyCycle(thrust_to_duty_percent(max(-1.0, min(1.0, left))))
        self.right.ChangeDutyCycle(thrust_to_duty_percent(max(-1.0, min(1.0, right))))

    def stop(self) -> None:
        self.set_thrust(0.0, 0.0)

    def close(self) -> None:
        self.stop()
        self.left.stop()
        self.right.stop()
        self.GPIO.cleanup()


def create_motors(backend: str, left_pin: int, right_pin: int) -> MotorPair:
    backend = backend.lower()
    if backend in ("dry_run", "dry-run", "none"):
        logger.warning("PWM backend=dry_run — motors will NOT move")
        return DryRunMotors()
    if backend == "pigpio":
        return PigpioMotors(left_pin, right_pin)
    if backend == "jetson":
        return JetsonMotors(left_pin, right_pin)
    raise ValueError(f"Unknown PWM backend: {backend}")
