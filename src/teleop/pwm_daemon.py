#!/usr/bin/env python3
"""Always-on PWM owner: mux radio > ROS > stop, write motors.

Listens for UDP JSON commands (see device profile daemon_host/port).
Only this process may drive ESC PWM pins.

Device defaults come from teleop/devices/{rpi,jetson}.json via --device / DEVICE_CONFIG.
"""

from __future__ import annotations

import argparse
import logging
import os
import select
import signal
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from device_config import load_device_config
from ipc import IpcCommand
from motors import create_motors
from protocol import SOURCE_RADIO, SOURCE_ROS

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("teleop.pwm_daemon")


class PwmDaemon:
    def __init__(
        self,
        host: str,
        port: int,
        backend: str,
        left_pin: int,
        right_pin: int,
        radio_timeout_s: float,
        ros_timeout_s: float,
        tick_hz: float,
    ) -> None:
        self.radio_timeout_s = radio_timeout_s
        self.ros_timeout_s = ros_timeout_s
        self.tick_period = 1.0 / tick_hz

        self._radio: IpcCommand | None = None
        self._ros: IpcCommand | None = None
        self._radio_ts = 0.0
        self._ros_ts = 0.0
        self._active = "none"
        self._running = True

        self.motors = create_motors(backend, left_pin, right_pin)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        logger.info(
            "Listening on udp://%s:%d backend=%s pins L=%d R=%d",
            host,
            port,
            backend,
            left_pin,
            right_pin,
        )

    def stop(self, *_args) -> None:
        self._running = False

    def _ingest(self) -> None:
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            cmd = IpcCommand.from_json(data)
            if cmd is None:
                continue
            now = time.monotonic()
            if cmd.source == SOURCE_RADIO:
                self._radio = cmd
                self._radio_ts = now
            elif cmd.source == SOURCE_ROS:
                self._ros = cmd
                self._ros_ts = now

    def _select(self) -> tuple[str, float, float]:
        now = time.monotonic()
        radio_fresh = (
            self._radio is not None
            and self._radio.arm
            and (now - self._radio_ts) <= self.radio_timeout_s
        )
        if radio_fresh:
            assert self._radio is not None
            return SOURCE_RADIO, self._radio.left, self._radio.right

        ros_fresh = (
            self._ros is not None
            and self._ros.arm
            and (now - self._ros_ts) <= self.ros_timeout_s
        )
        if ros_fresh:
            assert self._ros is not None
            return SOURCE_ROS, self._ros.left, self._ros.right

        return "none", 0.0, 0.0

    def run(self) -> None:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        next_tick = time.monotonic()
        try:
            while self._running:
                self._ingest()
                now = time.monotonic()
                if now >= next_tick:
                    source, left, right = self._select()
                    if source != self._active:
                        logger.info("Active source → %s", source)
                        self._active = source
                    self.motors.set_thrust(left, right)
                    next_tick = now + self.tick_period
                timeout = max(0.0, next_tick - time.monotonic())
                select.select([self.sock], [], [], timeout)
        finally:
            logger.info("Shutting down — motors stop")
            self.motors.close()
            self.sock.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="PWM mux daemon (radio > ROS > stop)")
    parser.add_argument(
        "--device",
        default=os.environ.get("DEVICE_CONFIG", "rpi"),
        help="Device profile name (rpi|jetson) or path to JSON under teleop/devices/",
    )
    parser.add_argument(
        "--backend",
        default=None,
        choices=["dry_run", "pigpio", "jetson"],
        help="Override profile backend (e.g. dry_run for safe bring-up)",
    )
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop Hz")
    args = parser.parse_args()

    cfg = load_device_config(args.device)
    backend = args.backend or cfg.backend
    logger.info("Loaded device profile '%s' from devices/%s.json", cfg.name, cfg.name)

    PwmDaemon(
        host=cfg.daemon_host,
        port=cfg.daemon_port,
        backend=backend,
        left_pin=cfg.left_pin,
        right_pin=cfg.right_pin,
        radio_timeout_s=cfg.radio_timeout_s,
        ros_timeout_s=cfg.ros_timeout_s,
        tick_hz=args.rate,
    ).run()


if __name__ == "__main__":
    main()
