#!/usr/bin/env python3
"""Robot-side serial receiver: telemetry radio → PWM daemon (non-ROS)."""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ipc import PwmDaemonClient
from protocol import SOURCE_RADIO, SerialFrameParser

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("teleop.radio_rx")


def main() -> None:
    parser = argparse.ArgumentParser(description="Forward teleop serial frames to PWM daemon")
    parser.add_argument("--port", default=os.environ.get("RADIO_SERIAL_PORT", "/dev/ttyUSB0"))
    parser.add_argument("--baud", type=int, default=int(os.environ.get("RADIO_BAUD", "57600")))
    parser.add_argument("--daemon-host", default=os.environ.get("PWM_DAEMON_HOST", "127.0.0.1"))
    parser.add_argument("--daemon-port", type=int, default=int(os.environ.get("PWM_DAEMON_PORT", "5600")))
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install pyserial") from exc

    client = PwmDaemonClient(host=args.daemon_host, port=args.daemon_port)
    parser_rx = SerialFrameParser()

    while True:
        try:
            logger.info("Opening radio serial %s @ %d", args.port, args.baud)
            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                while True:
                    data = ser.read(64)
                    if not data:
                        continue
                    for cmd in parser_rx.feed(data):
                        client.send(
                            source=SOURCE_RADIO,
                            left=cmd.left,
                            right=cmd.right,
                            arm=cmd.arm,
                            seq=cmd.seq,
                        )
        except serial.SerialException as exc:
            logger.error("Serial error: %s — retrying in 2s", exc)
            time.sleep(2.0)
        except KeyboardInterrupt:
            break

    client.close()
    logger.info("radio_rx stopped")


if __name__ == "__main__":
    main()
