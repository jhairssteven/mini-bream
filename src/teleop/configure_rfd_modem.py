#!/usr/bin/env python3
"""Configure one RFDesign RFD900+ (SiK P2P) modem for mini-bream teleop.

Run once on each end of a new pair with the same air/netid/freq settings.
SiK Holybro radios are not programmed by this tool (they are usually ready
at 57600); use this for RFD900+ / RFD900P USB adapters (CP2102).

Examples:
  # Show current settings (tries common UART bauds)
  python3 configure_rfd_modem.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_... --show

  # Apply lab teleop defaults
  python3 configure_rfd_modem.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_...

  # Longer range: lower air rate + higher power (if legal); match both ends
  python3 configure_rfd_modem.py --port ... --air-speed 12 --tx-power 30

Defaults: UART 57600, air 64 kbps, NETID 25, TX 20 dBm, MAVLINK off, OPPRESEND off.
Stop radio_rx / telemetry_tx first so the serial port is free.
Both ends must match (including S7 OPPRESEND); a SiK 1.9 ↔ 1.13 pair is OK if params match.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("teleop.configure_rfd")

# Host UART: SiK S1 one-byte form → baud
SERIAL_SPEED_TO_BAUD = {
    1: 1200,
    2: 2400,
    4: 4800,
    9: 9600,
    19: 19200,
    38: 38400,
    57: 57600,
    115: 115200,
    230: 230400,
}
BAUD_TO_SERIAL_SPEED = {v: k for k, v in SERIAL_SPEED_TO_BAUD.items()}

PROBE_BAUDS = (57600, 115200, 38400, 9600)


@dataclass
class ModemSettings:
    serial_speed: int = 57  # 57600
    air_speed: int = 64
    netid: int = 25
    tx_power: int = 20
    mavlink: int = 0
    ecc: int = 0
    oppresend: int = 0  # must match pair (older SiK 1.9 defaults to 1)
    min_freq: int = 915000
    max_freq: int = 928000
    num_channels: int = 50

    def as_at_commands(self) -> list[str]:
        return [
            f"ATS1={self.serial_speed}",
            f"ATS2={self.air_speed}",
            f"ATS3={self.netid}",
            f"ATS4={self.tx_power}",
            f"ATS5={self.ecc}",
            f"ATS6={self.mavlink}",
            f"ATS7={self.oppresend}",
            f"ATS8={self.min_freq}",
            f"ATS9={self.max_freq}",
            f"ATS10={self.num_channels}",
        ]


def _require_pyserial() -> None:
    try:
        import serial  # noqa: F401
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install pyserial") from exc


def enter_at_mode(ser, guard_s: float = 1.1) -> bool:
    """Enter AT command mode with SiK guard times. Returns True on OK."""
    ser.reset_input_buffer()
    time.sleep(guard_s)
    ser.write(b"+++")
    ser.flush()
    time.sleep(guard_s)
    resp = ser.read(64)
    ok = b"OK" in resp
    logger.debug("+++ @ %s → %r", ser.baudrate, resp)
    return ok


def at_cmd(ser, cmd: str, settle_s: float = 0.35) -> str:
    if not cmd.endswith("\r"):
        cmd = cmd + "\r"
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    time.sleep(settle_s)
    return ser.read(2048).decode("latin1", errors="replace")


def exit_at_mode(ser) -> None:
    at_cmd(ser, "ATO", settle_s=0.2)


def open_at_session(port: str, preferred_baud: int | None = None):
    """Open serial and enter AT mode; try preferred then common bauds."""
    import serial

    bauds: list[int] = []
    if preferred_baud:
        bauds.append(preferred_baud)
    for b in PROBE_BAUDS:
        if b not in bauds:
            bauds.append(b)

    last_err: Exception | None = None
    for baud in bauds:
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
        except Exception as exc:  # noqa: BLE001 — surface open errors per baud
            last_err = exc
            logger.debug("open %s @ %d failed: %s", port, baud, exc)
            continue
        if enter_at_mode(ser):
            logger.info("AT mode on %s @ %d", port, baud)
            return ser, baud
        ser.close()

    hint = f" (last open error: {last_err})" if last_err else ""
    raise SystemExit(
        f"Could not enter AT mode on {port} (tried bauds {bauds}){hint}. "
        "Is another process using the port (radio_rx / telemetry_tx)?"
    )


def read_identity(ser) -> str:
    return at_cmd(ser, "ATI").strip()


def read_params(ser) -> str:
    return at_cmd(ser, "ATI5", settle_s=0.5)


def apply_settings(ser, settings: ModemSettings) -> None:
    for cmd in settings.as_at_commands():
        resp = at_cmd(ser, cmd)
        if "OK" not in resp:
            raise SystemExit(f"Command failed: {cmd!r} → {resp!r}")
        logger.info("%s → OK", cmd)

    logger.info("Parameters after write:\n%s", read_params(ser))

    if "OK" not in at_cmd(ser, "AT&W"):
        raise SystemExit("AT&W failed")
    logger.info("AT&W → OK (saved)")
    # ATZ reboots; response may be truncated
    ser.write(b"ATZ\r")
    ser.flush()
    time.sleep(0.3)
    ser.close()
    logger.info("ATZ sent — modem rebooting")
    time.sleep(2.0)


def configure(port: str, settings: ModemSettings, *, show_only: bool) -> int:
    _require_pyserial()
    import serial

    host_baud = SERIAL_SPEED_TO_BAUD.get(settings.serial_speed)
    ser, _opened_at = open_at_session(port, preferred_baud=host_baud)
    try:
        ident = read_identity(ser)
        logger.info("Modem: %s", ident.replace("\r", " ").strip())
        params = read_params(ser)
        logger.info("Current parameters:\n%s", params)
        if show_only:
            exit_at_mode(ser)
            return 0
        apply_settings(ser, settings)
    finally:
        if ser.is_open:
            try:
                exit_at_mode(ser)
            except Exception:  # noqa: BLE001
                pass
            ser.close()

    if show_only:
        return 0

    # Confirm at target UART baud after reboot
    target_baud = SERIAL_SPEED_TO_BAUD[settings.serial_speed]
    time.sleep(0.5)
    ser = serial.Serial(port, target_baud, timeout=0.5)
    try:
        if not enter_at_mode(ser):
            raise SystemExit(f"Re-open AT mode failed at {target_baud} after reboot")
        logger.info("Confirmed AT mode @ %d after reboot", target_baud)
        logger.info("Final parameters:\n%s", read_params(ser))
        exit_at_mode(ser)
    finally:
        ser.close()
    logger.info("Configured %s — use RADIO_BAUD=%d", port, target_baud)
    return 0


def build_settings(args: argparse.Namespace) -> ModemSettings:
    if args.baud is not None:
        if args.baud not in BAUD_TO_SERIAL_SPEED:
            raise SystemExit(
                f"Unsupported --baud {args.baud}; choose one of "
                f"{sorted(BAUD_TO_SERIAL_SPEED)}"
            )
        serial_speed = BAUD_TO_SERIAL_SPEED[args.baud]
    else:
        serial_speed = args.serial_speed

    return ModemSettings(
        serial_speed=serial_speed,
        air_speed=args.air_speed,
        netid=args.netid,
        tx_power=args.tx_power,
        mavlink=args.mavlink,
        ecc=args.ecc,
        oppresend=args.oppresend,
        min_freq=args.min_freq,
        max_freq=args.max_freq,
        num_channels=args.num_channels,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Configure one RFD900+ modem for mini-bream emergency teleop",
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("RADIO_SERIAL_PORT"),
        help="Serial by-id path (or RADIO_SERIAL_PORT)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Print current ATI5 only; do not change settings",
    )
    parser.add_argument("--serial-speed", type=int, default=57, help="S1 one-byte UART (57=57600)")
    parser.add_argument(
        "--baud",
        type=int,
        default=None,
        help="Host UART baud (alternative to --serial-speed); e.g. 57600",
    )
    parser.add_argument("--air-speed", type=int, default=64, help="S2 air rate kbps (must match pair)")
    parser.add_argument("--netid", type=int, default=25, help="S3 NETID (must match pair)")
    parser.add_argument("--tx-power", type=int, default=20, help="S4 TX power dBm (0–30)")
    parser.add_argument("--mavlink", type=int, default=0, choices=(0, 1, 2), help="S6; use 0 for teleop")
    parser.add_argument("--ecc", type=int, default=0, choices=(0, 1), help="S5 ECC")
    parser.add_argument(
        "--oppresend",
        type=int,
        default=0,
        choices=(0, 1),
        help="S7 opportunistic resend; must match pair (lab default 0)",
    )
    parser.add_argument("--min-freq", type=int, default=915000)
    parser.add_argument("--max-freq", type=int, default=928000)
    parser.add_argument("--num-channels", type=int, default=50)
    args = parser.parse_args(argv)

    if not args.port:
        raise SystemExit("RADIO_SERIAL_PORT (or --port) must be set")
    if not (0 <= args.tx_power <= 30):
        raise SystemExit("--tx-power must be in 0..30")

    settings = build_settings(args)
    target_baud = SERIAL_SPEED_TO_BAUD[settings.serial_speed]
    logger.info(
        "Target: UART=%d (S1=%d) AIR=%d NETID=%d TX=%d MAVLINK=%d",
        target_baud,
        settings.serial_speed,
        settings.air_speed,
        settings.netid,
        settings.tx_power,
        settings.mavlink,
    )
    return configure(args.port, settings, show_only=args.show)


if __name__ == "__main__":
    sys.exit(main())
