#!/usr/bin/env python3
"""Smoke tests for teleop protocol, device configs, and PWM daemon mux."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from device_config import load_device_config
from ipc import PwmDaemonClient
from protocol import SerialFrameParser, TeleopCommand


def test_protocol() -> None:
    cmd = TeleopCommand(left=0.5, right=-0.25, arm=True, seq=7)
    pkt = cmd.encode_serial()
    assert len(pkt) == 9
    decoded = TeleopCommand.decode_serial(pkt)
    assert decoded is not None
    assert abs(decoded.left - 0.5) < 1e-3
    assert abs(decoded.right + 0.25) < 1e-3
    assert decoded.arm and decoded.seq == 7

    parser = SerialFrameParser()
    cmds = parser.feed(b"\x00\x01" + pkt[:4] + pkt)
    assert len(cmds) == 1
    print("OK protocol")


def test_device_configs() -> None:
    rpi = load_device_config("rpi")
    assert rpi.backend == "pigpio" and rpi.left_pin == 19 and rpi.right_pin == 12
    assert abs(rpi.max_thrust - 0.7) < 1e-9
    jetson = load_device_config("jetson")
    assert jetson.backend == "jetson" and jetson.left_pin == 33 and jetson.right_pin == 32
    assert abs(jetson.max_thrust - 0.7) < 1e-9
    print("OK device configs", rpi.name, jetson.name)


def test_daemon_mux_docker() -> None:
    """Run daemon with rpi profile + dry_run override; verify radio > ros > none."""
    proc = subprocess.Popen(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "host",
            "-v",
            f"{ROOT}:/opt/teleop",
            "mini-bream:teleop",
            "python3",
            "/opt/teleop/pwm_daemon.py",
            "--device",
            "rpi",
            "--backend",
            "dry_run",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    time.sleep(1.5)
    if proc.poll() is not None:
        out, _ = proc.communicate(timeout=2)
        raise AssertionError(f"daemon exited early:\n{out}")

    client = PwmDaemonClient()
    try:
        # Keep radio fresh across the ROS send
        for i in range(8):
            client.send("radio", 0.3, -0.1, arm=True, seq=i)
            time.sleep(0.05)
        client.send("ros", 0.9, 0.9, arm=True, seq=99)
        time.sleep(0.15)
        for i in range(4):
            client.send("radio", 0.3, -0.1, arm=True, seq=10 + i)
            time.sleep(0.05)
        # Release radio → ROS should take over
        client.send("radio", 0.0, 0.0, arm=False, seq=20)
        time.sleep(0.15)
        for i in range(6):
            client.send("ros", 0.4, 0.4, arm=True, seq=100 + i)
            time.sleep(0.05)
        # Stop ROS feed → none
        time.sleep(0.7)
    finally:
        client.close()
        proc.send_signal(signal.SIGTERM)
        out, _ = proc.communicate(timeout=5)

    assert "Loaded device profile 'rpi'" in out or "backend=dry_run" in out
    assert "Active source → radio" in out
    assert "Active source → ros" in out
    assert "Active source → none" in out
    # radio 0.3 * max_thrust 0.7 → 0.210 (dry-run logs post-cap thrust)
    assert "L=0.210" in out or "L=0.21" in out
    print("OK daemon mux\n" + "\n".join(out.strip().splitlines()[-12:]))


def test_radio_port_present() -> None:
    by_id = Path("/dev/serial/by-id")
    assert by_id.is_dir(), "missing /dev/serial/by-id"
    ftdi = list(by_id.glob("*FTDI*")) + list(by_id.glob("*FT231*"))
    assert ftdi, f"No FTDI/SiK device under {by_id}: {list(by_id.iterdir())}"
    target = ftdi[0].resolve()
    assert target.exists()
    print(f"OK SiK candidate {ftdi[0]} -> {target}")


def test_radio_serial_open() -> None:
    import serial

    port = os.environ.get(
        "RADIO_SERIAL_PORT",
        "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30GKEF6-if00-port0",
    )
    if not Path(port).exists():
        # Fall back to first FTDI by-id
        matches = list(Path("/dev/serial/by-id").glob("*FTDI*"))
        assert matches, "no FTDI serial device"
        port = str(matches[0])

    with serial.Serial(port, 57600, timeout=0.2) as ser:
        # Write a disarm frame; expect no exception (SiK is a transparent UART)
        ser.write(TeleopCommand(0.0, 0.0, arm=False, seq=1).encode_serial())
        _ = ser.read(64)
    print(f"OK opened radio serial {port}")


def main() -> None:
    test_protocol()
    test_device_configs()
    test_radio_port_present()
    test_radio_serial_open()
    test_daemon_mux_docker()
    print("\nAll teleop smoke tests passed.")


if __name__ == "__main__":
    main()
