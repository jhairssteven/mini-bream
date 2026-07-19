#!/usr/bin/env python3
"""Print live joystick axis/button indices for mapping joy_to_serial args.

Usage (on the ground PC, host or container):
  python3 /opt/teleop/js_dump.py
  python3 /opt/teleop/js_dump.py --device /dev/input/js0

Move sticks and press buttons; note the index that changes.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from js_reader import LinuxJoystick


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=os.environ.get("JOY_DEVICE", "/dev/input/js0"))
    args = parser.parse_args()

    joy = LinuxJoystick(args.device)
    print(f"Device {args.device}: {len(joy.axes)} axes, {len(joy.buttons)} buttons")
    print("Move sticks / press buttons. Ctrl+C to quit.\n")
    print(
        "joy_to_serial defaults: --left-axis 1 --right-axis 4 --deadman-button 5"
    )
    print("-" * 72)

    last_axes = list(joy.axes)
    last_buttons = list(joy.buttons)
    try:
        while True:
            joy.poll()
            changed = False
            for i, (a, b) in enumerate(zip(joy.axes, last_axes)):
                if abs(a - b) > 0.05:
                    print(f"  axis[{i:2d}] = {a:+.3f}")
                    changed = True
            for i, (a, b) in enumerate(zip(joy.buttons, last_buttons)):
                if a != b:
                    print(f"  button[{i:2d}] = {a}")
                    changed = True
            if changed:
                last_axes = list(joy.axes)
                last_buttons = list(joy.buttons)
                # Compact snapshot
                axes_s = " ".join(f"{v:+.2f}" for v in joy.axes)
                btns_s = "".join(str(b) for b in joy.buttons)
                print(f"  ALL axes: [{axes_s}]")
                print(f"  ALL btns: [{btns_s}]")
                print("-" * 72)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\ndone")
    finally:
        joy.close()


if __name__ == "__main__":
    main()
