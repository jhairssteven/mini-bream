#!/bin/bash
# Start pigpiod (hardware PWM), then run the service command.
set -e

pigpio_ready() {
  python3 -c "import pigpio; p=pigpio.pi(); ok=p.connected; p.stop(); raise SystemExit(0 if ok else 1)" 2>/dev/null
}

if command -v pigpiod >/dev/null 2>&1; then
  if ! pigpio_ready; then
    echo "[teleop] starting pigpiod..."
    pigpiod
    for _ in $(seq 1 30); do
      if pigpio_ready; then
        echo "[teleop] pigpiod is ready"
        break
      fi
      sleep 0.1
    done
  else
    echo "[teleop] pigpiod already reachable"
  fi
fi

exec "$@"
