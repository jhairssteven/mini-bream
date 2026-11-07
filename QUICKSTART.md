# Quick start

On each machine, from the repo:

```bash
cd src/docker
chmod +x mini_bream_env.sh
```

Radio teleop needs the modem path on **Pi** and **GS** (no default):

```bash
export RADIO_SERIAL_PORT=/dev/serial/by-id/<your-radio>   # SiK FTDI or RFD900+ CP2102
export RADIO_BAUD=57600
```

## 1. Raspberry Pi (motors + radio + frontseat)

```bash
./mini_bream_env.sh start pi --dry-run --build   # safe (no ESC drive)
# later, real motors:
# ./mini_bream_env.sh start pi --build
```

## 2. Jetson (LiDAR + ZED)

```bash
./mini_bream_env.sh start jetson --build
```

## 3. Ground station (Xbox teleop + RViz)

```bash
export JOY_DEVICE=/dev/input/js0
xhost +local:docker
./mini_bream_env.sh start gs --build
```

## Stop

```bash
./mini_bream_env.sh stop pi
./mini_bream_env.sh stop jetson
./mini_bream_env.sh stop gs
```

## Check radio teleop

```bash
# GS
docker logs -f mini_bream_telemetry_tx   # hold deadman (button 5)

# Pi
docker logs -f mini_bream_radio_rx
docker logs -f mini_bream_pwm_daemon     # expect: Active source → radio
```

More detail: `src/docker/README.md`, `src/teleop/README.md`.
