# Emergency teleop (non-ROS motor path)

## Goal

Keep a joystick override that still works if the robot ROS 2 stack dies.
ROS autonomy can run normally; radio teleop **overrides** it when armed.

## Roles

| Service | Where | Compose file | Role |
|---------|-------|--------------|------|
| `ground_station` | Ground PC | `docker-compose.ground.yml` | `joy_node` + `joy_to_serial` → telemetry radio |
| `pwm_daemon` | Robot | `docker-compose.prod.yml` | **Only** process that writes motor PWM; mux + failsafe |
| `radio_rx` | Robot | `docker-compose.prod.yml` | Serial from radio → UDP to daemon (no ROS) |
| `frontseat` | Robot | `docker-compose.prod.yml` | ROS stack; `motor_controller` sends thrust to daemon over UDP |

### Ground station ROS isolation (by design)

The ground station uses ROS 2 only as a convenience to read the joystick (`joy_node` → `/joy` → `joy_to_serial`). That ROS graph runs in its **own Docker network** (default bridge — **not** `network_mode: host`), so `/joy` and any other GS topics are **unreachable** from the robot autonomy stack. The only link to the boat is the **serial telemetry radio**; the robot never depends on the ground station’s ROS DDS domain.

## Data flow

```
Ground station                         Robot
──────────────                         ─────
joystick
   │
joy_node (/joy)
   │
joy_to_serial ── serial frame ──radio──► radio_rx ──UDP──┐
                                                         │
ROS autonomy / ThrustSourceSelector                      ▼
   │                                              pwm_daemon
motor_controller ──────────────UDP───────────────────────┤
                                                         │
                                                    PWM → ESCs
```

### Mux priority (inside `pwm_daemon`)

1. **Radio** if `arm=true` and packet age ≤ profile `radio_timeout_s` (default 0.25 s)
2. Else **ROS** if `arm=true` and packet age ≤ profile `ros_timeout_s` (default 0.5 s)
3. Else **stop** (thrust 0)

## Device profiles (PWM backend + pins)

Defaults live in JSON, not scattered env vars:

| File | Platform | Backend | Pins |
|------|----------|---------|------|
| `devices/rpi.json` | Raspberry Pi | `pigpio` | BCM 13 / 12 |
| `devices/jetson.json` | Jetson Orin | `jetson` | BOARD 33 / 32 |

Select with `DEVICE_CONFIG`:

```bash
DEVICE_CONFIG=rpi docker compose -f docker-compose.prod.yml up -d pwm_daemon
DEVICE_CONFIG=jetson docker compose -f docker-compose.prod.yml up -d pwm_daemon
```

Safe bring-up without moving motors (override backend only):

```bash
DEVICE_CONFIG=rpi PWM_BACKEND_OVERRIDE=dry_run \
  docker compose -f docker-compose.prod.yml up -d pwm_daemon
```

## Finding the SiK telemetry radio serial port

SiK Telemetry Radio v3 usually appears as an **FTDI FT231X** USB-UART (not as `ttyACM*` — those are often GPS).

**1. List stable IDs (preferred):**

```bash
ls -l /dev/serial/by-id/
```

Example on this robot:

```text
usb-FTDI_FT231X_USB_UART_D30GKEF6-if00-port0 -> ../../ttyUSB0   ← SiK radio
usb-u-blox_AG_-_..._u-blox_GNSS_receiver-if00 -> ../../ttyACM1 ← GPS
```

Use the **by-id** path in compose so it survives reboot / USB reorder:

```bash
export RADIO_SERIAL_PORT=/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30GKEF6-if00-port0
```

**2. Confirm vendor/product:**

```bash
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL'
```

SiK v3 typically shows `ID_VENDOR=FTDI` and `ID_MODEL=FT231X_USB_UART`.

**3. Unplug/replug test:**

```bash
dmesg -w
# unplug radio, plug back — look for FTDI / ttyUSB
```

**4. Baud:** SiK default is often **57600** (set `RADIO_BAUD` if you changed air/radio settings).

GPS modules (`u-blox` → `ttyACM*`) are **not** the telemetry radio — leave those for the frontseat GPS launch files.

## Serial protocol

9-byte little-endian frame (see `protocol.py`):

`AA 55 | left_i16 | right_i16 | flags | seq | crc8`

- Thrust: `i16 = round(thrust * 1000)`, range `[-1, 1]`
- `flags` bit0 = **arm** (deadman held on the ground joystick, default button 0)

## IPC to the daemon

UDP JSON to `127.0.0.1:5600` (host network on the robot):

```json
{"source":"ros"|"radio","left":0.2,"right":-0.1,"arm":true,"seq":1}
```

## How to run

**Robot**

```bash
cd src/docker
DEVICE_CONFIG=rpi PWM_BACKEND_OVERRIDE=dry_run \
  docker compose -f docker-compose.prod.yml up -d --build pwm_daemon radio_rx
docker compose -f docker-compose.prod.yml up --build frontseat
# then inside frontseat: ros2 launch …
```

**Ground station** (same repo)

```bash
cd src/docker
RADIO_SERIAL_PORT=/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_<SERIAL>-if00-port0 \
JOY_DEVICE=/dev/input/js0 \
  docker compose -f docker-compose.ground.yml up --build
```

Hold **button 0** (deadman) to arm; release or lose the radio link → motors stop / fall back to ROS.

## Failsafe notes

- Ground station always sends frames; with deadman released, `arm=0` and thrust 0.
- If radio goes silent longer than the timeout, radio priority drops immediately.
- If ROS also dies, daemon holds stop.
- `pwm_daemon` should stay up (`restart: unless-stopped`); it is the safety-critical service.
