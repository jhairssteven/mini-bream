# Emergency teleop (non-ROS motor path)

## Goal

Keep a joystick override that still works if the robot ROS 2 stack dies.
ROS autonomy can run normally; radio teleop **overrides** it when armed.

## Roles

| Service | Where | Compose file | Role |
|---------|-------|--------------|------|
| `ground_station` | Ground PC | `docker-compose.ground.yml` | Linux `js*` reader + `joy_to_serial` → telemetry radio |
| `pwm_daemon` | Robot | `docker-compose.prod.yml` | **Only** process that writes motor PWM; mux + failsafe |
| `radio_rx` | Robot | `docker-compose.prod.yml` | Serial from radio → UDP to daemon (no ROS) |
| `frontseat` | Robot | `docker-compose.prod.yml` | ROS stack; `motor_controller` sends thrust to daemon over UDP |

### Ground station isolation (by design)

The ground station does **not** use ROS to talk to the robot. It reads the controller via the Linux joystick device (`/dev/input/js0`) and sends framed packets over the SiK radio. The container uses Docker’s **default bridge network** (not `network_mode: host`), so even if ROS were present there, its topics would be unreachable from the robot autonomy stack. The only link to the boat is the **serial telemetry radio**.

(We intentionally avoid ROS 2 `joy_node` on the GS: it uses SDL and often fails to publish for Xbox 360 wireless receivers that still work fine as `/dev/input/js0`.)

## Data flow

```mermaid
flowchart LR
    subgraph GS["Ground station"]
        Xbox["Xbox / joystick"]
        JS["/dev/input/js0<br/>Linux joystick API"]
        TX["joy_to_serial"]
        GRadio["SiK radio"]

        Xbox --> JS --> TX
        TX -->|"Serial frames"| GRadio
    end

    GRadio <-->|"Telemetry radio link"| RRadio

    subgraph Robot
        RRadio["SiK radio"]
        RX["radio_rx"]
        ROS["ROS autonomy<br/>ThrustSourceSelector"]
        MC["motor_controller"]
        PWM["pwm_daemon"]
        ESC["ESCs / motors"]

        RRadio --> RX
        RX -->|"UDP radio command"| PWM
        ROS --> MC
        MC -->|"UDP ROS command"| PWM
        PWM -->|"Hardware PWM"| ESC
    end
```

### Mux priority (inside `pwm_daemon`)

1. **Radio** if `arm=true` and packet age ≤ profile `radio_timeout_s` (default 0.25 s)
2. Else **ROS** if `arm=true` and packet age ≤ profile `ros_timeout_s` (default 0.5 s)
3. Else **stop** (thrust 0)

## Device profiles (PWM backend + pins)

Defaults live in JSON, not scattered env vars:

| File | Platform | Backend | Pins |
|------|----------|---------|------|
| `devices/rpi.json` | Raspberry Pi | `pigpio` (hardware PWM) | BCM 13 / 12 |
| `devices/jetson.json` | Jetson Orin | `jetson` | BOARD 33 / 32 |

`pigpio` uses **hardware PWM** via `pigpiod` (built into the teleop image and started by `entrypoint.teleop.sh`). Pins must be BCM **12, 13, 18, or 19**.

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
- `flags` bit0 = **arm** (deadman held — default **button[5]**)

## IPC to the daemon

UDP JSON to `127.0.0.1:5600` (host network on the robot):

```json
{"source":"ros"|"radio","left":0.2,"right":-0.1,"arm":true,"seq":1}
```

## Controller setup — what to run (both machines)

Plug in the SiK radio on **both** sides and the Xbox receiver on the ground PC first.

### 0) Identify devices (each machine)

```bash
# Joystick (ground PC only)
ls -l /dev/input/js*

# SiK radio (both machines) — prefer by-id, see section above
ls -l /dev/serial/by-id/
```

Optional: map axes/buttons on the ground PC before starting compose:

```bash
# stop GS container if it already holds js0
docker stop mini_bream_ground 2>/dev/null
cd <repo>/src
python3 teleop/js_dump.py          # move sticks / press buttons; note indices
```

Current Xbox defaults (after mapping):

| Control | Index | Flag |
|---------|-------|------|
| Left thrust | `axis[1]` | `--left-axis 1` |
| Right thrust | `axis[4]` | `--right-axis 4` |
| Deadman (arm) | `button[5]` | `--deadman-button 5` |

### 1) Robot (Raspberry Pi) — start first

```bash
cd <repo>/src/docker

# Safe bring-up (logs thrust, does not drive ESCs)
DEVICE_CONFIG=rpi PWM_BACKEND_OVERRIDE=dry_run \
RADIO_SERIAL_PORT=/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_<ROBOT_SERIAL>-if00-port0 \
  docker compose -f docker-compose.prod.yml up -d --build pwm_daemon radio_rx

# When ready for real motors, omit PWM_BACKEND_OVERRIDE (uses devices/rpi.json → pigpio hw PWM)
# DEVICE_CONFIG=rpi RADIO_SERIAL_PORT=... docker compose -f docker-compose.prod.yml up -d pwm_daemon radio_rx

# Optional: ROS autonomy / frontseat (separate; not required for radio override)
docker compose -f docker-compose.prod.yml up --build frontseat
```

Watch robot logs:

```bash
docker logs -f mini_bream_pwm_daemon
docker logs -f mini_bream_radio_rx
```

You want `radio_rx` to open the SiK port, and when the GS is armed, `pwm_daemon` should show `Active source → radio`.

### 2) Ground station (PC with Xbox Controller) — start second

```bash
cd <repo>/src/docker

RADIO_SERIAL_PORT=/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_<GS_SERIAL>-if00-port0 \
JOY_DEVICE=/dev/input/js0 \
  docker compose -f docker-compose.ground.yml up --build
```

Watch GS logs:

```bash
docker logs -f mini_bream_ground
```

Expect something like:

```text
Joystick /dev/input/js0 (8 axes, 15 buttons) → serial ... [left=axis[1] right=axis[4] deadman=button[5]]
```

Hold **button[5]** → log `Deadman ARMED`; release → `disarmed` / robot falls back to ROS or stop.

### Order summary

1. Power radios + Xbox receiver  
2. On **Pi**: `pwm_daemon` + `radio_rx` (`docker-compose.prod.yml`)  
3. On **ground PC**: `ground_station` (`docker-compose.ground.yml`)  
4. Hold deadman (**button[5]**), move **axis[1]** / **axis[4]**, confirm Pi daemon logs  

## Failsafe notes

- Ground station always sends frames; with deadman released, `arm=0` and thrust 0.
- If radio goes silent longer than the timeout, radio priority drops immediately.
- If ROS also dies, daemon holds stop.
- `pwm_daemon` should stay up (`restart: unless-stopped`); it is the safety-critical service.
