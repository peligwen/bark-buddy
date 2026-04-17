# Bark-Buddy Firmware Protocol Reference

JSON/NDJSON (newline-delimited JSON) over WiFi TCP or USB serial. The firmware
listens on TCP port 9000; the host connects as a client. One JSON object per
line, UTF-8 encoded.

For pin/peripheral context, see `docs/hardware-schematic.md`.

---

## Transport

| Mode | Details |
|------|---------|
| WiFi TCP | Port 9000, primary path |
| USB Serial | 115200 baud, 8N1, debug/flash path |
| Local TCP (mock) | Port 9001 (default), `bark mock` spawns `firmware/test/bark-mock` |

The Python host uses `FirmwareTransport` for all three modes. Auto-detected:
USB serial JSON ping → mDNS `_mechdog._tcp` → exit with guidance. Override
with `--fw-tcp HOST[:PORT]`.

---

## Commands (Host → Firmware)

All commands receive an `{"type":"ack","ref_type":"<cmd_type>","ok":true}` (or
`"ok":false,"error":"..."`) in response unless noted.

### Motion

| Type | Required fields | Optional fields | Firmware behavior |
|------|----------------|-----------------|-------------------|
| `cmd_move` | `direction` (forward/backward/left/right/stop) | `speed` (float, default 1.0) | Start/update gait; speed scales stride frequency |
| `cmd_stand` | — | — | Stop gait, interpolate to standing pose over `STAND_RETURN_MS` |
| `cmd_balance` | `enabled` (bool) | — | Enable/disable IMU-driven balance correction |
| `cmd_transform` | — | `roll`, `pitch`, `yaw`, `x`, `y`, `z` (all float, degrees/mm) | Apply body transform offset on top of standing/gait pose |
| `cmd_gait_params` | — | `stride_length` (mm), `stride_height` (mm), `speed` (float) | Update gait parameters live; takes effect next stride |

### Servo / Offset

| Type | Required fields | Optional fields | Notes |
|------|----------------|-----------------|-------|
| `cmd_servo` | `index` (0–7), `pulse_us` (int) | — | Direct servo control; requires `PINS_VERIFIED=1`. Indices 8–10 require `AUX_SERVOS_ENABLED=1` |
| `cmd_offset` | `index` (0–7), `offset_us` (int) | — | Persistent servo trim; stored in NVS via Preferences |

### Engage / LED

| Type | Required fields | Optional fields | Firmware behavior |
|------|----------------|-----------------|-------------------|
| `cmd_engage` | `enabled` (bool) | — | Master engage/disengage. Disengage ramps to rest pose over `SHUTDOWN_RAMP_MS`, then detaches servos. Battery cutoff latches until power-cycle |
| `cmd_led` | `led` (int, 0=onboard, 1=sonar RGB front, 2=sonar RGB rear) | `r`, `g`, `b` (0–255) | Set LED color. `led=0` drives GPIO 18 (active-LOW blue LED). `led=1/2` controls sonar module RGB via `sonar_set_rgb()` |

### Buzzer

| Type | Required fields | Optional fields | Firmware behavior |
|------|----------------|-----------------|-------------------|
| `cmd_buzzer` | `freq_hz` (int) | `duration_ms` (int) | Play tone at `freq_hz` via LEDC ch 9 (GPIO 21). `freq_hz=0` stops immediately. `duration_ms` omitted or 0 = continuous until next cmd_buzzer |

### GPIO Expansion

Operates on auxiliary GPIO pins: 32 (J5), 33 (J5), 1 (UART0 TX header), 3 (UART0 RX header).
Pins 1 and 3 are not ADC-capable — analog reads on these pins return `analog: -1`.

| Type | `op` value | Additional fields | Result |
|------|-----------|-------------------|--------|
| `cmd_gpio` | `mode` | `pin`, `mode_val` ("input"/"output"/"input_pullup") | Configure pin mode |
| `cmd_gpio` | `write` | `pin`, `value` (0/1) | Drive pin HIGH or LOW |
| `cmd_gpio` | `read` | `pin` | Returns `telem_gpio` immediately |
| `cmd_gpio` | `analog` | `pin` | Returns `telem_gpio` with `analog` field (ADC raw); -1 if not ADC-capable |
| `cmd_gpio` | `subscribe` | `pin`, `interval_ms` (optional) | Stream `telem_gpio` on change or interval |
| `cmd_gpio` | `unsubscribe` | `pin` | Stop streaming |

### I2C

Operates on I2C bus 1 (SDA=22, SCL=23) or bus 2 (SDA=19, SCL=13 / J4 connector).
See `docs/hardware-schematic.md` for bus topology.

**Note:** `cmd_i2c_write` (bare write without `op` field) is deprecated. Use
`cmd_i2c` with `op="write"` instead.

| Type | `op` value | Additional fields | Result |
|------|-----------|-------------------|--------|
| `cmd_i2c` | `scan` | `bus` (1 or 2) | Returns `telem_i2c` with `addrs` list |
| `cmd_i2c` | `read` | `bus`, `addr`, `reg` (optional), `len` | Returns `telem_i2c` with `data` (byte array) |
| `cmd_i2c` | `write` | `bus`, `addr`, `reg` (optional), `data` (byte array) | Writes bytes; returns ack |

### Connection

| Type | Fields | Firmware behavior |
|------|--------|-------------------|
| `ping` | — | Responds with `{"type":"pong"}`. Resets heartbeat timer (10s timeout) |

---

## Telemetry (Firmware → Host)

Firmware streams telemetry continuously. Rates are configured in `firmware/include/config.h`.

### Continuous Streams

| Type | Fields | Rate | Notes |
|------|--------|------|-------|
| `telem_imu` | `pitch`, `roll`, `yaw` (deg), `ax`, `ay`, `az` (g), `gx`, `gy`, `gz` (deg/s) | 50 Hz | Complementary-filtered; axis order: roll first from QMI8658 read_angle() |
| `telem_sonar` | `distance_mm` (int) | 20 Hz | I2C ultrasonic at 0x77; -1 if out of range |
| `telem_battery` | `voltage_mv` (int), `pct` (int 0–100), `low` (bool) | 1 Hz | GPIO 34, 30K/10K divider, calibrated factor 3.9 |
| `telem_status` | `engaged` (bool), `ramping` (bool), `balance` (bool), `battery_cutoff` (bool) | 1 Hz | Overall system state |

### Event / On-Demand

| Type | Fields | When emitted |
|------|--------|-------------|
| `ack` | `ref_type`, `ok` (bool), `error` (string, on failure) | After each command |
| `pong` | — | After each `ping` |
| `boot` | `fw_version`, `imu` (bool), `sonar` (bool), `servos` (bool), `pins_verified` (bool) | Once on startup; re-sent to each new TCP client |
| `telem_button` | `event` ("press"/"release"/"long_press") | Physical button K1 (GPIO 5, active-LOW, 10K pullup, 20ms debounce, 1000ms long-press threshold) |
| `telem_gpio` | `pin` (int), `digital` (0/1), `analog` (int or -1) | After `cmd_gpio op=read/analog`; continuously on subscription |
| `telem_i2c` | `bus` (int), `op` ("scan"/"read"/"write"), `addrs` (scan), `data` (read), `ok` (bool) | After `cmd_i2c` |

---

## Connection Management

- Host sends `ping` periodically; firmware responds with `pong`
- Heartbeat timeout: 10s without any message → stop gait, set LEDs blue (disconnected state)
- On new TCP client: firmware sends `boot` message
- Host retries TCP connection with exponential backoff on disconnect
- Battery cutoff: when `voltage_mv` falls below `BATTERY_LOW_MV` (6400 mV), `telem_status.battery_cutoff` latches true; `cmd_engage enabled=true` is rejected until power-cycle

---

## LEDC Channel Allocation

All PWM on the ESP32 LEDC hardware peripheral:

| Channels | Assignment |
|----------|-----------|
| 0–7 | Main servos (firmware indices 0–7) |
| 8 | Pin probe (internal diagnostic) |
| 9 | Buzzer (GPIO 21, via S8050 NPN) |
| 10–12 | Aux servos (GPIO 15, 0, 12 — behind `AUX_SERVOS_ENABLED=1`) |

---

## WebSocket Protocol (Browser ↔ Host)

The Python host bridges browser WebSocket commands to firmware JSON commands.
The browser never communicates directly with firmware.

### Control Lock

Only one browser client can control the robot at a time. Lock auto-acquired on
first control command; times out after 30s of inactivity.

**Browser → Host (lock management):**

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_identify` | `name` | Register client name |
| `cmd_lock` | `name` | Request control lock |
| `cmd_unlock` | — | Release lock |
| `cmd_lock_yield` | — | Voluntarily yield to challenger |

**Host → Browser (lock events):**

| Type | Fields | Description |
|------|--------|-------------|
| `lock_status` | `locked`, `holder`, `is_mine` | Broadcast on lock change |
| `lock_challenge` | `challenger` | Sent to holder when another client requests |
| `lock_denied` | `holder` | Sent to requester when lock held |

### Control Commands (require lock)

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_move` | `direction` | Forwarded to firmware |
| `cmd_stand` | — | Forwarded to firmware |
| `cmd_balance` | `enabled` | Forwarded to firmware |
| `cmd_scan` | `action` (start/stop) | 360° ultrasonic scan behavior |
| `cmd_patrol` | `action`, `waypoints` | Waypoint patrol behavior |

### Utility Commands (no lock)

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_map` | `action` (get/clear) | Map data management |
| `cmd_reset` | — | Reset dead reckoning |

### Telemetry (Host → Browser)

Host forwards firmware telemetry and adds odometry:

| Type | Fields | Description |
|------|--------|-------------|
| `telem_imu` | `pitch`, `roll`, `yaw` | Forwarded from firmware |
| `telem_sonar` | `distance_mm` | Forwarded from firmware |
| `telem_battery` | `voltage_mv`, `pct`, `low` | Forwarded from firmware |
| `telem_status` | `engaged`, `ramping`, `balance`, `battery_cutoff` | Forwarded from firmware |
| `telem_button` | `event` | Forwarded from firmware |
| `telem_odometry` | `x`, `y`, `heading`, `motion` | Dead-reckoned position (host-computed) |
| `scan_point` | `x`, `y`, `distance_mm` | Live point during scan |
| `scan_complete` | — | 360° scan finished |
| `map_data` | `bounds`, `points`, `walls`, `chains`, `scans`, `scan_count`, `point_count` | Full map snapshot |
