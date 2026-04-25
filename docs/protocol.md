
# Protocol

All messages are JSON, newline-delimited (NDJSON) over TCP or USB serial.
The host sends commands; firmware streams telemetry and acks.
Maximum message size: 512 bytes (firmware JSON parse buffer).

---

## Commands (host → firmware)

### cmd_move

Move in a direction using the IK gait engine.

```json
{"type": "cmd_move", "direction": "forward", "speed": 1.0}
```

| Field | Type | Values |
|-------|------|--------|
| direction | string | `"forward"` \| `"backward"` \| `"left"` \| `"right"` \| `"rotate_left"` \| `"rotate_right"` \| `"stop"` |
| speed | float | 0.0–1.0; scales gait frequency |

### cmd_stand

Stop gait and return to standing pose.

```json
{"type": "cmd_stand"}
```

### cmd_balance

Enable or disable active balance (pitch/roll compensation via IMU feedback).

```json
{"type": "cmd_balance", "enabled": true}
```

### cmd_engage

Engage or disengage all servos. Engage ramps from lying-down to standing over 2 s.
Disengage ramps back to rest pose and then cuts PWM.

```json
{"type": "cmd_engage", "enabled": true}
```

Rejected (ack ok=false) when `battery_cutoff` is latched **and** the battery is present. USB-only operation (battery switch physically off, `telem_battery.present = false`) is permitted — the cutoff latch never sets in that case, or is cleared automatically within ~3 s if it was set before the battery was switched off.

### cmd_servo

Direct servo pulse width, bypassing IK. Works only when engaged.

```json
{"type": "cmd_servo", "index": 0, "pulse_us": 1500}
```

| Field | Type | Range |
|-------|------|-------|
| index | int | 0–7 |
| pulse_us | int | 500–2500 μs |

### cmd_offset

Set a persistent trim offset for a servo. Saved to NVS (survives reboot).

```json
{"type": "cmd_offset", "index": 0, "offset_us": 0}
```

| Field | Type | Range |
|-------|------|-------|
| index | int | 0–7 |
| offset_us | int | −500 to +500 μs |

### cmd_servo_pin

Reassign a servo index to a different GPIO at runtime.

```json
{"type": "cmd_servo_pin", "index": 0, "pin": 26}
```

Firmware detaches old GPIO, attaches new GPIO to LEDC channel, writes current pulse.
If the new pin is already assigned to another index, they swap automatically.

### cmd_transform

Apply a body-frame transform on top of standing pose.
All fields are optional; omitted fields default to 0.

```json
{"type": "cmd_transform", "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| roll, pitch, yaw | degrees | body rotation |
| x, y, z | mm | body translation |

### cmd_gait_params

Set gait stride parameters. Changes take effect on next gait cycle.

```json
{"type": "cmd_gait_params", "stride_length": 20, "stride_height": 10, "frequency": 1.5}
```

| Field | Unit | Safe range | Default | Description |
|-------|------|------------|---------|-------------|
| stride_length | mm | 0–30 | 12 | foot swing distance per step |
| stride_height | mm | 0–20 | 10 | foot lift height above standing |
| frequency | Hz | 0.5–3.0 | 1.5 | step frequency; firmware applies no bounds — callers must clamp |

Omitted fields revert to compiled defaults (`config.h`). Firmware does not enforce min/max.

### cmd_balance_config

Read or update active-balance PID gains. Omitted fields are unchanged; response echoes all current values.

```json
{"type": "cmd_balance_config", "pitch_kp": 0.3, "pitch_ki": 0.0, "pitch_kd": 0.05,
                                "roll_kp":  0.3, "roll_ki":  0.0, "roll_kd":  0.05}
```

| Field | Description |
|-------|-------------|
| pitch_kp / pitch_ki / pitch_kd | PID gains for pitch axis |
| roll_kp / roll_ki / roll_kd | PID gains for roll axis |

Response is an `ack` with `ok: true` and all six gain fields echoed.
Requires engagement. Default gains: kp=0.3, ki=0.0, kd=0.05; max correction 8°, deadband 0.5°.

### cmd_probe_pin

Wiggle a GPIO ±50 µs around a center pulse to identify an unknown servo pin.
Useful during initial setup or after a wiring change.

```json
{"type": "cmd_probe_pin", "pin": 14, "pulse_us": 1500}
```

| Field | Description |
|-------|-------------|
| pin | GPIO number to probe (must not be reserved or actively driving an engaged servo) |
| pulse_us | center pulse (500–2500 µs; default 1500) |

Responds with `ack` echoing `pin` on success. Rejected (`ok: false`) if the pin is reserved, already driving an engaged servo, or `pin` is omitted.

### cmd_led

Set onboard LED color. `led: 0` is the onboard blue LED (GPIO 18, active-LOW).

```json
{"type": "cmd_led", "led": 0, "r": 0, "g": 15, "b": 0}
```

r, g, b range: 0–255. Firmware applies `LED_BRIGHTNESS` scale (default 40/255).

### cmd_buzzer

Play a tone on the piezo buzzer.

```json
{"type": "cmd_buzzer", "freq_hz": 2400, "duration_ms": 200}
```

`freq_hz: 0` stops immediately. `duration_ms: 0` plays indefinitely until next cmd_buzzer.

### cmd_gpio

GPIO expansion. Valid pins: 32, 33, 1 (UART TX), 3 (UART RX).

```json
{"type": "cmd_gpio", "op": "mode",        "pin": 32, "mode": "input"}
{"type": "cmd_gpio", "op": "write",       "pin": 32, "value": 1}
{"type": "cmd_gpio", "op": "read",        "pin": 32}
{"type": "cmd_gpio", "op": "analog",      "pin": 32}
{"type": "cmd_gpio", "op": "subscribe",   "pin": 32, "interval_ms": 100}
{"type": "cmd_gpio", "op": "unsubscribe", "pin": 32}
```

Read/analog results arrive as `telem_gpio`. Subscribe sends `telem_gpio` at the given interval.

### cmd_i2c

I2C bus operations. `bus: 1` = primary (SDA GPIO 22, SCL GPIO 23); `bus: 2` = secondary (SDA GPIO 19, SCL GPIO 13).

```json
{"type": "cmd_i2c", "op": "scan",  "bus": 1}
{"type": "cmd_i2c", "op": "read",  "bus": 1, "addr": 119, "reg": 0, "len": 2}
{"type": "cmd_i2c", "op": "write", "bus": 1, "addr": 119, "reg": 0, "data": [1, 2]}
```

Results arrive as `telem_i2c`.

### cmd_ota_request_nonce

Request a one-time nonce before a WiFi OTA update. WiFi only; rejected with `wifi_disabled` on serial.

```json
{"type": "cmd_ota_request_nonce"}
```

Firmware responds with `telem_ota_nonce`.

### cmd_ota_update

Trigger a WiFi OTA firmware update. On the WiFi path, `nonce` and `sig` are required (obtained via `cmd_ota_request_nonce` + Ed25519 signing). On the serial path, authentication is omitted (physical access is trusted).

```json
{"type": "cmd_ota_update", "url": "http://192.168.1.2:PORT/firmware.bin", "sha256": "<64-hex>", "nonce": "<64-hex>", "sig": "<128-hex>"}
```

| Field | Required on WiFi | Description |
|-------|-----------------|-------------|
| url | yes | HTTP URL to fetch firmware binary from. Host component must match TCP client IP. |
| sha256 | yes | Expected SHA-256 of firmware binary (hex). |
| nonce | yes | 32-byte nonce issued by `cmd_ota_request_nonce` (hex). |
| sig | yes | Ed25519 signature over `nonce_bytes ‖ sha256_bytes` (128-hex). |

Auth failure error codes: `missing_auth`, `bad_auth_encoding`, `sig` (wrong owner key — device is owned by a different host).

### ping

Keepalive and build-info probe. Firmware replies with `pong`.

```json
{"type": "ping"}
```

Response:

```json
{"type": "pong", "arduino_esp32_core": "3.3.8", "pio_platform": "55.03.38-1"}
```

| Field | Description |
|-------|-------------|
| `arduino_esp32_core` | arduino-esp32 core version the firmware was compiled with |
| `pio_platform` | PlatformIO platform version string (absent in mock builds) |

---

## Telemetry (firmware → host)

### telem_joints

Per-leg IK joint angles in radians, emitted once per gait tick (~20–50 Hz)
while the dog is engaged. The 3D visualization uses these to drive the model
in real time; host/browser clients that don't care can ignore the message.

```json
{"type": "telem_joints",
 "fl": {"h": 0.30, "k": -0.60},
 "fr": {"h": 0.30, "k": -0.60},
 "rl": {"h": 0.30, "k": -0.60},
 "rr": {"h": 0.30, "k": -0.60}}
```

Legs: `fl` front-left, `fr` front-right, `rl` rear-left, `rr` rear-right.
Fields: `h` hip angle (radians), `k` knee angle (radians). Sign convention
matches `firmware/include/ik.h`: positive hip swings the leg forward,
negative knee folds inward.

### telem_imu

50 Hz. Complementary-filter fused orientation.

```json
{"type": "telem_imu", "pitch": 2.1, "roll": -0.3, "yaw": 45.0,
 "ax": 0.02, "ay": -0.01, "az": 9.81, "gx": 0.1, "gy": -0.2, "gz": 0.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| pitch | degrees | positive = nose up |
| roll | degrees | positive = left side up |
| yaw | degrees | integrated gyro; drifts; not suitable for localization |
| ax, ay, az | m/s² | body-frame acceleration |
| gx, gy, gz | deg/s | body-frame angular rate |

### telem_sonar

20 Hz.

```json
{"type": "telem_sonar", "distance_mm": 250}
```

### telem_battery

1 Hz.

```json
{"type": "telem_battery", "voltage_mv": 7400, "pct": 80, "present": true, "low": false}
```

| Field | Description |
|-------|-------------|
| voltage_mv | ADC-derived millivolts (4:1 divider, GPIO 34) |
| pct | 0–100, mapped from 6000–8400 mV |
| present | `false` when battery is physically absent (USB-only; ADC reads < 4500 mV for 3+ consecutive seconds) |
| low | `true` when battery is present and below 6700 mV |

Battery cutoff latches at 6400 mV (3.2 V/cell for 2S LiPo). Requires reboot to clear **unless** the battery is subsequently detected as absent (physically disconnected), in which case the latch clears automatically and a `battery_absent_clear_latch` event is emitted.

### telem_status

1 Hz. Overall system state.

```json
{"type": "telem_status", "engaged": true, "ramping": false,
 "balance": true, "battery_cutoff": false, "ms_since_last_host_msg": 120}
```

| Field | Description |
|-------|-------------|
| engaged | servos powered and at standing pose |
| ramping | engage or disengage ramp in progress |
| balance | active balance enabled |
| battery_cutoff | latched low-battery cutoff; servos off; requires reboot |
| ms_since_last_host_msg | milliseconds since firmware last received a host message; used as a watchdog liveness indicator |

### telem_button

Physical button K1 (GPIO 5) events.

```json
{"type": "telem_button", "event": "press"}
```

`event`: `"press"` | `"release"` | `"long_press"` (≥ 1000 ms)

### telem_gpio

GPIO subscription tick or read result.

```json
{"type": "telem_gpio", "pin": 32, "digital": 1, "analog": -1}
```

`analog`: ADC reading in raw counts, or -1 if not requested / pin not ADC-capable.

### telem_i2c

I2C operation result.

```json
{"type": "telem_i2c", "bus": 1, "op": "scan",  "addrs": [106, 119]}
{"type": "telem_i2c", "bus": 1, "op": "read",   "data": [0, 15]}
{"type": "telem_i2c", "bus": 1, "op": "write",  "ok": true}
```

### telem_servo_pins

Current servo-index → GPIO mapping. Sent on client connect and after `cmd_servo_pin`.

```json
{"type": "telem_servo_pins", "pins": [25, 26, 27, 14, 16, 17, 15, 2]}
```

Array index matches servo index (0 = FL_hip … 7 = RR_knee).

### telem_ota_nonce

One-time nonce for signing a WiFi OTA command. Sent in response to `cmd_ota_request_nonce`.

```json
{"type": "telem_ota_nonce", "nonce": "<64-hex>"}
```

| Field | Description |
|-------|-------------|
| nonce | 32-byte random nonce encoded as 64 lowercase hex characters. Valid for `OTA_NONCE_TTL_MS` (30 s). Single use — consumed on first verify attempt. |

### telem_event

Asynchronous lifecycle events. Emitted outside the 1 Hz status cadence, immediately when the triggering condition occurs.

```json
{"type": "telem_event", "event": "engage_complete", "t": 12450}
{"type": "telem_event", "event": "heartbeat_detach", "t": 22100, "ms_since_last_msg": 10001}
{"type": "telem_event", "event": "battery_cutoff_detach", "t": 31000, "mv": 6350}
```

| `event` value | When emitted |
|---------------|-------------|
| `engage_start` | Engage ramp begins (cmd_engage enabled=true accepted) |
| `engage_complete` | Engage ramp finishes; servos fully powered at standing pose |
| `disengage_start` | Disengage ramp begins (cmd_engage enabled=false accepted) |
| `disengage_complete` | Disengage ramp finishes; PWM cut |
| `heartbeat_detach` | Host went silent for > HEARTBEAT_TIMEOUT_MS; servos detached. Includes `ms_since_last_msg` field. |
| `battery_cutoff_detach` | Battery voltage dropped below cutoff threshold; servos detached and latch set. Includes `mv` field. |
| `battery_absent_clear_latch` | Battery absent (USB-only) confirmed; stale cutoff latch cleared. Includes `mv` field. |

`t` is `millis()` at event time (uint32, wraps after ~49 days).

### ack

Sent after every command. `ok: false` means the command was rejected.

```json
{"type": "ack", "ref_type": "cmd_move",   "ok": true}
{"type": "ack", "ref_type": "cmd_engage", "ok": false, "error": "battery_cutoff"}
```

Common rejection reasons: `"battery_cutoff"`, `"not_engaged"`, `"bad_index"`, `"bad_pin"`.
