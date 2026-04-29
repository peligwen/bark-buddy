
# Protocol

Bark-Buddy has two wire protocols, both JSON-based:

| Hop | Format | Transport |
|-----|--------|-----------|
| Host ↔ Firmware | NDJSON (newline-delimited) | USB serial or TCP |
| Browser ↔ Host | JSON text frames | WebSocket |

Maximum firmware-side message size: 512 bytes (firmware JSON parse buffer).
The host-side protocol re-frames most firmware telemetry verbatim and adds a
small set of host-originated messages for session, lock, and lifecycle.

This document is the contract. If a behavior isn't here, it isn't committed to.

---

# Part 1: Host ↔ Firmware

The host sends commands; firmware streams telemetry, asynchronous events, and acks.

## 1.1 Commands (host → firmware)

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

Idempotency: a `cmd_engage` whose target state matches current state returns `ok=true` with no side effects.

### cmd_servo

Direct servo pulse width, bypassing IK. Works only when engaged.

```json
{"type": "cmd_servo", "index": 0, "pulse_us": 1500}
```

| Field | Type | Range |
|-------|------|-------|
| index | int | 0–7 (main legs); 8–10 (auxiliary servos, only when `AUX_SERVOS_ENABLED=1`) |
| pulse_us | int | 500–2500 μs |

Ack includes `actual_us` (the post-clamp/offset pulse the firmware actually wrote)
and `written` (false if the gait engine clamped or absorbed the write).

### cmd_offset

Set a persistent trim offset for a servo.

```json
{"type": "cmd_offset", "index": 0, "offset_us": 0, "op": "set"}
```

| Field | Type | Range / Values |
|-------|------|----------------|
| index | int | 0–7 |
| offset_us | int | −500 to +500 μs (optional; if omitted, no offset is changed) |
| op | string | `"set"` (default — RAM only, fast for slider scrubbing) \| `"save"` (also flush to NVS) |

Set/save are intentionally separate to avoid wearing flash during calibration. Ack always echoes the full 8-element `offsets` array.

### cmd_servo_pin

Reassign a servo index to a different GPIO at runtime (not persisted).

```json
{"type": "cmd_servo_pin", "index": 0, "pin": 26}
```

If the new pin is already assigned to another index, they swap automatically.
Successful response triggers a `telem_servo_pins` broadcast.

### cmd_transform

Apply a body-frame transform on top of standing pose.
All fields are optional; omitted fields default to 0.

```json
{"type": "cmd_transform", "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                          "x": 0.0, "y": 0.0, "z": 0.0, "ms": 100}
```

| Field | Unit | Description |
|-------|------|-------------|
| roll, pitch, yaw | degrees | body rotation |
| x, y, z | mm | body translation |
| ms | int | interpolation duration to reach the target pose (default 100) |

### cmd_gait_params

Set gait stride parameters. Changes blend in over the next ~500 ms; the firmware
interpolates `swing_time_ms` and `stand_time_ms` to avoid a step-discontinuity in
the duty factor.

```json
{"type": "cmd_gait_params", "stride_length": 20, "stride_height": 10,
                            "frequency": 1.5,
                            "swing_time_ms": 150, "stand_time_ms": 200}
```

| Field | Unit | Safe range | Default | Description |
|-------|------|------------|---------|-------------|
| stride_length | mm | 0–30 | 12 | foot swing distance per step |
| stride_height | mm | 0–20 | 10 | foot lift height above standing |
| frequency | Hz | 0.5–3.0 | 1.5 | step frequency; firmware applies no bounds — callers must clamp |
| swing_time_ms | ms | 50–500 | 150 | time foot is in air per step |
| stand_time_ms | ms | 50–500 | 200 | time foot is on ground per step |

Duty factor = `stand_time_ms / (swing_time_ms + stand_time_ms)`.
Omitted fields revert to compiled defaults (`config.h`). Firmware does not enforce min/max.

### cmd_balance_config

Read or update active-balance PID gains. Omitted fields are unchanged; response echoes all current values.

```json
{"type": "cmd_balance_config", "pitch_kp": 0.3, "pitch_ki": 0.0, "pitch_kd": 0.05,
                                "roll_kp":  0.3, "roll_ki":  0.0, "roll_kd":  0.05}
```

Response is an `ack` with `ok: true` and all six gain fields echoed.
Requires engagement. Default gains: kp=0.3, ki=0.0, kd=0.05; max correction 8°, deadband 0.5°.

### cmd_yaw_trim

Read or update the persistent yaw-drift trim applied during forward/backward
walking. Compensates for asymmetric leg behaviour without re-tuning each pose.

```json
{"type": "cmd_yaw_trim", "op": "set",  "value": 0.05}
{"type": "cmd_yaw_trim", "op": "save"}
{"type": "cmd_yaw_trim", "op": "get"}
```

| Field | Type | Range / Values |
|-------|------|----------------|
| op | string | `"get"` (default) \| `"set"` (update RAM) \| `"save"` (flush RAM to NVS) |
| value | float | −0.7 to +0.7; positive = bias left, negative = bias right |

Ack always echoes the current `value`. Loaded at boot from NVS.

### cmd_probe_pin

Wiggle a GPIO ±50 µs around a center pulse to identify an unknown servo pin.

```json
{"type": "cmd_probe_pin", "pin": 14, "pulse_us": 1500}
```

| Field | Description |
|-------|-------------|
| pin | GPIO number to probe (must not be reserved or actively driving an engaged servo) |
| pulse_us | center pulse (500–2500 µs; default 1500) |

Responds with `ack` echoing `pin` on success. Rejected (`ok: false`) if the pin is reserved (`error: "pin_reserved:<reason>"`), already driving an engaged servo (`servo_pin_engaged`), or `pin` is omitted (`invalid_pin`).

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
{"type": "cmd_gpio", "op": "mode",        "pin": 32, "mode": "input_floating"}
{"type": "cmd_gpio", "op": "write",       "pin": 32, "value": 1}
{"type": "cmd_gpio", "op": "read",        "pin": 32}
{"type": "cmd_gpio", "op": "analog",      "pin": 32}
{"type": "cmd_gpio", "op": "subscribe",   "pin": 32, "interval_ms": 100, "mode": "input_pullup"}
{"type": "cmd_gpio", "op": "unsubscribe", "pin": 32}
```

`mode` values: `"input_floating"` (default), `"input_pullup"`, `"input_pulldown"`, `"output"`.
Read/analog results arrive as `telem_gpio`. Subscribe sends `telem_gpio` at the given interval.

### cmd_i2c

I2C bus operations. `bus: 1` = primary (SDA GPIO 22, SCL GPIO 23); `bus: 2` = secondary (SDA GPIO 19, SCL GPIO 13).

```json
{"type": "cmd_i2c", "op": "scan",  "bus": 1}
{"type": "cmd_i2c", "op": "read",  "bus": 1, "addr": 119, "reg": 0, "len": 2}
{"type": "cmd_i2c", "op": "write", "bus": 1, "addr": 119, "reg": 0, "data": [1, 2]}
```

Results arrive as `telem_i2c`. `cmd_i2c op:scan` is rejected with `scan_blocked_during_motion` while balance is enabled or gait is non-STOP, because a scan stalls the sensor task ~130 ms.

### cmd_ota_request_nonce

Request a one-time nonce before a WiFi OTA update. WiFi only; rejected with `wifi_disabled` on serial.

```json
{"type": "cmd_ota_request_nonce"}
```

Firmware responds with `telem_ota_nonce`.

### cmd_ota_update

Trigger a WiFi OTA firmware update. On the WiFi path, all of `nonce`, `sig`, and a non-empty `sha256` are required. On the serial path, authentication and sha256 are optional (physical access is trusted).

```json
{"type": "cmd_ota_update", "url": "http://192.168.1.2:PORT/firmware.bin",
                            "sha256": "<64-hex>",
                            "nonce":  "<64-hex>",
                            "sig":    "<128-hex>"}
```

| Field | Required on WiFi | Description |
|-------|-----------------|-------------|
| url | yes | HTTP URL to fetch firmware binary from. Host component must match TCP client IP. URLs with `@` in the authority are rejected (userinfo bypass prevention). |
| sha256 | yes (must be non-empty) | Expected SHA-256 of firmware binary (64 lowercase hex chars). |
| nonce | yes | 32-byte nonce issued by `cmd_ota_request_nonce` (hex). |
| sig | yes | Ed25519 signature over `nonce_bytes ‖ sha256_bytes` (128-hex). |

Auth/validation failure error codes:
`missing_url`, `missing_auth`, `missing_sha256`, `bad_auth_encoding`, `sig`, `url_not_allowed`.

OTA progress is reported via `ota_status` messages.

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

## 1.2 Telemetry (firmware → host)

### boot

Sent once on firmware startup, after sensors are initialised.

```json
{"type": "boot", "imu": true, "sonar": true, "pins_verified": true,
 "fw_version": "0.4.0", "fw_build": "2026-04-23T15:42:00Z",
 "gpio2_at_boot": 1}
```

| Field | Description |
|-------|-------------|
| imu, sonar | sensor presence at probe time |
| pins_verified | `PINS_VERIFIED` compile flag |
| fw_version | semver string from `config.h` |
| fw_build | ISO-8601 build timestamp |
| gpio2_at_boot | GPIO 2 strap state at boot, captured before any other peripheral attaches |

### telem_joints

Per-leg IK joint angles in radians, emitted once per gait tick (~20–50 Hz)
while the dog is engaged. The 3D visualization uses these to drive the model
in real time; clients that don't care can ignore the message.

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
 "balance": true, "battery_cutoff": false, "ms_since_last_host_msg": 120,
 "wifi": true, "wifi_ip": "192.168.1.42", "tcp_port": 9000}
```

| Field | Description |
|-------|-------------|
| engaged | servos powered and at standing pose |
| ramping | engage or disengage ramp in progress |
| balance | active balance enabled |
| battery_cutoff | latched low-battery cutoff; servos off; requires reboot |
| ms_since_last_host_msg | milliseconds since firmware last received a host message; used as a watchdog liveness indicator |
| wifi, wifi_ip, tcp_port | WiFi association state (only present when WiFi-built and connected) |

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
{"type": "telem_i2c", "bus": 1, "op": "read",  "addr": 119, "reg": 0, "data": [0, 15]}
{"type": "telem_i2c", "bus": 1, "op": "write", "ok": true}
```

### telem_servo_pins

Current servo-index → GPIO mapping. Sent on client connect and after `cmd_servo_pin`.

```json
{"type": "telem_servo_pins", "pins": [25, 26, 16, 17, 27, 14, 15, 2]}
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

### ota_status

Asynchronous OTA progress.

```json
{"type": "ota_status", "status": "downloading"}
{"type": "ota_status", "status": "flashing"}
{"type": "ota_status", "status": "complete"}
{"type": "ota_status", "status": "failed", "error": "hash_mismatch"}
```

`status` values: `"downloading"`, `"flashing"`, `"complete"`, `"failed"`.
On `failed`, `error` is one of: `http_error`, `update_begin_failed`, `flash_error`,
`download_timeout`, `incomplete_download`, `hash_compute_error`, `hash_mismatch`.

### telem_event

Asynchronous lifecycle / safety events. Emitted outside the 1 Hz status cadence,
immediately when the triggering condition occurs.

```json
{"type": "telem_event", "event": "engage_complete", "t": 12450}
{"type": "telem_event", "event": "heartbeat_detach", "t": 22100, "ms_since_last_msg": 10001}
{"type": "telem_event", "event": "battery_cutoff_detach", "t": 31000, "mv": 6350}
{"type": "telem_event", "event": "tilt_fault", "t": 41500, "pitch": 47.2, "roll": -3.1}
```

| `event` value | When emitted | Extra fields |
|---------------|-------------|--------------|
| `engage_start` | Engage ramp begins (cmd_engage enabled=true accepted) | — |
| `engage_complete` | Engage ramp finishes; servos fully powered at standing pose | — |
| `disengage_start` | Disengage ramp begins (cmd_engage enabled=false accepted) | — |
| `disengage_complete` | Disengage ramp finishes; PWM cut | — |
| `heartbeat_detach` | Host went silent for > HEARTBEAT_TIMEOUT_MS; servos detached | `ms_since_last_msg` |
| `battery_cutoff_detach` | Battery voltage dropped below cutoff; servos detached and latch set | `mv` |
| `battery_absent_clear_latch` | Battery absent (USB-only) confirmed; stale cutoff latch cleared | `mv` |
| `tilt_fault` | Pitch or roll exceeded `BALANCE_TILT_CUTOFF_DEG`; gait stops, balance off until tilt clears for `BALANCE_TILT_HOLD_MS` | `pitch`, `roll` |

`t` is `millis()` at event time (uint32, wraps after ~49 days).

### ack

Sent after every command. `ok: false` means the command was rejected.

```json
{"type": "ack", "ref_type": "cmd_move",   "ok": true}
{"type": "ack", "ref_type": "cmd_engage", "ok": false, "error": "battery_cutoff"}
```

Common rejection reasons: `"battery_cutoff"`, `"not_engaged"`, `"ramping"`, `"bad_index"`, `"bad_pin"`, `"missing_params"`, `"unknown_op"`, `"timeout"`, `"unknown_type"`.

### error

Non-fatal firmware error not tied to a specific command. Mostly used for
parser failures and unexpected internal conditions.

```json
{"type": "error", "msg": "json parse failed: <detail>"}
```

---

# Part 2: Browser ↔ Host

The host serves the static web UI and a single WebSocket endpoint at `/ws`.
All messages are UTF-8 JSON text frames. Most firmware telemetry is forwarded
verbatim; this section documents only host-originated messages and host-only
commands.

## 2.1 Browser → Host commands

In addition to the firmware-direct passthroughs (any `cmd_*` listed in Part 1
that targets the firmware is forwarded as-is), the host accepts:

### cmd_identify

Set the operator name shown in lock-status messages. No response.

```json
{"type": "cmd_identify", "name": "Gwen"}
```

### cmd_lock

Acquire the control lock. If currently locked by another client, the holder
receives a `lock_challenge` and the requester receives `lock_denied`.

```json
{"type": "cmd_lock", "name": "Gwen"}
```

### cmd_unlock

Voluntarily release the control lock. No-op if the caller does not hold it.

```json
{"type": "cmd_unlock"}
```

### cmd_lock_yield

Identical effect to `cmd_unlock` (release if holder), but signals that the
release is in response to a `lock_challenge`. Reserved for future telemetry.

```json
{"type": "cmd_lock_yield"}
```

### cmd_reset

Reset host-side session state (motion, mode) and recompute the static-asset
hash so connected clients reload if files changed. Broadcast to all clients.

```json
{"type": "cmd_reset"}
```

### cmd_restart_server

Re-exec the host process. Used during development. Broadcasts `server_restarting`
before exiting; clients reconnect when the new process is up.

```json
{"type": "cmd_restart_server"}
```

### Lock semantics

The control lock guards motion-affecting commands (`cmd_move`, `cmd_stand`,
`cmd_balance`, `cmd_engage`). Other commands (`cmd_servo`, `cmd_offset`, OTA,
diagnostics, etc.) bypass the lock — they are session-scoped tools, not
operator-authority commands. The lock auto-releases after 30 s of inactivity.
The first lock-gated command from a client auto-acquires the lock if no one
holds it.

## 2.2 Host → Browser messages

### telem_status (host-augmented)

The host augments the firmware `telem_status` with session-level fields before
forwarding to browsers:

```json
{"type": "telem_status", "engaged": true, "ramping": false,
 "balance": true, "battery_cutoff": false, "battery_present": true,
 "transport": "fw-usb:/dev/cu.usbserial-10",
 "fw_version": "0.4.0", "available_fw_version": "0.4.1",
 "battery_mv": 7400, "ota_status": null,
 "fallen": false, "connected": true, "lifecycle": "active"}
```

| Field | Description |
|-------|-------------|
| transport | `"mock"` \| `"fw-usb:<port>"` \| `"fw-wifi:<ip>"` \| `"fw"` |
| fw_version | running firmware version (from `boot`) |
| available_fw_version | local firmware binary version, for OTA UI |
| battery_mv | most recent battery voltage |
| ota_status | one of the `ota_status.status` strings, or null |
| connected | transport is open |
| fallen | balance layer detected a fall |
| lifecycle | derived view: `"ramping"` \| `"active"` \| `"disengaged"` \| `"unknown"` |

### telem_odometry

Browser-facing odometry feed. Emitted at 20 Hz from the telemetry loop.

```json
{"type": "telem_odometry", "motion": "forward", "heading": 12.4,
                            "x": 0.0, "y": 0.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| motion | string | last accepted `cmd_move.direction` or `"stand"` / `"stop"` |
| heading | degrees | IMU yaw (drifts; not for localization) |
| x, y | mm | dead-reckoned position; reserved, currently always 0 |

### event_fall / event_recovered

Emitted by the host balance layer when pitch/roll exceeds / clears the fall
threshold. The IMU snapshot at the moment of fall is included.

```json
{"type": "event_fall", "imu": {"pitch": 62.0, "roll": 1.5, "yaw": 0.0}}
{"type": "event_recovered"}
```

### balance_state

Emitted on `cmd_balance` accept; reflects the host-side balance-layer state.

```json
{"type": "balance_state", "enabled": true}
```

### lock_status

Sent on `cmd_lock`/`cmd_unlock` accept and on every WS connect.

```json
{"type": "lock_status", "locked": true, "holder": "Gwen", "is_you": true}
```

`is_you` is computed per-client.

### lock_challenge

Sent only to the current holder when another client requests the lock.

```json
{"type": "lock_challenge", "challenger": "Pat"}
```

### lock_denied

Sent to a client whose `cmd_lock` was rejected, or whose lock-gated command
arrived without holding the lock.

```json
{"type": "lock_denied", "holder": "Gwen"}
```

### version

Sent on connect and after `cmd_reset`. Browser uses this to detect that
static assets have changed and reload.

```json
{"type": "version", "hash": "a1b2c3d4"}
```

### reset

Confirmation broadcast for `cmd_reset`.

```json
{"type": "reset"}
```

### server_restarting

Best-effort notification before the host re-execs. Clients should expect the
WebSocket to close immediately and reconnect.

```json
{"type": "server_restarting"}
```

---

## 2.3 Forwarded firmware messages

The following firmware messages pass through the host unchanged (or with
trivial augmentation): `telem_imu`, `telem_sonar`, `telem_battery`,
`telem_button`, `telem_gpio`, `telem_i2c`, `telem_servo_pins`, `telem_joints`,
`telem_event`, `boot`, `ota_status`, `ack`. See Part 1 for shapes.
