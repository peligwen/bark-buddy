
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

Rejected (ack ok=false) when `battery_cutoff` is latched.

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
{"type": "cmd_gait_params", "stride_length": 20, "stride_height": 10, "speed": 1.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| stride_length | mm | foot swing distance per step |
| stride_height | mm | foot lift height per step |
| speed | 0.0–1.0 | scales gait frequency |

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

### ping

No-op keepalive. Firmware replies with `ack`.

```json
{"type": "ping"}
```

---

## Telemetry (firmware → host)

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
{"type": "telem_battery", "voltage_mv": 7400, "pct": 80, "low": false}
```

| Field | Description |
|-------|-------------|
| voltage_mv | ADC-derived millivolts (4:1 divider, GPIO 34) |
| pct | 0–100, mapped from 6000–8400 mV |
| low | true when below 6700 mV |

Battery cutoff latches at 6400 mV (3.2 V/cell for 2S LiPo). Requires reboot to clear.

### telem_status

1 Hz. Overall system state.

```json
{"type": "telem_status", "engaged": true, "ramping": false,
 "balance": true, "battery_cutoff": false, "mode": "remote"}
```

| Field | Description |
|-------|-------------|
| engaged | servos powered and at standing pose |
| ramping | engage or disengage ramp in progress |
| balance | active balance enabled |
| battery_cutoff | latched low-battery cutoff; servos off; requires reboot |
| mode | always `"remote"` post Phase 1 |

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

### ack

Sent after every command. `ok: false` means the command was rejected.

```json
{"type": "ack", "ref_type": "cmd_move",   "ok": true}
{"type": "ack", "ref_type": "cmd_engage", "ok": false, "error": "battery_cutoff"}
```

Common rejection reasons: `"battery_cutoff"`, `"not_engaged"`, `"bad_index"`, `"bad_pin"`.
