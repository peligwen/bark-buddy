# Communication Protocol

Two firmware paths share the same Python host and web UI. The custom C++ firmware over WiFi is the primary path; the stock MicroPython firmware over serial/WebREPL is the fallback.

---

## Custom Firmware Protocol (Primary)

JSON/NDJSON (newline-delimited JSON) over WiFi TCP or USB serial. The firmware listens on TCP port 9000; the host connects as a client.

### Transport

- **WiFi TCP:** Port 9000, JSON/NDJSON (primary)
- **USB Serial:** 115200 baud, 8N1, JSON/NDJSON (debug)
- **Encoding:** UTF-8, one JSON object per line

### Commands (Host → Firmware)

| Type | Example | Description |
|------|---------|-------------|
| `cmd_move` | `{"type":"cmd_move","direction":"forward","speed":1.0}` | Move: forward, backward, left, right, stop |
| `cmd_stand` | `{"type":"cmd_stand"}` | Stand in neutral pose |
| `cmd_balance` | `{"type":"cmd_balance","enabled":true}` | Enable/disable balance correction |
| `cmd_servo` | `{"type":"cmd_servo","index":0,"pulse_us":1500}` | Direct servo control (requires `PINS_VERIFIED`) |
| `cmd_led` | `{"type":"cmd_led","led":1,"r":0,"g":15,"b":0}` | Set sonar module LED color |
| `ping` | `{"type":"ping"}` | Heartbeat ping |

### Telemetry (Firmware → Host)

| Type | Fields | Rate |
|------|--------|------|
| `telem_imu` | `pitch`, `roll`, `yaw`, `ax`, `ay`, `az`, `gx`, `gy`, `gz` | 50 Hz |
| `telem_sonar` | `distance_mm` | 20 Hz |
| `telem_battery` | `voltage_mv`, `pct`, `low` | 1 Hz |
| `telem_status` | `mode`, `balance`, `servos`, `low_battery` | 1 Hz |
| `ack` | `ref_type`, `ok`, `error` (optional) | Per command |
| `boot` | `imu`, `sonar`, `servos`, `pins_verified` | Once on startup |
| `pong` | — | Per ping |

### Events (Firmware → Host)

| Type | Description |
|------|-------------|
| `event_fall` | Fall detected (tilt exceeds threshold) |
| `event_recovered` | Recovery from fall completed |

### Connection Management

- Host sends `ping` periodically; firmware responds with `pong`
- Firmware timeout: 5s without any message → stop gait, set LEDs to blue (disconnected)
- On reconnect: firmware sends `boot` message, LEDs turn green
- Host retries TCP connection with exponential backoff on disconnect

---

## Stock Firmware Protocol (Fallback)

Text-based CMD protocol injected via MicroPython REPL. Used for bootstrapping, pin discovery, and when custom firmware isn't flashed.

### Transport

- **USB Serial REPL:** 115200 baud, 8N1 — host sends Python expressions to REPL
- **WiFi WebREPL:** Same REPL commands over WebSocket — fallback wireless path
- **Encoding:** ASCII text

### REPL Commands

The host translates high-level commands into Python expressions evaluated on the REPL:

| Action | REPL Command | Response |
|--------|-------------|----------|
| Move forward | `_dog.move(35, 0)` | — |
| Move backward | `_dog.move(-35, 0)` | — |
| Turn left | `_dog.move(0, -35)` | — |
| Turn right | `_dog.move(0, 35)` | — |
| Stop | `_dog.move(0, 0)` | — |
| Read IMU | `_imu.read_angle()` | `[pitch, roll]` |
| Read sonar | `_sonar.getDistance()` | distance in cm (float) |
| Read battery | (via CMD protocol) | millivolts |
| Action group | `_dog.action_run(code)` | — (1=wave, 4=sit, 5=lie down) |
| Enable balance | `_dog.homeostasis(1)` | — |
| Disable balance | `_dog.homeostasis(0)` | — |

### Stock CMD Protocol (Legacy)

Some operations use the stock CMD text format: `CMD|<function_code>|<sub_function>|<data>|$`

| Code | Function | Example | Response |
|------|----------|---------|----------|
| 1 | Posture adjustment | `CMD\|1\|3\|1\|$` (enable balance) | `CMD\|1\|OK\|$` |
| 2 | Action groups | `CMD\|2\|1\|4\|$` (sit down) | `CMD\|2\|OK\|$` |
| 3 | Motion control | `CMD\|3\|3\|$` (forward) | `CMD\|3\|OK\|$` |
| 4 | Ultrasonic sensor | `CMD\|4\|1\|$` | `CMD\|4\|<distance_mm>\|$` |
| 5 | IMU data | `CMD\|5\|$` | `CMD\|5\|<x_tilt>\|<y_tilt>\|$` |
| 6 | Battery level | `CMD\|6\|$` | `CMD\|6\|<millivolts>\|$` |

**Caveat:** The stock firmware's background thread (`start_main`) can conflict with direct REPL calls. `_dog.move()` works when no BLE/WiFi socket is connected to the firmware's listener. Avoid `set_gait_params()` and `homeostasis()` toggling — can break the default gait.

---

## WebSocket Protocol (Host ↔ Browser)

The web UI communicates with the Python host via WebSocket using JSON. This protocol is the same regardless of which firmware path is active — the host translates as needed.

```json
{"type": "cmd_move", "direction": "forward"}
{"type": "cmd_stand"}
{"type": "cmd_scan", "action": "start"}
{"type": "cmd_map", "action": "get"}
{"type": "telem_imu", "pitch": 2.1, "roll": -0.3}
{"type": "telem_sonar", "distance_mm": 250}
{"type": "telem_battery", "voltage_mv": 7400, "pct": 80}
{"type": "telem_status", "mode": "remote", "balance": true}
```

### Connection Management

- WebSocket ping/pong at 5s interval
- If no ping response in 5s, UI shows disconnected
- Auto-reconnect with backoff on WebSocket close
