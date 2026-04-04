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
| `cmd_balance` | `{"type":"cmd_balance","enabled":true}` | Enable/disable balance flag (not yet applied to servo output) |
| `cmd_servo` | `{"type":"cmd_servo","index":0,"pulse_us":1500}` | Direct servo control (requires `PINS_VERIFIED`) |
| `cmd_led` | `{"type":"cmd_led","led":1,"r":0,"g":15,"b":0}` | Set sonar module LED color |
| `ping` | `{"type":"ping"}` | Heartbeat ping |

### Telemetry (Firmware → Host)

| Type | Fields | Rate |
|------|--------|------|
| `telem_imu` | `pitch`, `roll`, `yaw`, `ax`, `ay`, `az`, `gx`, `gy`, `gz` | 50 Hz |
| `telem_sonar` | `distance_mm` | 20 Hz |
| `telem_battery` | `voltage_mv`, `pct`, `low` | 1 Hz |
| `telem_status` | `mode`, `balance`, `servos`, `low_battery`, `wifi`, `wifi_ip`, `tcp_port` | 1 Hz |
| `ack` | `ref_type`, `ok`, `error` (optional) | Per command |
| `boot` | `imu`, `sonar`, `servos`, `pins_verified` | Once on startup |
| `pong` | — | Per ping |

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

### Connection Management

- WebSocket reconnects automatically on close (fixed 2s interval)
- On connect, client sends `cmd_identify` then `cmd_map get` to get initial state
- Server broadcasts `version` on connect; client reloads if hash changed

### Control Lock

Only one browser client can control the dog at a time. Lock is auto-acquired on the first control command and times out after 30s of inactivity.

**Browser → Host:**

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_identify` | `name` | Register client name for lock display |
| `cmd_lock` | `name` | Request control. If held by another, sends `lock_challenge` to holder and `lock_denied` to requester |
| `cmd_unlock` | — | Release control lock |
| `cmd_lock_yield` | — | Current holder voluntarily yields to challenger |

**Host → Browser:**

| Type | Fields | Description |
|------|--------|-------------|
| `lock_status` | `locked`, `holder`, `is_mine` | Broadcast when lock changes |
| `lock_challenge` | `challenger` | Sent to current holder when someone else requests control |
| `lock_denied` | `holder` | Sent to requester when lock is held |

### Control Commands (require lock)

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_move` | `direction` (forward/backward/left/right/stop) | Move the dog |
| `cmd_stand` | — | Return to standing pose |
| `cmd_balance` | `enabled` (bool) | Toggle balance layer on host |
| `cmd_action` | `action` (int, 1=wave, 4=sit, 5=lie down) | Run action group |
| `cmd_pose` | `pose` (stand/sit/lie_down/etc.) | Set named pose |
| `cmd_patrol` | `action` (start/stop), `waypoints` (list of `{x,y,heading}`) | Waypoint patrol |
| `cmd_scan` | `action` (start/stop) | 360° ultrasonic scan |

### Utility Commands (no lock required)

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_set_default_pose` | `pose` | Set the idle rest pose name (client-side preference) |
| `cmd_map` | `action` (get/clear) | Get full map data or clear accumulated points |
| `cmd_transport` | `mode` (sim/usb/usb-fw/wifi/wifi-fw/hybrid), `wifi_host` (optional) | Switch transport mode |
| `cmd_wifi_setup` | `ssid`, `password` | Configure WiFi on the ESP32 via REPL |
| `cmd_sim_noise` | `params` (dict) | Configure simulation noise/latency parameters |
| `cmd_reset` | — | Reset dead reckoning, broadcast new web hash |
| `cmd_restart_server` | — | Hot-reload the Python host process |

### Telemetry (Host → Browser)

| Type | Fields | Description |
|------|--------|-------------|
| `telem_imu` | `pitch`, `roll`, `yaw` | Attitude from IMU (degrees) |
| `telem_ultrasonic` | `distance_mm` | Sonar distance |
| `telem_battery` | `voltage_mv`, `pct`, `low` | Battery state |
| `telem_status` | `mode`, `balance`, `motion`, `transport`, `battery_mv`, etc. | Full system status |
| `telem_odometry` | `x`, `y`, `heading`, `motion` | Dead-reckoned position |
| `balance_state` | `enabled` | Balance layer toggle result |

### Events (Host → Browser)

| Type | Fields | Description |
|------|--------|-------------|
| `event_fall` | — | Fall detected (tilt > 35°) |
| `event_recovered` | — | Recovery from fall completed |
| `scan_point` | `x`, `y`, `distance_mm` | Live point added during scan or continuous mapping |
| `scan_complete` | — | 360° scan finished |
| `map_data` | `bounds`, `points`, `walls`, `chains`, `scans`, `scan_count`, `point_count` | Full map snapshot (sent on request and every 1s) |
| `version` | `hash` | Web file hash (client reloads if changed) |
| `reset` | — | Dead reckoning reset |
| `transport_result` | `ok`, `mode`, `error` (optional) | Result of `cmd_transport` |
| `wifi_setup_result` | `ok`, `ip` (on success), `error` (on failure) | Result of `cmd_wifi_setup` |
