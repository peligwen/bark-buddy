# Communication Protocol

Bidirectional tagged JSON messages over serial (USB UART). WiFi (TCP) transport planned for later — same message format.

## Transport

- **Serial:** 115200 baud, 8N1
- **Framing:** One JSON object per line (newline-delimited JSON, aka NDJSON)
- **Max message size:** 512 bytes
- **Encoding:** UTF-8

## Message Format

Every message is a JSON object with a `type` field:

```json
{"type": "cmd_move", "direction": "forward", "speed": 0.5}
```

### Host → Firmware (Commands)

| Type | Fields | Description |
|------|--------|-------------|
| `cmd_move` | `direction`: forward/backward/left/right/stop, `speed`: 0.0-1.0 | Movement command |
| `cmd_stand` | — | Stand in neutral position |
| `cmd_set_gait` | `params`: {step_height, stride_length, speed} | Adjust gait parameters |
| `cmd_patrol` | `action`: start/stop, `waypoints`: [{x, y, heading}] | Start/stop patrol |
| `cmd_balance` | `enabled`: bool | Enable/disable balance layer |
| `ping` | — | Heartbeat / connection check |

### Firmware → Host (Telemetry & Responses)

| Type | Fields | Description |
|------|--------|-------------|
| `telem_imu` | `pitch`, `roll`, `yaw`, `ax`, `ay`, `az`, `gx`, `gy`, `gz` | IMU data (streamed at 20 Hz) |
| `telem_status` | `mode`, `battery_pct`, `servo_errors`: [] | Status report (streamed at 1 Hz) |
| `ack` | `ref_type`: string, `ok`: bool, `error`: string? | Command acknowledgment |
| `event_fall` | `pitch`, `roll` | Fall detected |
| `event_recovered` | — | Stand-up recovery complete |
| `pong` | — | Heartbeat response |

## Heartbeat

- Host sends `ping` every 2 seconds
- Firmware responds with `pong`
- If firmware receives no message for 5 seconds → freeze servos, enter safe mode
- If host receives no message for 5 seconds → attempt reconnect

## Connection Lifecycle

1. **Connect:** Host opens serial port, sends `ping`
2. **Ready:** Firmware responds with `pong` + initial `telem_status`
3. **Active:** Bidirectional message flow
4. **Lost:** Timeout detected → firmware freezes servos; host retries connection
5. **Recovered:** `ping`/`pong` succeeds → resume from last known state

## Examples

### Move forward at half speed
```
→ {"type": "cmd_move", "direction": "forward", "speed": 0.5}
← {"type": "ack", "ref_type": "cmd_move", "ok": true}
```

### IMU telemetry stream
```
← {"type": "telem_imu", "pitch": 2.1, "roll": -0.3, "yaw": 45.0, "ax": 0.01, "ay": -0.02, "az": 9.81, "gx": 0.0, "gy": 0.0, "gz": 0.0}
```

### Fall detection
```
← {"type": "event_fall", "pitch": 45.2, "roll": 3.1}
← {"type": "event_recovered"}
```

### Connection check
```
→ {"type": "ping"}
← {"type": "pong"}
```
