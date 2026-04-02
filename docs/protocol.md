# Communication Protocol

The MechDog ships with stock firmware on an ESP32-S3 that speaks a text-based CMD protocol over UART serial. We use it directly from the Python host — no custom firmware needed for MVP.

## Transport

- **Serial:** 115200 baud, 8N1 (UART pins IO32 TX / IO33 RX)
- **USB:** Connect via USB adapter to dev machine
- **Encoding:** ASCII text

## Stock CMD Protocol

Format: `CMD|<function_code>|<sub_function>|<data>|$`

Responses follow the same `CMD|...|$` format.

### Function Codes

| Code | Function | Send Example | Response |
|------|----------|-------------|----------|
| 1 | Posture adjustment | `CMD\|1\|3\|1\|$` (enable self-balance) | `CMD\|1\|OK\|$` |
| 2 | Action groups | `CMD\|2\|1\|4\|$` (sit down) | `CMD\|2\|OK\|$` |
| 3 | Motion control | `CMD\|3\|3\|$` (forward) | `CMD\|3\|OK\|$` |
| 4 | Ultrasonic sensor | `CMD\|4\|1\|$` | `CMD\|4\|<distance_mm>\|$` |
| 5 | IMU data | `CMD\|5\|$` | `CMD\|5\|<x_tilt>\|<y_tilt>\|$` |
| 6 | Battery level | `CMD\|6\|$` | `CMD\|6\|<millivolts>\|$` |

### Motion Control (Function 3)

| Sub-code | Action |
|----------|--------|
| 1 | Stop |
| 2 | Stand |
| 3 | Forward |
| 4 | Backward |
| 5 | Turn left |
| 6 | Turn right |
| 7 | Shift left |
| 8 | Shift right |

### Posture Adjustment (Function 1)

| Sub-code | Data | Action |
|----------|------|--------|
| 1 | angle | Pitch adjustment |
| 2 | angle | Roll adjustment |
| 3 | 0/1 | Self-balance off/on |
| 4 | height | Body height adjustment |

### Action Groups (Function 2)

| Sub-code | Data | Action |
|----------|------|--------|
| 1 | 1 | Wave |
| 1 | 2 | Stretch |
| 1 | 3 | Turn around |
| 1 | 4 | Sit down |
| 1 | 5 | Lie down |

## Python Host Protocol Layer

The Python host wraps these CMD strings in a high-level API:

```python
await dog.move_forward()      # sends CMD|3|3|$
await dog.stop()              # sends CMD|3|1|$
await dog.enable_balance()    # sends CMD|1|3|1|$
imu = await dog.read_imu()   # sends CMD|5|$, parses response
battery = await dog.read_battery()  # sends CMD|6|$, parses response
```

## WebSocket Protocol (Host ↔ Browser)

The web UI communicates with the Python host via WebSocket using JSON:

```json
{"type": "cmd_move", "direction": "forward"}
{"type": "telem_imu", "pitch": 2.1, "roll": -0.3}
{"type": "telem_status", "mode": "remote", "battery_mv": 7400}
```

This keeps the browser-side protocol clean and decoupled from the hardware CMD format.

## Connection Management

- Host polls IMU (`CMD|5|$`) and battery (`CMD|6|$`) at regular intervals
- If serial read times out (500ms), mark connection as lost
- On connection loss: retry serial open with backoff
- WebSocket: if no host↔browser ping in 5s, UI shows disconnected
