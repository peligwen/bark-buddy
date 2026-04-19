# Architecture

## Components

```
Browser (vanilla JS + Three.js)
    │ WebSocket JSON
    ▼
Python host (aiohttp)
    ├── server.py             — WS session, command routing, telemetry relay, static files
    ├── firmware_transport.py — sole transport to firmware
    └── behaviors/
        ├── balance.py        — active pitch/roll compensation via cmd_transform
        └── button_engage.py  — K1 button → engage/disengage
    │ NDJSON over TCP (WiFi or USB serial)
    ▼
Firmware (C++ / ESP32-WROOM-32D)
    ├── command_handlers.cpp  — JSON dispatch table
    ├── gait.cpp, servos.cpp  — IK gait engine, servo control
    ├── imu.cpp, sonar.cpp    — sensor drivers
    └── sensor_task.cpp       — FreeRTOS sensor loop (50 Hz IMU, 20 Hz sonar)
```

**Mock firmware** (`firmware/test/bark-mock`) — the same C++ source compiled as a native binary
with platform shims in `firmware/mock/`. The host connects to it identically to real hardware
via TCP on localhost. This is the primary dev loop.

## Layer Responsibilities

| Layer | Owns | Does NOT own |
|-------|------|--------------|
| Firmware | Motion, hardware I/O, sensor streaming, command execution | Coordination, session state |
| Host | Behavior composition, WebSocket session, telemetry relay | Servo math, sensor fusion |
| Web | Presentation, user input, 3D visualization | Business logic, transport |

No layer reaches past its neighbor. The browser never speaks TCP; firmware never speaks WebSocket.

## Data Flow

**Command path (browser → firmware):**
1. Browser sends JSON over WebSocket to `server.py`
2. `server.py` routes to a behavior or calls `FirmwareTransport.send_json()`
3. Firmware receives NDJSON, dispatches via handler table, actuates servos

**Telemetry path (firmware → browser):**
1. Firmware sensor task streams NDJSON (IMU 50 Hz, sonar 20 Hz, status 1 Hz, battery 1 Hz)
2. `FirmwareTransport` parses and fires Python callbacks
3. `server.py` relays to all connected WebSocket clients as JSON

## Transport Boundary

`FirmwareTransport.send_json(msg: dict)` is the **only** host→firmware command path.
No text protocols, no direct serial writes, no alternate transports.

Auto-detection order on startup: USB serial JSON ping → mDNS `_mechdog._tcp` → exit with guidance.
Override: `--fw-tcp HOST[:PORT]`.

## Composable Behavior Pattern

Behaviors take a `FirmwareTransport` instance and run as asyncio tasks alongside the main
WebSocket loop. They register telemetry callbacks and send commands through the transport.

```python
self._balance = BalanceLayer(transport)           # reads IMU, writes cmd_transform
self._button_engage = ButtonEngageBehavior(...)   # reads K1 button, writes cmd_engage
```

When the transport is replaced (e.g., after OTA flash), `server.py` re-creates all behaviors
against the new transport via `_replace_transport()`.

## Mock Firmware Internals

`firmware/mock/` contains host-friendly replacements for Arduino/ESP-IDF headers, resolved via
include-path prepend. IMU and sonar drivers are replaced at link time:

- `imu_mock.cpp` — reads from physics model (`physics.cpp`)
- `sonar_mock.cpp` — reads from physics model
- `physics.cpp` — servo duty → FK via `pulses_to_foot()` → pitch/roll from foot heights → low-pass filter → gyro derivative → gravity projection
