# Architecture

## Components

```
Browser (vanilla JS + Three.js)
    │ WebSocket JSON
    ▼
Python host (aiohttp)
    ├── server.py             — WS session, command routing, telemetry relay, static files
    ├── dog/                  — transport package (Dog class, io, discover)
    ├── camera.py             — pulls JPEG stills from the vision module
    ├── ota.py                — firmware OTA
    ├── lock.py               — WS control lock
    ├── behaviors/
    │   ├── balance.py        — active pitch/roll compensation via cmd_transform
    │   ├── button_engage.py  — K1 button → engage/disengage
    │   └── capture.py        — stop-and-snap keyframe recorder
    └── perception/           — offline reconstruction of a capture run (GPU)
        ├── da3.py            — Depth Anything 3: depth + camera poses per keyframe
        ├── geometry.py       — floor fit, metric scale, floor-aligned frame, occupancy map
        ├── checks.py         — reconstruction vs sonar, gyro and commanded motion
        └── reconstruct.py    — pipeline and outputs (`bark reconstruct`)
    │ NDJSON over TCP (WiFi or USB serial)          │ HTTP GET (WiFi)
    ▼                                               ▼
Firmware (C++ / ESP32-WROOM-32D)                Vision module (C++ / ESP32-S3)
    ├── command_handlers.cpp  — JSON dispatch       └── camera/src/main.cpp — JPEG still server
    ├── gait.cpp, servos.cpp  — IK gait engine, servo control
    ├── imu.cpp, sonar.cpp    — sensor drivers
    └── sensor_task.cpp       — FreeRTOS sensor loop (50 Hz IMU, 20 Hz sonar)
```

The vision module is a separate WiFi peer with its own firmware and no link to the
main controller. The host only reads from it.

**Mock firmware** (`firmware/test/bark-mock`) — the same C++ source compiled as a native binary
with platform shims in `firmware/mock/`. The host connects to it identically to real hardware
via TCP on localhost. This is the primary dev loop.

## Layer Responsibilities

| Layer | Owns | Does NOT own |
|-------|------|--------------|
| Firmware | Motion, hardware I/O, sensor streaming, command execution | Coordination, session state |
| Vision module | Exposing the sensor: one fresh JPEG per request | Any interpretation of the image |
| Host | Behavior composition, WebSocket session, telemetry relay, perception | Servo math, sensor fusion |
| Web | Presentation, user input, 3D visualization | Business logic, transport |

No layer reaches past its neighbor. The browser never speaks TCP; firmware never speaks WebSocket.

## Data Flow

**Command path (browser → firmware):**
1. Browser sends JSON over WebSocket to `server.py`
2. `server.py` routes to a behavior or calls `Dog.send_json()`
3. Firmware receives NDJSON, dispatches via handler table, actuates servos

**Telemetry path (firmware → browser):**
1. Firmware sensor task streams NDJSON (IMU 50 Hz, sonar 20 Hz, status 1 Hz, battery 1 Hz)
2. `Dog` parses and fires Python callbacks
3. `server.py` relays to all connected WebSocket clients as JSON

**Capture path (camera + firmware → disk):**
1. With `--cam-url` and `--record`, `CaptureRecorder` watches accepted motion commands and telemetry
2. Once the dog has stood still for the settle time it pulls one still through `Camera.grab()`
3. The keyframe is saved with the IMU, sonar and joint readings and the moves since the last keyframe
4. `bark reconstruct` later turns the run into a metric map and a consistency report

Keyframes are taken only at rest. Stills from a standing dog are sharp over a slow
camera link, and the reconstruction model registers widely spaced views, so there is
no video stream anywhere in the system.

## Transport Boundary

`Dog.send_json(msg: dict)` is the **only** host→firmware command path.
No text protocols, no direct serial writes, no alternate transports.
`Camera.grab()` is the only path to the vision module, and it is read-only.

Auto-detection order on startup: USB serial JSON ping → mDNS `_mechdog._tcp` → exit with guidance.
Override: `--fw-tcp HOST[:PORT]`.

## Composable Behavior Pattern

Behaviors take a `Dog` instance and run as asyncio tasks alongside the main
WebSocket loop. They register telemetry callbacks and send commands through the transport.

```python
self._balance = BalanceLayer(transport)           # reads IMU, writes cmd_transform
self._button_engage = ButtonEngageBehavior(...)   # reads K1 button, writes cmd_engage
self._recorder = CaptureRecorder(get_transport, camera, root)  # reads telemetry + camera, writes a run
```

When the transport is replaced (e.g., after OTA flash), `server.py` re-creates the behaviors
bound to it via `_on_transport_replaced()`. `CaptureRecorder` takes a getter instead, so one
run survives a transport swap.

## Mock Firmware Internals

`firmware/mock/` contains host-friendly replacements for Arduino/ESP-IDF headers, resolved via
include-path prepend. IMU and sonar drivers are replaced at link time:

- `imu_mock.cpp` — reads from physics model (`physics.cpp`)
- `sonar_mock.cpp` — reads from physics model
- `physics.cpp` — servo duty → FK via `pulses_to_foot()` → pitch/roll from foot heights → low-pass filter → gyro derivative → gravity projection

`freertos_shim.h` provides the mutex, task and queue primitives the shared sources use.

The mock has no world: the body does not translate or turn, the sonar reads a constant,
and there is no simulated camera. Host tests stand in for the vision module with a local
HTTP server; a rendered camera arrives with the simulated world (see overview.md roadmap).
