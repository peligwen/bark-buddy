# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Status

- **Milestone 1** (Remote, Balance, Scanning) — complete
- **Milestone 2** (Ultrasonic Scanning/Mapping) — complete
- **Milestone 3** (Pure-Python Physics Simulation) — complete
- **Milestone 4** (Wall Mesh & 3D Visualization) — complete
- **Firmware Foundation Refactor** (FreeRTOS sensor task, command handler dispatch, WiFi reconnect) — complete
- **Custom-Firmware-Only Refactor** (deleted stock/hybrid paths, collapsed CMD protocol, replaced Python sim with Mock Firmware) — complete
- **Engage Switch Refactor** (master engage/disengage, deleted lifecycle FSM, cmd_engage) — complete
- **Current work** — IK-based gait pipeline (foot-position IK, body transforms, active balance, stride config)
- **Next milestone** — SLAM-based localization, composite multi-scan mapping, waypoint navigation UI

## Architecture

Single firmware path:

- **Custom firmware:** C++ on ESP32-WROOM-32D (D0WD chip), JSON/NDJSON over WiFi TCP (port 9000) or USB serial. Full servo control, IK gait engine, FreeRTOS sensor task, command handler dispatch table, IMU/sonar streaming, heartbeat.
- **Mock Firmware:** same C++ source compiled as a native binary (`firmware/test/bark-mock`) with thin platform shims. Launched via `bark mock`. Host connects via `FirmwareTransport` over local TCP — identical to real hardware path.

Components:
- **MechDog:** ESP32-D0WD, 8 PWM servos, QMI8658 IMU, I2C ultrasonic
- **Local dev machine (Python):** Web server, behavior engine, `FirmwareTransport`
- **Browser:** Dark-themed control UI with 3D visualization

Flow: Browser → WebSocket (JSON) → Python host → JSON/NDJSON (USB serial or WiFi TCP or local TCP) → firmware (real or mock) → servos

## Key Design Decisions

- **Mock Firmware:** native C++ binary compiled from real firmware source with host shims (`firmware/mock/`). Include-path substitution — `firmware/mock/` prepended so `<Arduino.h>`, `<WiFi.h>`, etc. resolve to host-friendly stubs. Driver-level link substitution for IMU/sonar (reads from physics model). Physics: servo duty → FK via `pulses_to_foot()` → pitch/roll from foot heights → low-pass → gyro derivative → gravity projection.
- **Single transport:** `FirmwareTransport` is the only transport class. `send_json(msg: dict)` is the sole command path — no CMD text protocol.
- **Auto-detect hardware:** USB serial JSON ping → mDNS `_mechdog._tcp` → exit with guidance. No silent fallback.
- **`bark mock`:** spawns `firmware/test/bark-mock --tcp-port 9001`, waits for port, starts server with `--fw-tcp 127.0.0.1:9001`.
- **Firmware API:** JSON messages — `cmd_move`, `cmd_stand`, `cmd_balance`, `cmd_servo`, `cmd_led`, `cmd_transform`, `cmd_gait_params`, `cmd_engage`, `cmd_offset`. Firmware streams telemetry (`telem_imu`, `telem_sonar`, `telem_battery`, `telem_status`).
- **Browser protocol:** WebSocket + JSON
- **Behaviors:** Composable layers — `BalanceLayer` and `ScanBehavior` take `Transport` directly.
- **Web UI:** Dark theme, D-pad controls, 3D dog view + scan map, vanilla JS (ES modules)
- **Goal:** Semi-autonomous — user sets goals from the UI, dog navigates using a continuously updated world model (sonar + future camera)

## Project Layout

- `firmware/` — Custom C++ firmware (PlatformIO, ESP32)
  - `src/` — main.cpp, gait.cpp, imu.cpp, servos.cpp, sonar.cpp, balance.cpp, command_handlers.cpp, offsets.cpp, sensor_task.cpp
  - `include/` — config.h, protocol.h, gait.h, imu.h, servos.h, sonar.h, balance.h, body_transform.h, cf_filter.h, command_handlers.h, comms.h, gait_math.h, ik.h, offsets.h, sensor_task.h, update_led.h
  - `mock/` — platform shims + physics model for native mock build
    - `Arduino.h`, `WiFi.h`, `ESPmDNS.h`, `Wire.h`, `Preferences.h`, `esp_compat.h`, `freertos_shim.h`, `HTTPClient.h`, `Update.h` — host shims
    - `physics.cpp` / `physics.h` — servo duty → IMU/sonar physics model
    - `net_tcp.cpp` / `net_tcp.h` — BSD sockets TCP server
    - `mock_main.cpp` — host entry point; resends boot on new TCP client
    - `mock_sensor_task.cpp` — std::thread sensor workers (50 Hz IMU, 10 Hz sonar)
    - `imu_mock.cpp`, `sonar_mock.cpp` — link-time replacements for IMU/sonar drivers
    - `mock_globals.cpp` — singleton definitions (Wire, WiFi, MDNS, ESP, Update)
  - `test/` — IK, transform, balance, offset, gait, servo unit tests + Makefile
    - `make bark-mock` — builds the native mock firmware binary
  - `platformio.ini` — ESP32 build config
- `host/` — Python host application
  - `server.py` — web server + WebSocket + telemetry loop
  - `comms.py` — Transport ABC + constants (DIRECTIONS, SERIAL_BAUD)
  - `firmware_transport.py` — sole transport (USB serial or TCP); `DeadReckoningMixin`
  - `dead_reckoning.py` — odometry mixin; `record_motion(direction)` for server to call
  - `capture_pose.py` — servo offset capture tool
  - `behaviors/` — balance.py, scan.py, map_store.py, wall_fit.py, wall_mesh.py, octree.py
- `web/` — static web UI (ES modules)
  - `index.html` — page structure
  - `style.css` — dark theme, responsive layout
  - `app.module.js` — main entry point
  - `modules/` — ws.js, controls.js, map.js, panels.js
  - `dog3d/` — Three.js 3D visualization (model, gait, camera, sonar, walls, overlay, state)
- `docs/` — architecture, decisions, implementation plan, protocol spec, superpowers/

## Conventions

- Firmware: C++ (PlatformIO), ArduinoJson, ESP32-WROOM-32D (D0WD chip)
- Mock firmware build: `cd firmware/test && make bark-mock` (clang++, C++17, MOCK_FIRMWARE=1)
- Host: Python 3.11+, asyncio, pyserial-asyncio, websockets
- Web: Vanilla HTML/CSS/JS (ES modules), Three.js r128 via CDN
- Transport: auto-detected (USB serial → hardware, mDNS → WiFi hardware). Override: `--fw-tcp HOST[:PORT]`.

## CLI

- `bark` — auto-detect hardware (USB → mDNS → exit with guidance)
- `bark mock` — build + launch mock firmware, connect server to it
- `bark flash` — flash firmware via PlatformIO
- `bark test` — run firmware unit tests
- `bark kill` — kill any running bark server

## Custom Firmware Protocol (JSON/NDJSON)

Commands (host → firmware):
- `{"type": "cmd_move", "direction": "forward", "speed": 1.0}`
- `{"type": "cmd_stand"}`
- `{"type": "cmd_balance", "enabled": true}`
- `{"type": "cmd_servo", "index": 0, "pulse_us": 1500}`
- `{"type": "cmd_led", "led": 1, "r": 0, "g": 15, "b": 0}`
- `{"type": "cmd_transform", "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}`
- `{"type": "cmd_gait_params", "stride_length": 20, "stride_height": 10, "speed": 1.0}`
- `{"type": "cmd_engage", "enabled": true}`
- `{"type": "cmd_offset", "index": 0, "offset_us": 0}`
- `{"type": "ping"}`

Telemetry (firmware → host):
- `{"type": "telem_imu", "pitch": 2.1, "roll": -0.3, "yaw": 45.0, ...}`
- `{"type": "telem_sonar", "distance_mm": 250}`
- `{"type": "telem_battery", "voltage_mv": 7400, "pct": 80, "low": false}`
- `{"type": "telem_status", "engaged": true, "ramping": false, "balance": true, "battery_cutoff": false}`
- `{"type": "ack", "ref_type": "cmd_move", "ok": true}`

## Workflow Guidelines

- Commit early and often. Small, focused commits.
- Ask before major pivots.
- Custom firmware: test with `pio test` before flashing. Servo pins must be verified before enabling `PINS_VERIFIED`.
- Mock firmware: `cd firmware/test && make bark-mock` to build; `./bark-mock --tcp-port 9001` to run standalone.

## Planned Future Work

- **SLAM-based localization** — IMU + sonar scan matching for proper position estimation (replaces dead reckoning)
- **Composite multi-scan mapping** — merge overlapping scans into a persistent world model
- **Waypoint navigation UI** — user places waypoints on 2D map; path planning + obstacle avoidance handles routing
- **Camera/vision** — ESP32-S3 camera module integration; visual data layered on sonar world model
- **Semantic understanding** — room labeling, local LLM for natural language goals ("go check the living room")
- **Multi-platform support** — abstract robot config to support other quadrupeds and custom builds

## Out of Scope

Object carrying, mobile app, Pi integration.
