# Bark-Buddy

Control system for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Status

- **Milestone 1** (Remote, Balance, Patrol) — complete
- **Milestone 2** (Ultrasonic Scanning/Mapping) — complete
- **Milestone 3** (PyBullet Physics Simulation) — complete
- **Milestone 4** (Wall Mesh & 3D Visualization) — complete
- **Current work** — Custom firmware deployment, WiFi transport, hardware testing

## Architecture

Two firmware paths, same Python host and web UI:

- **Custom firmware (primary):** C++ on ESP32-S3, JSON/NDJSON over WiFi TCP (port 9000) or serial. Full servo control, gait engine, IMU/sonar streaming, heartbeat. This is the target.
- **Stock firmware (fallback):** MicroPython REPL over USB serial or WiFi WebREPL. Used for bootstrapping and when custom firmware isn't flashed.
- **Serial:** Available on both firmware paths for debugging and development.

Components:
- **MechDog:** ESP32-S3, 8 PWM servos, QMI8658 IMU, I2C ultrasonic
- **Local dev machine (Python):** Web server, behavior engine, transport layer
- **Browser:** Dark-themed control UI with 3D visualization

Flow (custom firmware): Browser → WebSocket (JSON) → Python host → JSON/NDJSON over WiFi TCP → custom firmware → servos
Flow (stock fallback): Browser → WebSocket (JSON) → Python host → REPL commands → stock firmware → servos

## Key Design Decisions

- **Firmware:** Custom C++ firmware over WiFi is the primary target. Stock MicroPython firmware is the fallback/bootstrap path.
- **Transport (primary):** WiFi TCP with JSON/NDJSON — custom firmware listens on port 9000, host connects as TCP client.
- **Transport (fallback):** USB serial REPL or WiFi WebREPL — for stock firmware, bootstrapping, and pin discovery.
- **Transport (debug):** USB serial available on both firmware paths for development and debugging.
- **Custom firmware API:** JSON messages — `cmd_move`, `cmd_stand`, `cmd_balance`, `cmd_servo`, `cmd_led`. Firmware streams telemetry (`telem_imu`, `telem_sonar`, `telem_battery`, `telem_status`).
- **Stock firmware API:** `_dog.move(speed, direction)` via HW_MechDog module. Don't modify gait params or homeostasis — stock defaults work.
- **Browser protocol:** WebSocket + JSON (same regardless of firmware path)
- **Behaviors:** Composable layers — balance runs beneath remote or patrol
- **Web UI:** Dark theme, D-pad controls, 3D dog view + scan map, vanilla JS (ES modules)
- **Testing:** Mock transport for dev, PyBullet sim, real hardware via WiFi/serial

**Stock firmware caveat:** The stock firmware runs a background thread (`start_main`) that polls BLE/WiFi for CMD protocol. Direct `_dog.move()` calls work when neither BLE nor WiFi socket is connected to the firmware's listener. Calling `set_gait_params()` or `homeostasis()` can break the default gait — avoid modifying firmware state beyond `move()`. This limitation is one reason custom firmware is preferred.

## Project Layout

- `firmware/` — Custom C++ firmware (PlatformIO, ESP32-S3)
  - `src/` — main.cpp, gait.cpp, imu.cpp, servos.cpp, sonar.cpp
  - `include/` — config.h, protocol.h, gait.h, imu.h, servos.h, sonar.h, poses.h
  - `test/` — kinematics, balance PID, gait, pose tests
  - `platformio.ini` — ESP32-S3 build config
- `host/` — Python host application
  - `server.py` — web server + WebSocket + telemetry loop
  - `comms.py` — CMD protocol layer + Transport ABC
  - `firmware_transport.py` — transport for custom firmware (JSON/NDJSON)
  - `hw_transport.py` — shared base for stock firmware transports (CMD→REPL)
  - `repl_transport.py` — USB serial REPL transport (stock firmware / debug)
  - `webrepl_transport.py` — WiFi WebREPL transport (stock firmware fallback)
  - `mock_serial.py` — mock transport for dev without hardware
  - `mock_firmware.py` — mock custom firmware for dev
  - `setup_wifi.py` — interactive WiFi + WebREPL setup script
  - `capture_profile.py` — profile capture + parameter optimizer
  - `behaviors/` — balance.py, patrol.py, scan.py, map_store.py, wall_fit.py, wall_mesh.py, octree.py
  - `sim/` — PyBullet simulation (sim_transport.py, mechdog.urdf)
- `web/` — static web UI (ES modules)
  - `index.html` — page structure
  - `style.css` — dark theme, responsive layout
  - `app.module.js` — main entry point
  - `modules/` — ws.js, controls.js, map.js, panels.js
  - `dog3d/` — Three.js 3D visualization (model, gait, camera, sonar, walls, overlay, state)
- `docs/` — architecture, decisions, implementation plan, protocol spec

## Conventions

- Firmware: C++ (PlatformIO), ArduinoJson, ESP32-S3
- Host: Python 3.11+, asyncio, pyserial-asyncio, websockets
- Web: Vanilla HTML/CSS/JS (ES modules), Three.js r128 via CDN
- Transport: `--wifi 192.168.1.163` (primary) or `--serial /dev/cu.usbserial-XXX` (debug) or mock (default)

## Custom Firmware Protocol (JSON/NDJSON)

Commands (host → firmware):
- `{"type": "cmd_move", "direction": "forward", "speed": 1.0}`
- `{"type": "cmd_stand"}`
- `{"type": "cmd_balance", "enabled": true}`
- `{"type": "cmd_servo", "index": 0, "pulse_us": 1500}`
- `{"type": "cmd_led", "led": 1, "r": 0, "g": 15, "b": 0}`
- `{"type": "ping"}`

Telemetry (firmware → host):
- `{"type": "telem_imu", "pitch": 2.1, "roll": -0.3, "yaw": 45.0, ...}`
- `{"type": "telem_sonar", "distance_mm": 250}`
- `{"type": "telem_battery", "voltage_mv": 7400, "pct": 80, "low": false}`
- `{"type": "telem_status", "mode": "idle", "balance": true, "servos": true}`
- `{"type": "ack", "ref_type": "cmd_move", "ok": true}`

## Stock Firmware API (via REPL — fallback)

- Move: `_dog.move(speed, direction)` — speed (neg=back), direction (neg=left, pos=right)
- Stop: `_dog.move(0, 0)`
- IMU: `_imu.read_angle()` → `[pitch, roll]`
- Sonar: `_sonar.getDistance()` → centimeters (float)
- Actions: `_dog.action_run(code)` — 1=wave, 4=sit, 5=lie down
- Balance: `_dog.homeostasis(0|1)` — avoid toggling, can break gait

## Workflow Guidelines

- Commit early and often. Small, focused commits.
- Ask before major pivots.
- Custom firmware: test with `pio test` before flashing. Servo pins must be verified before enabling `PINS_VERIFIED`.
- Stock firmware: don't modify gait/balance state — use move() only.

## Out of Scope

Camera/vision, object carrying, runtime AI, mobile app, Pi integration.
