# Bark-Buddy

Control system for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Status

- **Milestone 1** (Remote, Balance, Patrol) — complete
- **Milestone 2** (Ultrasonic Scanning/Mapping) — complete
- **Milestone 3** (PyBullet Physics Simulation) — complete
- **Current work** — Hardware integration, WiFi transport, UI refinement

## Architecture

Python host talks to MechDog's MicroPython REPL over USB serial or WiFi WebREPL:

- **MechDog (stock firmware):** ESP32-S3, 8 PWM servos, QMI8658 IMU, I2C ultrasonic
- **Local dev machine (Python):** Web server, behavior engine, transport layer
- **Browser:** Dark-themed control UI with 3D visualization

Flow: Browser → WebSocket (JSON) → Python host → REPL commands → stock firmware → servos

## Key Design Decisions

- **Firmware:** Stock MicroPython firmware (no custom firmware)
- **Transport:** USB serial REPL or WiFi WebREPL — both translate CMD protocol to Python calls
- **Dog API:** `_dog.move(speed, direction)` via HW_MechDog module. Don't modify gait params or homeostasis — stock defaults work.
- **Browser protocol:** WebSocket + JSON (decoupled from CMD format)
- **Behaviors:** Composable layers — balance runs beneath remote or patrol
- **Web UI:** Dark theme, D-pad controls, 3D dog view + scan map, vanilla JS
- **Testing:** Mock transport for dev, real hardware via USB/WiFi

**Critical hardware note:** The stock firmware runs a background thread (`start_main`) that polls BLE/WiFi for CMD protocol and controls movement. Direct `_dog.move()` calls work when neither BLE nor WiFi socket is connected to the firmware's listener. Calling `set_gait_params()` or `homeostasis()` can break the default gait — avoid modifying firmware state beyond `move()`.

## Project Layout

- `host/` — Python host application
  - `server.py` — web server + WebSocket + telemetry loop
  - `comms.py` — CMD protocol layer + Transport ABC
  - `hw_transport.py` — shared base for hardware transports (CMD→REPL translation)
  - `repl_transport.py` — USB serial REPL transport
  - `webrepl_transport.py` — WiFi WebREPL transport
  - `mock_serial.py` — mock transport for dev without hardware
  - `setup_wifi.py` — interactive WiFi + WebREPL setup script
  - `behaviors/` — balance.py, patrol.py, scan.py, map_store.py
  - `sim/` — PyBullet physics simulation transport
- `web/` — static web UI
  - `index.html` — page structure
  - `style.css` — dark theme, responsive layout
  - `app.js` — WebSocket client, controls, 2D map
  - `dog3d.js` — Three.js 3D visualization
- `docs/` — architecture, decisions, implementation plan, protocol spec

## Conventions

- Host: Python 3.11+, asyncio, pyserial-asyncio, websockets
- Web: Vanilla HTML/CSS/JS, Three.js r128 via CDN
- Transport: `--serial /dev/cu.usbserial-XXX` or `--wifi 192.168.1.163` or mock (default)

## MechDog Hardware API (via REPL)

- Move: `_dog.move(speed, direction)` — speed (neg=back), direction (neg=left, pos=right)
- Stop: `_dog.move(0, 0)`
- IMU: `_imu.read_angle()` → `[pitch, roll]`
- Sonar: `_sonar.getDistance()` → centimeters (float)
- Actions: `_dog.action_run(code)` — 1=wave, 4=sit, 5=lie down
- Balance: `_dog.homeostasis(0|1)` — avoid toggling, can break gait

## Workflow Guidelines

- Commit early and often. Small, focused commits.
- Ask before major pivots.
- Don't modify firmware gait/balance state — use move() only.

## Out of Scope

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, custom firmware.
