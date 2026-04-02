# Bark-Buddy

Firmware for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Current Milestone: Thin MVP

Validate full architecture with minimal working versions of:
- **Remote Control** — Web UI → walk, turn, stop
- **Balance & Recovery** — IMU tilt correction, fall detection, stand-up
- **Patrol** — Execute predefined movement sequences autonomously

## Architecture

Two-process system connected via serial (→ WiFi later):

- **MechDog controller (C/C++):** Servo control, gait engine, IMU reading, balance layer
- **Local dev machine (Python):** Web server, behavior engine, comms layer
- **Browser:** Web UI for remote control and status

Flow: Browser → WebSocket → Python host → serial → C++ firmware → servos

## Key Design Decisions

- **Transport:** Serial first, WiFi later (abstracted transport layer)
- **Protocol:** Bidirectional tagged JSON messages
- **Gait:** Parametric / IK (build on MechDog's existing locomotion)
- **Balance:** Complementary filter + PID (always-on composable layer)
- **Behaviors:** Composable layers — balance runs beneath remote or patrol
- **Patrol (MVP):** Dead reckoning + IMU heading with waypoints
- **Connection loss:** Auto-reconnect with servo freeze
- **Web UI:** D-pad controls, 2D gauges, Python serves everything
- **Testing:** Mock serial stub for host dev, real hardware for firmware

See `docs/decisions.md` for full rationale.

## Project Layout

- `firmware/` — C/C++ for MechDog controller (PlatformIO)
  - `src/main.cpp` — entry point, serial comms, message routing
  - `src/gait.cpp` — parametric gait engine / IK
  - `src/balance.cpp` — complementary filter + PID balance
  - `src/servos.cpp` — servo abstraction
  - `include/protocol.h` — message types and structures
- `host/` — Python host application
  - `server.py` — web server + WebSocket
  - `comms.py` — abstract transport (serial/WiFi)
  - `mock_serial.py` — serial stub for dev without hardware
  - `behaviors/` — remote.py, balance.py, patrol.py
- `web/` — static web UI (index.html, app.js, style.css)
- `docs/` — architecture, decisions, implementation plan, protocol spec

## Conventions

- Firmware: C++17, PlatformIO build system
- Host: Python 3.11+, asyncio, websockets
- Protocol: Bidirectional tagged JSON over serial/WiFi (see docs/protocol.md)
- Web: Vanilla HTML/CSS/JS, no framework

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits, obstacle avoidance, SLAM.
