# Bark-Buddy

Firmware for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Current Milestone: Thin MVP

Validate full architecture with minimal working versions of:
- **Remote Control** — Web UI → walk, turn, stop
- **Balance & Recovery** — IMU tilt correction, fall detection, stand-up
- **Patrol** — Execute predefined movement sequences autonomously

## Architecture

Two-process system connected via serial/WiFi:

- **MechDog controller (C/C++):** Servo control, gait engine, IMU reading
- **Local dev machine (Python):** Web server, behavior engine, comms layer
- **Browser:** Web UI for remote control and status

Flow: Browser → WebSocket → Python host → serial/WiFi → C++ firmware → servos

## Project Layout

- `firmware/` — C/C++ for MechDog controller (PlatformIO)
  - `src/main.cpp` — entry point, comms
  - `src/gait.cpp` — walk, turn, stand
  - `src/balance.cpp` — IMU balancing
  - `src/servos.cpp` — servo abstraction
- `host/` — Python host application
  - `server.py` — web server + WebSocket
  - `comms.py` — serial/WiFi comms with dog
  - `behaviors/` — remote.py, balance.py, patrol.py
- `web/` — static web UI (index.html, app.js, style.css)
- `docs/` — architecture details, implementation plan, protocol spec

## Conventions

- Firmware: C++17, PlatformIO build system
- Host: Python 3.11+, asyncio, websockets
- Protocol: JSON messages over serial/WiFi (see docs/protocol.md when created)
- Web: Vanilla HTML/CSS/JS, no framework

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits, obstacle avoidance, SLAM.
