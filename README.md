# bark-buddy

Give our Hiwonder MechDog Open Source AI Robot Dog some real brains.

## What Is This?

Custom firmware and host software for the [Hiwonder MechDog](https://www.hiwonder.com/) robot dog. The goal is a semi-autonomous robot: custom C++ firmware on the ESP32-S (D0WD) communicates over WiFi with a Python host that runs behaviors, serves a web UI, and maintains a world model from ultrasonic scanning. The dog navigates to user-defined goals using that world model.

The stock MicroPython firmware is supported as a fallback for bootstrapping and when the custom firmware isn't flashed. USB serial is available for debugging on both paths.

## Completed Milestones

| Milestone | Capabilities |
|---|---|
| 1 — Remote, Balance, Scan | Web UI D-pad control, IMU-based balance, ultrasonic scan, telemetry dashboard |
| 2 — Ultrasonic Mapping | 360° scanning, point cloud accumulation, wall detection (chain-based + DBSCAN/PCA), wall mesh generation with corner snapping |
| 3 — Physics Simulation | Pure-Python physics engine (rigid body, ground contact, leg kinematics), simulated IMU/sonar, no external dependencies |
| 4 — Wall Mesh & Visualization | Three.js 3D dog model, gait animation, wall rendering, ES module UI refactor |

## Current Work

- IK-based gait pipeline (foot-position IK, body transforms, active balance, stride config)
- Firmware Foundation Refactor complete (FreeRTOS sensor task, command dispatch, WiFi reconnect)

## Up Next

- SLAM-based localization (IMU + sonar scan matching)
- Composite multi-scan mapping
- Waypoint navigation UI with path planning and obstacle avoidance

## Architecture

```
                              ┌─ WiFi TCP (JSON/NDJSON) ─→  Custom Firmware (primary)
Browser (Web UI)  ←WS/JSON→  Python Host (Dev PC) ─┤
                              └─ Serial REPL (debug/fallback) ─→  Stock Firmware
```

See [docs/architecture.md](docs/architecture.md) for details.

## Getting Started

> **Prerequisites:** Hiwonder MechDog, Python 3.11+, PlatformIO (for custom firmware)

```bash
pip install -r host/requirements.txt

# WiFi (custom firmware — primary)
python host/server.py --wifi 192.168.1.163

# USB serial (debugging / stock firmware fallback)
python host/server.py --serial /dev/ttyUSB0

# Pure-Python simulation (no hardware needed, no extra dependencies)
python host/server.py --sim

# Sim mode (default, no flags needed)
python host/server.py
```

Open `http://localhost:8080` in your browser.

### Flashing Custom Firmware

```bash
cd firmware
pio test          # run tests first
pio run -t upload # flash to ESP32-S (D0WD)
```

## Documentation

- [Architecture & Tech Stack](docs/architecture.md)
- [Design Decisions](docs/decisions.md)
- [Implementation Plan](docs/implementation-plan.md)
- [Communication Protocol](docs/protocol.md)
