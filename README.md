# bark-buddy

Give our Hiwonder MechDog Open Source AI Robot Dog some real brains.

## What Is This?

Custom firmware and host software for the [Hiwonder MechDog](https://www.hiwonder.com/) robot dog. The goal is a custom C++ firmware on the ESP32-S3 communicating over WiFi with a Python host that runs behaviors, serves a web UI, and provides 3D visualization and ultrasonic mapping.

The stock MicroPython firmware is supported as a fallback for bootstrapping and when the custom firmware isn't flashed. USB serial is available for debugging on both paths.

## Completed Milestones

| Milestone | Capabilities |
|---|---|
| 1 — Remote, Balance, Patrol | Web UI D-pad control, IMU-based balance, waypoint patrol, telemetry dashboard |
| 2 — Ultrasonic Mapping | 360° scanning, point cloud accumulation, wall detection (chain-based + DBSCAN/PCA), wall mesh generation with corner snapping |
| 3 — Physics Simulation | PyBullet SimTransport with URDF model, simulated IMU/sonar, room creation for mapping tests |
| 4 — Wall Mesh & Visualization | Three.js 3D dog model, gait animation, wall rendering, ES module UI refactor |

## Current Work

- Custom firmware deployment and WiFi transport integration
- Servo pin verification and hardware testing
- UI refinement and wall rendering fidelity

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

# PyBullet simulation (no hardware needed)
python host/server.py --sim

# Mock mode (no hardware, no sim)
python host/server.py
```

Open `http://localhost:8080` in your browser.

### Flashing Custom Firmware

```bash
cd firmware
pio test          # run tests first
pio run -t upload # flash to ESP32-S3
```

## Documentation

- [Architecture & Tech Stack](docs/architecture.md)
- [Design Decisions](docs/decisions.md)
- [Implementation Plan](docs/implementation-plan.md)
- [Communication Protocol](docs/protocol.md)
