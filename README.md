# bark-buddy

Give our Hiwonder MechDog Open Source AI Robot Dog some real brains.

## What Is This?

Host software and web UI for the [Hiwonder MechDog](https://www.hiwonder.com/) robot dog. Python runs on a local dev machine talking to the MechDog's stock MicroPython firmware over USB serial or WiFi WebREPL. A browser-based control UI provides remote control, telemetry, 3D visualization, and ultrasonic mapping.

## Completed Milestones

| Milestone | Capabilities |
|---|---|
| 1 — Remote, Balance, Patrol | Web UI D-pad control, IMU-based balance, waypoint patrol, telemetry dashboard |
| 2 — Ultrasonic Mapping | 360° scanning, point cloud accumulation, wall detection (chain-based + DBSCAN/PCA), wall mesh generation with corner snapping |
| 3 — Physics Simulation | PyBullet SimTransport with URDF model, simulated IMU/sonar, room creation for mapping tests |

## Current Work

- Hardware integration and testing
- WiFi WebREPL transport
- Wall mesh refinement (chain splitting at corners, gap filling)
- Web UI polish (ES modules, 3D dog visualization)

## Architecture

```
Browser (Web UI)  ←WebSocket/JSON→  Python Host (Dev PC)  ←Serial REPL / WiFi WebREPL→  Stock Firmware (MechDog)
```

See [docs/architecture.md](docs/architecture.md) for details.

## Getting Started

> **Prerequisites:** Stock Hiwonder MechDog, Python 3.11+

```bash
pip install -r host/requirements.txt

# Mock mode (no hardware)
python host/server.py

# USB serial
python host/server.py --serial /dev/ttyUSB0

# WiFi
python host/server.py --wifi 192.168.1.163

# PyBullet simulation
python host/server.py --sim
```

Open `http://localhost:8080` in your browser.

## Documentation

- [Architecture & Tech Stack](docs/architecture.md)
- [Design Decisions](docs/decisions.md)
- [Implementation Plan](docs/implementation-plan.md)
- [Communication Protocol](docs/protocol.md)
