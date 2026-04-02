# bark-buddy

Give our Hiwonder MechDog Open Source AI Robot Dog some real brains.

## What Is This?

Firmware and host software for the [Hiwonder MechDog](https://www.hiwonder.com/) robot dog. Uses a hybrid C++/Python architecture: C++ runs on the MechDog controller for real-time servo and IMU control, while Python runs on a local dev machine for high-level behaviors and a web-based remote control interface.

## Current Milestone: Thin MVP

| Capability | Description |
|---|---|
| Remote Control | Web UI to walk, turn, and stop the dog |
| Balance & Recovery | IMU-based tilt correction and fall recovery |
| Patrol | Autonomous predefined movement sequences |

## Architecture

```
Browser (Web UI)  ←HTTP/WS→  Python Host (Dev PC)  ←Serial/WiFi→  C++ Firmware (MechDog)
```

See [docs/architecture.md](docs/architecture.md) for details.

## Getting Started

> **Prerequisites:** Stock Hiwonder MechDog, PlatformIO, Python 3.11+

*Setup instructions will be added as implementation progresses.*

## Documentation

- [Architecture & Tech Stack](docs/architecture.md)
- [Implementation Plan](docs/implementation-plan.md)
- Communication Protocol (docs/protocol.md — TBD)
