# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog.

## Mission

Give the MechDog autonomous navigation. The operator sets goals from a browser UI; the dog handles navigation using its continuously updated world model.

The dog is a WiFi actuator with a camera. Everything that understands the world — reconstruction, mapping, planning — runs on a GPU machine on the same network. The dog carries no extra computer.

## Hardware Scope

- **Stock MechDog** — main controller, eight servos, IMU, forward sonar.
- **Vision module** — the kit's ESP32-S3 camera, running our own still-image firmware.
- **2D LiDAR (reserve)** — permitted if camera-only localization proves too weak. Not fitted.

## Kernel Scope

The kernel is the minimal set of components that must work before any autonomous behavior is possible:

- **Firmware** — motion engine on ESP32: IK gait, servo control, IMU/sonar telemetry, active balance, command dispatch
- **Mock firmware** — the same firmware C++ compiled as a native binary for desktop dev
- **Host** — Python server: transport to firmware, WebSocket to browser, composable behavior layers
- **Web** — primary control UI: D-pad, 3D dog view, diagnostics
- **CLI** — `bark` entry point

## Perception Roadmap

Built on the kernel, in order. Each stage is gated on the one before.

1. **Prove the camera** *(in progress)* — stop-and-snap keyframe capture, offline reconstruction with a feed-forward geometry model, metric scale from the known camera height, consistency checks against sonar and gyro. Go/no-go for camera-only navigation.
2. **Simulated world** — a rendered room and camera in the mock, so the navigation stack can be developed and scored without hardware.
3. **Live map and localization** — the offline pipeline run incrementally as a host behavior; map panel in the UI.
4. **Waypoint navigation** — click-to-go on the map: planner, path follower, obstacle stop.
5. **Language goals** — a vision-language model picks waypoints and labels places.

## Deferred Work

- Learned odometry and distilled depth models trained on the dog's own captures
- Learned (RL) gait
- Multi-platform robot support

## Core Docs

| Doc | Contents |
|-----|----------|
| [architecture.md](architecture.md) | Components, layers, data flow, transport boundary |
| [protocol.md](protocol.md) | Authoritative wire format: every command and telemetry message |
| [hardware.md](hardware.md) | Pins, peripherals, IMU axes, servo layout |
| [design-principles.md](design-principles.md) | Rules that govern how the project grows |
| [attic.md](attic.md) | Index of code removed in Phase 1; lives in `pre-reshape` branch |
