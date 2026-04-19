# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog.

## Mission

Give the MechDog autonomous navigation on a single stock hardware platform — no Raspberry Pi, no extra sensors. The operator sets goals from a browser UI; the dog handles navigation using its continuously updated world model.

## Kernel Scope

The kernel is the minimal set of components that must work before any autonomous behavior is possible:

- **Firmware** — motion engine on ESP32: IK gait, servo control, IMU/sonar telemetry, active balance, command dispatch
- **Mock firmware** — the same firmware C++ compiled as a native binary for desktop dev
- **Host** — Python server: transport to firmware, WebSocket to browser, composable behavior layers
- **Web** — primary control UI: D-pad, 3D dog view, diagnostics
- **CLI** — `bark` entry point

## Deferred Work

Not in scope until the kernel is stable:

- SLAM-based localization (IMU + sonar scan matching)
- Composite multi-scan mapping
- Waypoint navigation UI
- Camera/vision (ESP32-S3 integration)
- Semantic understanding / LLM goals
- Multi-platform robot support

## Core Docs

| Doc | Contents |
|-----|----------|
| [architecture.md](architecture.md) | Components, layers, data flow, transport boundary |
| [protocol.md](protocol.md) | Authoritative wire format: every command and telemetry message |
| [hardware.md](hardware.md) | Pins, peripherals, IMU axes, servo layout |
| [design-principles.md](design-principles.md) | Rules that govern how the project grows |
| [attic.md](attic.md) | Index of code removed in Phase 1; lives in `pre-reshape` branch |
