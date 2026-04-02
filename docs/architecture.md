# Architecture & Tech Stack

## System Overview

```
┌─────────────────┐      Serial (→WiFi)     ┌──────────────────┐
│  Local Dev PC   │◄────────────────────►   │  MechDog (Stock) │
│                 │                         │                  │
│  Python Host    │   Bidirectional JSON    │  C/C++ Firmware  │
│  - Web Server   │   (tagged messages)     │  - Servo Control │
│  - Behavior     │                         │  - IMU Reading   │
│    Engine       │                         │  - Gait Engine   │
│  - (Future: AI) │                         │  - Balance Layer │
└─────────────────┘                         └──────────────────┘
       ▲
       │ HTTP/WebSocket
       ▼
┌─────────────────┐
│  Browser (UI)   │
│  D-pad + Gauges │
└─────────────────┘
```

## Hardware

- **Stock Hiwonder MechDog** — servos + controller board + IMU
- No Raspberry Pi or extra sensors for Milestone 1

## Tech Stack

| Layer | Language | Tools | Responsibility |
|---|---|---|---|
| Firmware | C++17 | PlatformIO | Servo control, parametric gait/IK, IMU reading, balance (comp. filter + PID), serial comms |
| Host | Python 3.11+ | asyncio, websockets | Behavior layers, web server, communication bridge, mock serial for dev |
| Web UI | HTML/CSS/JS | Vanilla (no framework) | D-pad remote control, 2D gauges, status dashboard |

## Communication

- **Transport:** Serial (USB) initially, WiFi later via abstracted transport layer
- **Protocol:** Bidirectional tagged JSON messages — both sides send independently
- **Firmware ↔ Host:** Commands down, telemetry up, acks as needed
- **Host ↔ Browser:** WebSocket for real-time control and telemetry
- **Connection loss:** Auto-reconnect with servo freeze
- Protocol spec: see `docs/protocol.md`

## Behavior Model

Composable layers, not exclusive modes:

```
┌──────────────────────────────┐
│  Active Behavior (top layer) │  ← Remote control OR Patrol
├──────────────────────────────┤
│  Balance Layer (always-on)   │  ← Complementary filter + PID
├──────────────────────────────┤
│  Gait Engine                 │  ← Parametric / IK
├──────────────────────────────┤
│  Servo Abstraction           │
└──────────────────────────────┘
```

Balance correction runs continuously beneath whatever behavior is active.

## AI Integration (Future)

- **Development time:** Claude generates firmware and behavior code
- **Runtime (post-MVP):** Local network server running Ollama/Llama/Phi for decision-making

## Project Structure

```
bark-buddy/
├── CLAUDE.md              # Claude Code project context (auto-loaded)
├── README.md              # Human-facing project overview
├── firmware/              # C/C++ — MechDog controller
│   ├── src/
│   │   ├── main.cpp       # Entry point, serial comm, message routing
│   │   ├── gait.cpp       # Parametric gait engine / IK
│   │   ├── balance.cpp    # Complementary filter + PID balance
│   │   └── servos.cpp     # Servo abstraction layer
│   ├── include/
│   │   └── protocol.h     # Message types and structures
│   └── platformio.ini     # PlatformIO build config
├── host/                  # Python — local dev machine
│   ├── server.py          # Web server + WebSocket handler
│   ├── comms.py           # Abstract transport (serial/WiFi)
│   ├── mock_serial.py     # Serial stub for dev without hardware
│   ├── behaviors/
│   │   ├── remote.py      # Manual remote control layer
│   │   ├── balance.py     # Balance & recovery layer
│   │   └── patrol.py      # Patrol behavior (waypoint navigation)
│   └── requirements.txt
├── web/                   # Static web UI
│   ├── index.html
│   ├── style.css
│   └── app.js
└── docs/
    ├── architecture.md    # This file
    ├── decisions.md       # Design decisions log
    ├── implementation-plan.md
    └── protocol.md        # Communication protocol spec
```
