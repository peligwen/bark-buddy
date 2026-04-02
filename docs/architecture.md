# Architecture & Tech Stack

## System Overview

```
┌─────────────────┐      WiFi/Serial      ┌──────────────────┐
│  Local Dev PC   │◄────────────────────►  │  MechDog (Stock) │
│                 │                        │                  │
│  Python Host    │                        │  C/C++ Firmware  │
│  - Web Server   │   Commands/Telemetry   │  - Servo Control │
│  - Behavior     │                        │  - IMU Reading   │
│    Engine       │                        │  - Gait Engine   │
│  - (Future: AI) │                        │                  │
└─────────────────┘                        └──────────────────┘
       ▲
       │ HTTP/WebSocket
       ▼
┌─────────────────┐
│  Browser (UI)   │
│  Remote Control │
└─────────────────┘
```

## Hardware

- **Stock Hiwonder MechDog** — servos + controller board + IMU
- No Raspberry Pi or extra sensors for Milestone 1

## Tech Stack

| Layer | Language | Tools | Responsibility |
|---|---|---|---|
| Firmware | C++17 | PlatformIO | Servo control, gait engine, IMU reading, serial/WiFi comms |
| Host | Python 3.11+ | asyncio, websockets | Behavior logic, web server, communication bridge |
| Web UI | HTML/CSS/JS | Vanilla (no framework) | Remote control interface, status dashboard |

## Communication

- **Firmware ↔ Host:** JSON messages over serial or WiFi
- **Host ↔ Browser:** WebSocket for real-time control and telemetry
- Protocol spec: see `docs/protocol.md` (TBD)

## AI Integration (Future)

- **Development time:** Claude generates firmware and behavior code
- **Runtime (post-MVP):** Local network server running Ollama/Llama/Phi for decision-making; robot calls via API

## Project Structure

```
bark-buddy/
├── CLAUDE.md              # Claude Code project context (auto-loaded)
├── README.md              # Human-facing project overview
├── firmware/              # C/C++ — MechDog controller
│   ├── src/
│   │   ├── main.cpp       # Entry point, serial/WiFi comm
│   │   ├── gait.cpp       # Gait engine (walk, turn, stand)
│   │   ├── balance.cpp    # IMU-based balancing
│   │   └── servos.cpp     # Servo abstraction layer
│   ├── include/
│   └── platformio.ini     # PlatformIO build config
├── host/                  # Python — local dev machine
│   ├── server.py          # Web server + WebSocket handler
│   ├── comms.py           # Serial/WiFi communication with dog
│   ├── behaviors/
│   │   ├── remote.py      # Manual remote control mode
│   │   ├── balance.py     # Balance & recovery behavior
│   │   └── patrol.py      # Autonomous patrol behavior
│   └── requirements.txt
├── web/                   # Static web UI
│   ├── index.html
│   ├── style.css
│   └── app.js
└── docs/
    ├── architecture.md    # This file
    ├── implementation-plan.md
    └── protocol.md        # Communication protocol spec (TBD)
```
