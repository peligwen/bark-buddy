# Architecture & Tech Stack

## System Overview

```
┌─────────────────┐      USB Serial (UART)   ┌──────────────────┐
│  Local Dev PC   │◄────────────────────►    │  MechDog (Stock) │
│                 │                          │                  │
│  Python Host    │   CMD protocol           │  Stock Firmware  │
│  - Web Server   │   (text-based)           │  - ESP32-S3      │
│  - Behavior     │                          │  - Servo Control │
│    Engine       │                          │  - MPU6050 IMU   │
│  - CMD comms    │                          │  - Self-Balance  │
│  - (Future: AI) │                          │  - 8x PWM Servos │
└─────────────────┘                          └──────────────────┘
       ▲
       │ HTTP/WebSocket (JSON)
       ▼
┌─────────────────┐
│  Browser (UI)   │
│  D-pad + Gauges │
└─────────────────┘
```

## Hardware

- **Stock Hiwonder MechDog** — ESP32-S3, 8 coreless PWM servos (2/leg), MPU6050 IMU
- **Stock firmware** — built-in CMD protocol for motion, balance, IMU, battery
- No Raspberry Pi or extra sensors for Milestone 1

## Tech Stack

| Layer | Language | Tools | Responsibility |
|---|---|---|---|
| MechDog | Stock firmware | ESP32-S3 + MPU6050 | Servo control, built-in balance, IMU, CMD protocol |
| Host | Python 3.11+ | asyncio, pyserial-asyncio, aiohttp | Behavior layers, CMD translation, web server, telemetry polling |
| Web UI | HTML/CSS/JS | Vanilla (no framework) | D-pad remote control, 2D gauges, status dashboard |

## Communication

- **Host → MechDog:** CMD text protocol over USB serial (`CMD|<func>|<data>|$`)
- **Host ↔ Browser:** WebSocket with JSON messages
- **Telemetry:** Host polls IMU + battery at regular intervals, pushes to browser
- Protocol spec: see `docs/protocol.md`

## Behavior Model

Composable layers, not exclusive modes:

```
┌──────────────────────────────┐
│  Active Behavior (top layer) │  ← Remote control OR Patrol
├──────────────────────────────┤
│  Balance Layer               │  ← Toggles stock self-balance (CMD|1|3|1|$)
├──────────────────────────────┤
│  CMD Protocol Layer          │  ← Translates behavior commands to CMD strings
├──────────────────────────────┤
│  Serial Transport            │  ← pyserial-asyncio
└──────────────────────────────┘
```

Balance is toggled on the stock firmware. Higher-level behaviors (patrol, remote)
issue movement commands that the CMD layer translates to serial commands.

## AI Integration (Future)

- **Development time:** Claude generates behavior code
- **Runtime (post-MVP):** Local network server running Ollama/Llama/Phi for decision-making

## Project Structure

```
bark-buddy/
├── CLAUDE.md              # Claude Code project context (auto-loaded)
├── README.md              # Human-facing project overview
├── firmware/              # C/C++ — future custom firmware (not used for MVP)
│   ├── src/main.cpp       # Reference: custom firmware scaffold
│   ├── include/protocol.h # Reference: JSON protocol types
│   └── platformio.ini     # ESP32-S3 PlatformIO config
├── host/                  # Python — local dev machine
│   ├── server.py          # Web server + WebSocket handler
│   ├── comms.py           # CMD protocol layer + serial transport
│   ├── mock_serial.py     # Mock transport for dev without hardware
│   ├── behaviors/
│   │   ├── remote.py      # Manual remote control layer
│   │   ├── balance.py     # Balance toggle + monitoring
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
    └── protocol.md        # CMD + WebSocket protocol spec
```
