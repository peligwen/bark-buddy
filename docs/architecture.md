# Architecture & Tech Stack

## System Overview

```
                                    WiFi TCP (JSON/NDJSON)
┌─────────────────┐      ◄──────────────────────────►    ┌──────────────────────┐
│  Local Dev PC   │           port 9000 (primary)        │  MechDog             │
│                 │                                      │                      │
│  Python Host    │      USB Serial (debug/fallback)     │  Custom Firmware     │
│  - Web Server   │      ◄──────────────────────────►    │  - ESP32-S (D0WD) (C++)    │
│  - Behavior     │           115200 baud                │  - Gait Engine       │
│    Engine       │                                      │  - Servo Control     │
│  - Transport    │                                      │  - QMI8658 IMU       │
│    Layer        │                                      │  - I2C Sonar         │
│                 │                                      │  - Telemetry Stream  │
└─────────────────┘                                      └──────────────────────┘
       ▲
       │ HTTP/WebSocket (JSON)
       ▼
┌─────────────────┐
│  Browser (UI)   │
│  D-pad + 3D Dog │
│  + Scan Map     │
└─────────────────┘
```

### Stock Firmware Fallback

When the custom firmware isn't flashed, the Python host can talk to the stock MicroPython firmware via REPL commands over USB serial or WiFi WebREPL. This path is for bootstrapping (e.g., pin discovery, initial hardware validation) and as a fallback.

```
Python Host  ←Serial REPL / WiFi WebREPL→  Stock MicroPython Firmware
```

## Hardware

- **Hiwonder MechDog** — ESP32-S (D0WD), 8 coreless PWM servos (2/leg), QMI8658 IMU, I2C ultrasonic
- **No Raspberry Pi or extra sensors** — stock hardware only

## Firmware Paths

| Path | Firmware | Protocol | Transport | Use Case |
|---|---|---|---|---|
| **Primary** | Custom C++ (PlatformIO) | JSON/NDJSON | WiFi TCP :9000 | Production — full servo/gait/sensor control |
| **Fallback** | Stock MicroPython | REPL commands | USB serial / WiFi WebREPL | Bootstrapping, pin discovery, fallback |
| **Debug** | Either | Same as above | USB serial | Development, debugging, log inspection |

## Tech Stack

| Layer | Language | Tools | Responsibility |
|---|---|---|---|
| Custom Firmware | C++ | PlatformIO, ArduinoJson, ESP32-S (D0WD) | Gait engine, servo PWM, IMU/sonar streaming, balance, heartbeat, LED control |
| Stock Firmware | MicroPython | ESP32-S (D0WD) (stock) | Fallback motion via `_dog.move()`, REPL-accessible sensors |
| Host | Python 3.11+ | asyncio, pyserial-asyncio, aiohttp, websockets | Behavior layers, transport abstraction, web server, telemetry relay |
| Web UI | HTML/CSS/JS | Vanilla ES modules, Three.js r128 | D-pad control, 3D dog visualization, 2D scan map, telemetry gauges |
| Simulation | Python | PyBullet | Physics sim with URDF model, simulated sensors |

## Communication

- **Host → Custom Firmware:** JSON/NDJSON over WiFi TCP (port 9000) — commands and heartbeat
- **Custom Firmware → Host:** JSON/NDJSON telemetry stream (IMU, sonar, battery, status, acks)
- **Host → Stock Firmware (fallback):** REPL commands over USB serial or WiFi WebREPL
- **Host ↔ Browser:** WebSocket with JSON messages (same regardless of firmware path)
- Protocol spec: see `docs/protocol.md`

## Behavior Model

Composable layers, not exclusive modes:

```
┌──────────────────────────────┐
│  Active Behavior (top layer) │  ← Remote control OR Patrol OR Scan
├──────────────────────────────┤
│  Balance Layer               │  ← Custom FW: host-side PID; Stock FW: toggle homeostasis
├──────────────────────────────┤
│  Mapping Layer               │  ← Point cloud, wall detection, mesh generation
├──────────────────────────────┤
│  Transport Layer             │  ← WiFi TCP (primary) / Serial REPL (fallback) / Sim / Mock
└──────────────────────────────┘
```

## Project Structure

```
bark-buddy/
├── CLAUDE.md                # Claude Code project context
├── README.md                # Project overview
├── firmware/                # Custom C++ firmware (primary)
│   ├── src/                 # main.cpp, gait.cpp, imu.cpp, servos.cpp, sonar.cpp
│   ├── include/             # config.h, protocol.h, gait.h, imu.h, servos.h, sonar.h, poses.h
│   ├── test/                # kinematics, balance PID, gait, pose tests
│   └── platformio.ini       # ESP32-S (D0WD) PlatformIO config
├── host/                    # Python host
│   ├── server.py            # Web server + WebSocket handler
│   ├── comms.py             # Protocol layer + transport ABC
│   ├── firmware_transport.py # Custom firmware transport (JSON/NDJSON over WiFi/serial)
│   ├── hw_transport.py      # Stock firmware base (CMD→REPL translation)
│   ├── repl_transport.py    # Stock firmware: USB serial REPL (debug/fallback)
│   ├── webrepl_transport.py # Stock firmware: WiFi WebREPL (fallback)
│   ├── mock_serial.py       # Mock transport for dev
│   ├── mock_firmware.py     # Mock custom firmware for dev
│   ├── setup_wifi.py        # WiFi + WebREPL setup
│   ├── capture_profile.py   # Profile capture + parameter optimizer
│   ├── behaviors/           # balance, patrol, scan, map_store, wall_fit, wall_mesh, octree
│   ├── sim/                 # PyBullet sim (sim_transport.py, mechdog.urdf)
│   └── requirements.txt
├── web/                     # Static web UI (ES modules)
│   ├── index.html
│   ├── style.css
│   ├── app.module.js        # Main entry point
│   ├── modules/             # ws.js, controls.js, map.js, panels.js
│   └── dog3d/               # Three.js 3D (model, gait, camera, sonar, walls, overlay, state)
└── docs/
    ├── architecture.md      # This file
    ├── decisions.md         # Design decisions log
    ├── implementation-plan.md
    └── protocol.md          # Custom firmware + stock CMD protocol spec
```
