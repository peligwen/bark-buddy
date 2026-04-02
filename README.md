# bark-buddy

Give our Hiwonder MechDog Open Source AI Robot Dog some real brains.

## Vision

Use Claude to write firmware for the dog that enables real-world capabilities like patrolling and inspecting areas, carrying objects, balancing, and recovering from a fall. Explore the capabilities!

## Milestone 1: Thin MVP

A minimal working version of each core capability to validate the full architecture end-to-end.

### Priority Capabilities

1. **Remote Control** — Web interface to manually command the dog (walk, turn, stop)
2. **Balance & Recovery** — Basic self-balancing using IMU data; detect a fall and attempt to stand back up
3. **Patrol & Inspect** — Walk a predefined path autonomously

### Tech Stack

- **C/C++ (MechDog controller):** Low-level servo control, real-time gait loops, IMU reading
- **Python (local dev machine):** High-level behavior logic, web server for remote control, AI integration
- **Communication:** Serial or WiFi between MechDog controller and Python host

### AI Integration

- **Offline:** Claude generates behavior scripts and firmware code during development
- **Runtime (future):** Local network server running a small model (Ollama/Llama/Phi) for decision-making — skipped for first milestone

### Architecture

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
       │ HTTP
       ▼
┌─────────────────┐
│  Browser (UI)   │
│  Remote Control │
└─────────────────┘
```

### Project Structure

```
bark-buddy/
├── README.md
├── SCOPE.md               # Detailed scope & implementation plan
├── firmware/              # C/C++ code for MechDog controller
│   ├── src/
│   │   ├── main.cpp       # Entry point, serial/WiFi comm
│   │   ├── gait.cpp       # Gait engine (walk, turn, stand)
│   │   ├── balance.cpp    # IMU-based balancing
│   │   └── servos.cpp     # Servo abstraction layer
│   ├── include/
│   └── platformio.ini     # Build config (PlatformIO)
├── host/                  # Python host application
│   ├── server.py          # Web server + WebSocket handler
│   ├── behaviors/
│   │   ├── remote.py      # Manual remote control mode
│   │   ├── balance.py     # Balance & recovery behavior
│   │   └── patrol.py      # Autonomous patrol behavior
│   ├── comms.py           # Serial/WiFi communication with dog
│   └── requirements.txt
├── web/                   # Web UI for remote control
│   ├── index.html
│   ├── style.css
│   └── app.js
└── docs/
    └── protocol.md        # Communication protocol spec
```
