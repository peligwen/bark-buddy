# Implementation Plan — Milestone 1 (Thin MVP)

## Scope

Minimal working versions of remote control, balance & recovery, and patrol to validate the architecture. See [decisions.md](decisions.md) for design rationale.

## Phases

### Phase 1: Communication Foundation

1. Research MechDog stock controller (board type, existing SDK/protocol, serial interface)
2. Define bidirectional tagged JSON protocol (`docs/protocol.md`)
3. Write C++ firmware entry point: serial comms, message parser, message router
4. Write Python `comms.py` with abstract transport layer (serial impl + WiFi interface for later)
5. Write `mock_serial.py` — serial stub that echoes commands and generates fake IMU data
6. Verify: host sends command via mock → gets ack; mock streams fake telemetry → host receives

### Phase 2: Basic Movement (Remote Control)

1. Implement servo abstraction layer in C++ (`servos.cpp`)
2. Implement parametric gait engine / IK in C++ (`gait.cpp`) — forward, backward, turn L/R, stop
3. Build Python web server (`server.py`) — serves static files + WebSocket
4. Create web UI with on-screen D-pad (`web/index.html`, `app.js`, `style.css`)
5. Wire end-to-end: browser → WebSocket → Python → serial → firmware → servos
6. Verify: open web UI, tap D-pad, dog moves

### Phase 3: Balance & Recovery

1. Read IMU data on MechDog controller (accelerometer + gyroscope)
2. Implement complementary filter for stable tilt estimation
3. Implement PID controller for continuous balance correction (`balance.cpp`)
4. Wire balance as a composable layer — runs beneath remote control
5. Stream IMU telemetry to Python host → forward to browser
6. Implement fall detection + stand-up recovery sequence
7. Verify: push dog → corrects; tip over → recovery; telemetry visible in UI

### Phase 4: Patrol

1. Define waypoint format: `(x, y, heading)` in local coordinate frame
2. Implement dead reckoning — IMU heading + step counting for position estimation
3. Implement patrol behavior in Python (`behaviors/patrol.py`) — navigate waypoint sequence
4. Add patrol mode toggle to web UI
5. Verify: start patrol → dog navigates waypoint path, balance layer active throughout

### Phase 5: Integration & Polish

1. Mode switching via web UI (remote / patrol) — balance always composable
2. Status dashboard: 2D gauges for pitch/roll, status bar for mode/connection
3. Auto-reconnect with servo freeze on connection loss
4. Error handling for lost connection, invalid messages, servo faults

## Verification Matrix

| Capability | Test |
|---|---|
| Remote Control | Open web UI, D-pad controls move the dog |
| Balance | Push dog lightly → PID correction; tip over → stand-up recovery |
| Patrol | Start patrol → dog navigates waypoints via dead reckoning |
| Telemetry | IMU gauges update in real-time in web UI |
| Connection Loss | Unplug serial → dog freezes → replug → resumes |
| Composability | Patrol + balance active simultaneously; remote + balance active simultaneously |

## Late Goals (post-MVP)

- 3D orientation model in web UI (Three.js)
- Physics simulator for host-side testing (PyBullet/MuJoCo)
- Sensor-based room mapping for patrol navigation
- WiFi transport (swap serial for TCP via abstract transport layer)

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits (trot/gallop), obstacle avoidance, SLAM.
