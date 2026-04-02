# Implementation Plan — Milestone 1 (Thin MVP)

## Scope

Minimal working versions of remote control, balance & recovery, and patrol to validate the architecture.

## Phases

### Phase 1: Communication Foundation

1. Research MechDog stock controller (board type, existing SDK/protocol, serial interface)
2. Define command/telemetry protocol (JSON over serial or WiFi)
3. Write C++ firmware that accepts movement commands and reports IMU data
4. Write Python `comms.py` to send commands and receive telemetry

### Phase 2: Basic Movement (Remote Control)

1. Implement servo abstraction and gait engine in C++ (forward, backward, turn L/R, stop)
2. Build Python web server with WebSocket support
3. Create minimal web UI with directional controls
4. Wire end-to-end: browser → WebSocket → Python → serial → firmware → servos

### Phase 3: Balance & Recovery

1. Read IMU data on MechDog controller
2. Stream IMU telemetry to Python host
3. Implement balance correction (adjust stance based on tilt)
4. Implement fall detection + stand-up recovery sequence

### Phase 4: Patrol

1. Define patrol as a sequence of movement commands with timing
2. Implement patrol behavior in Python
3. Add patrol mode toggle to web UI

### Phase 5: Integration & Polish

1. Mode switching (remote / balance / patrol) via web UI
2. Status dashboard showing IMU data and current mode
3. Error handling for lost connection

## Verification

| Capability | Test |
|---|---|
| Remote Control | Open web UI, directional controls move the dog |
| Balance | Push dog lightly → correction; tip over → recovery attempt |
| Patrol | Start patrol mode → dog walks predefined path |
| Communication | Telemetry data appears in web UI in real-time |

## Out of Scope

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits (trot/gallop), obstacle avoidance, SLAM.
