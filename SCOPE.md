# Bark-Buddy: Scope & Implementation Plan

## Decided Scope

### First Milestone: Full-Stack Thin MVP

A minimal working version of each capability to prove the architecture works end-to-end.

**Hardware:** Stock Hiwonder MechDog (no Raspberry Pi or extra sensors)

### Priority Capabilities

| Capability | Thin MVP Definition |
|---|---|
| **Remote Control** | Web UI with directional controls; dog walks, turns, stops |
| **Balance & Recovery** | IMU-based tilt correction; fall detection + stand-up attempt |
| **Patrol & Inspect** | Walk a predefined sequence of movement commands autonomously |

### Tech Stack: Hybrid Python + C/C++

- **C/C++ (on MechDog controller):** Low-level servo control, real-time gait loops, IMU reading
- **Python (on local dev machine):** High-level behavior logic, web server for remote control, AI integration
- **Communication:** Serial or WiFi between the MechDog controller and the Python host

### AI Integration: Hybrid with Local Models

- **Offline:** Claude generates behavior scripts and firmware code during development
- **Runtime (future):** Local network server running a small model (Ollama/Llama/Phi) for decision-making; the robot calls it via API. **Skipped for first milestone** — focus on deterministic behaviors first.

### Remote Control: Web Interface

- Simple web UI served from the local dev machine
- Controls: directional movement, speed, stop, stance changes
- Status display: connection state, IMU readings, current mode

---

## Implementation Plan

### Phase 1: Communication Foundation

1. Research MechDog stock controller (board type, existing SDK/protocol, serial interface)
2. Define a simple command/telemetry protocol (JSON over serial or WiFi)
3. Write C++ firmware that accepts movement commands and reports IMU data
4. Write Python `comms.py` to send commands and receive telemetry

### Phase 2: Basic Movement (Remote Control)

1. Implement servo abstraction and basic gait engine in C++ (walk forward, backward, turn left/right, stop)
2. Build Python web server with WebSocket support
3. Create minimal web UI with directional controls
4. Wire it all together: browser -> WebSocket -> Python -> serial -> firmware -> servos

### Phase 3: Balance & Recovery

1. Read IMU data on the MechDog controller
2. Stream IMU telemetry to the Python host
3. Implement basic balance correction (adjust stance based on tilt)
4. Implement fall detection + stand-up recovery sequence

### Phase 4: Patrol

1. Define a simple patrol as a sequence of movement commands with timing
2. Implement patrol behavior in Python that sends commands according to the sequence
3. Add patrol mode toggle to web UI

### Phase 5: Integration & Polish

1. Mode switching (remote / balance / patrol) via web UI
2. Status dashboard showing IMU data and current mode
3. Basic error handling for lost connection

---

## Verification

- **Remote Control:** Open web UI, verify directional controls move the dog
- **Balance:** Push the dog lightly, observe correction; tip it over, observe recovery attempt
- **Patrol:** Start patrol mode, dog walks a predefined path autonomously
- **Communication:** Verify telemetry data appears in web UI in real-time

---

## Out of Scope (First Milestone)

- Camera / computer vision
- Object carrying
- AI-powered decision making at runtime
- Mobile app
- Raspberry Pi integration
- Advanced gaits (trot, gallop)
- Obstacle avoidance
- Mapping / SLAM
