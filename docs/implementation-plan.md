# Implementation Plan — Milestone 1 (Thin MVP)

## Scope

Minimal working versions of remote control, balance & recovery, and patrol using the MechDog's stock firmware and CMD protocol. No custom firmware needed.

## Phases

### Phase 1: Communication Foundation ✅

1. ~~Define protocol spec~~ → Using stock CMD protocol (`CMD|func|data|$`)
2. ~~Write Python comms layer~~ → `comms.py` with CMD protocol + abstract transport
3. ~~Write mock transport~~ → `mock_serial.py` for dev without hardware
4. ~~Verify round-trip~~ → Smoke test passes

### Phase 2: Basic Movement (Remote Control)

1. Build Python web server (`server.py`) — serves static files + WebSocket
2. Create web UI with on-screen D-pad (`web/index.html`, `app.js`, `style.css`)
3. Wire WebSocket to comms layer: browser D-pad → JSON → Python → CMD → serial
4. Verify on mock: D-pad sends correct CMD strings, mock acks them
5. Verify on hardware: D-pad controls move the dog

### Phase 3: Balance & Telemetry

1. Enable stock self-balance via `CMD|1|3|1|$`
2. Poll IMU data (`CMD|5|$`) at 10 Hz, push to browser via WebSocket
3. Poll battery (`CMD|6|$`) at 0.5 Hz, push to browser
4. Display 2D gauges for pitch/roll in web UI
5. Display battery level and connection status
6. Verify: enable balance → push dog → it corrects; telemetry shows in UI

### Phase 4: Patrol

1. Define waypoint format: `(x, y, heading)` in local coordinate frame
2. Implement dead reckoning — IMU heading + timed movement for position estimation
3. Implement patrol behavior in Python (`behaviors/patrol.py`) — navigate waypoint sequence
4. Add patrol mode toggle to web UI
5. Verify: start patrol → dog navigates waypoint path

### Phase 5: Integration & Polish

1. Mode switching via web UI (remote / patrol) — balance always composable
2. Status dashboard: 2D gauges for pitch/roll, battery, mode, connection
3. Reconnection handling: detect serial timeout, retry with backoff
4. Action group triggers in UI (sit, wave, stretch)

## Verification Matrix

| Capability | Test |
|---|---|
| Remote Control | Open web UI, D-pad controls move the dog via CMD protocol |
| Balance | Enable balance → push dog → stock firmware corrects |
| Telemetry | IMU gauges + battery update in real-time in web UI |
| Patrol | Start patrol → dog navigates waypoints via dead reckoning |
| Connection Loss | Unplug serial → UI shows disconnected → replug → resumes |
| Composability | Patrol + balance active simultaneously |

## Late Goals (post-MVP)

- 3D orientation model in web UI (Three.js)
- Physics simulator for host-side testing (PyBullet/MuJoCo)
- Sensor-based room mapping for patrol navigation
- Custom firmware for advanced gaits and fine-grained servo control
- WiFi transport

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits (trot/gallop), obstacle avoidance, SLAM.
