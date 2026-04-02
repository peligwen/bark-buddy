# Implementation Plan — Milestone 1 (Thin MVP)

## Scope

Minimal working versions of remote control, balance & recovery, and patrol using the MechDog's stock firmware and CMD protocol. No custom firmware needed.

## Phases

### Phase 1: Communication Foundation ✅

1. ~~Define protocol spec~~ → Using stock CMD protocol (`CMD|func|data|$`)
2. ~~Write Python comms layer~~ → `comms.py` with CMD protocol + abstract transport
3. ~~Write mock transport~~ → `mock_serial.py` for dev without hardware
4. ~~Verify round-trip~~ → Smoke test passes

### Phase 2: Basic Movement (Remote Control) ✅

1. ~~Build Python web server~~ → `server.py` with aiohttp (static files + WebSocket)
2. ~~Create web UI with D-pad~~ → `web/index.html`, `app.js`, `style.css`
3. ~~Wire WebSocket to comms~~ → browser D-pad → JSON → Python → CMD → serial
4. ~~Verify on mock~~ → 6 end-to-end checks pass (HTTP, WebSocket, telemetry, commands)
5. Verify on hardware: D-pad controls move the dog

### Phase 3: Balance & Telemetry ✅

1. ~~Enable stock self-balance~~ → `behaviors/balance.py` auto-enables on connect
2. ~~Poll IMU at 10 Hz~~ → via `balance.update()` in telemetry loop
3. ~~Poll battery at 0.5 Hz~~ → broadcast to browser
4. ~~2D gauges with color thresholds~~ → green/yellow/red based on tilt
5. ~~Status bar~~ → mode, balance on/off, battery %, connection
6. ~~Fall detection~~ → 35° threshold, auto-recovery, pulsing alert in UI

### Phase 4: Patrol ✅

1. ~~Waypoint format~~ → `Waypoint(x, y, heading)` dataclass
2. ~~Dead reckoning~~ → IMU heading + timed movement for position estimation
3. ~~Patrol behavior~~ → `behaviors/patrol.py` navigates waypoint sequence
4. ~~Web UI~~ → demo patrol button, stop button, position/waypoint display
5. Verify on hardware: dog navigates waypoint path

### Phase 5: Integration & Polish ✅

1. ~~Ultrasonic sensor~~ → `read_ultrasonic()` in comms, polled at 5 Hz, shown in status bar
2. ~~Mode switching~~ → remote ↔ patrol via web UI, balance always composable
3. ~~Reconnection~~ → detect serial loss, retry with exponential backoff, broadcast state
4. ~~Serial support~~ → `--serial /dev/ttyUSB0` flag for real hardware (mock by default)
5. ~~Action groups~~ → stand, wave, sit, lie down in web UI
6. ~~Status dashboard~~ → pitch/roll gauges, ultrasonic distance, battery %, mode, balance, connection

## Verification Matrix

| Capability | Test |
|---|---|
| Remote Control | Open web UI, D-pad controls move the dog via CMD protocol |
| Balance | Enable balance → push dog → stock firmware corrects |
| Telemetry | IMU gauges + battery + ultrasonic update in real-time in web UI |
| Patrol | Start patrol → dog navigates waypoints via dead reckoning |
| Connection Loss | Unplug serial → UI shows disconnected → replug → resumes |
| Composability | Patrol + balance active simultaneously |
| Ultrasonic | Distance display updates, color warnings for close objects |

## Milestone 2 Roadmap

### Ultrasonic Mapping
- Record ultrasonic distance readings with heading/position during patrol
- Build a 2D occupancy-style map from sweep data (turn in place, sample distances)
- Visualize scanned map in web UI (polar plot or 2D grid)
- Use map data for obstacle-aware waypoint planning

### Other Goals
- 3D orientation model in web UI (Three.js)
- Physics simulator for host-side testing (PyBullet/MuJoCo)
- Sensor-based room mapping (ultrasonic mapping feeds into this)
- Custom firmware for advanced gaits and fine-grained servo control
- WiFi transport

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits (trot/gallop), obstacle avoidance, SLAM.
