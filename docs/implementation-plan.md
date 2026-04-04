# Implementation Plan

## Scope

Custom C++ firmware running on the MechDog's ESP32-S (D0WD), communicating over WiFi with a Python host that provides behaviors, a web UI, and 3D visualization. The stock MicroPython firmware was used for bootstrapping and remains available as a fallback. Serial is the debug transport.

## Bootstrapping (Stock Firmware) ✅

These phases used the stock MicroPython firmware and REPL protocol to bring up the host, web UI, and behavior layers without needing custom firmware.

### Phase 1: Communication Foundation ✅

1. ~~Define protocol spec~~ → Using stock CMD protocol (`CMD|func|data|$`)
2. ~~Write Python comms layer~~ → `comms.py` with CMD protocol + abstract transport
3. ~~Write mock transport~~ → `mock_serial.py` for dev without hardware
4. ~~Verify round-trip~~ → Smoke test passes

### Phase 2: Basic Movement (Remote Control) ✅

1. ~~Build Python web server~~ → `server.py` with aiohttp (static files + WebSocket)
2. ~~Create web UI with D-pad~~ → `web/index.html`, `app.module.js`, `style.css`
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

### Phase 5: Integration & Polish ✅

1. ~~Ultrasonic sensor~~ → `read_ultrasonic()` in comms, polled at 5 Hz, shown in status bar
2. ~~Mode switching~~ → remote ↔ patrol via web UI, balance always composable
3. ~~Reconnection~~ → detect serial loss, retry with exponential backoff, broadcast state
4. ~~Serial support~~ → `--serial /dev/ttyUSB0` flag for real hardware (mock by default)
5. ~~Action groups~~ → stand, wave, sit, lie down in web UI
6. ~~Status dashboard~~ → pitch/roll gauges, ultrasonic distance, battery %, mode, balance, connection

## Ultrasonic Mapping ✅

### Scan Behavior & Map Store ✅

1. ~~ScanBehavior~~ → `behaviors/scan.py` — 360° sweep in 15° steps, median-filtered readings
2. ~~MapStore~~ → `behaviors/map_store.py` — accumulates scan results into point cloud
3. ~~Server integration~~ → `cmd_scan` (start/stop), `cmd_map` (get/clear) WebSocket commands
4. ~~Web UI~~ → canvas-based 2D map with color-coded distance points, scan progress bar
5. ~~Tests~~ → 10 checks: scan execute/cancel/coords/serialization, map add/bounds/clear/dict, WS scan/map

## Physics Simulation ✅

### SimTransport + URDF ✅

1. ~~URDF model~~ → `sim/mechdog.urdf` — 8-DOF MechDog from Hiwonder specs (173x73x50mm body, 55mm upper + 60mm lower legs, 4 feet with friction)
2. ~~SimTransport~~ → `sim/sim_transport.py` — PyBullet transport implementing CMD protocol
   - Velocity-based forward/backward movement (0.10 m/s matching stock firmware)
   - Kinematic turning (45°/s matching stock firmware)
   - Body stabilization (pitch/roll correction simulating self-balance)
   - Ultrasonic cone ray casting (5 rays, 30° cone, 3m max range)
   - Simulated IMU from body orientation
   - Room/wall creation for mapping tests (`add_wall`, `add_box_room`)
   - Fast-forward via `speed_factor` parameter (headless DIRECT mode)
3. ~~Tests~~ → 12 checks: connect, standing, fwd/back/left/right, IMU, ultrasonic open/wall, battery, room walls, heading-aware forward

## Wall Mesh & Visualization ✅

### Wall Detection Pipeline ✅

1. ~~Wall fitting~~ → `behaviors/wall_fit.py` — DBSCAN clustering + PCA line fitting as fallback
2. ~~Wall mesh~~ → `behaviors/wall_mesh.py` — chain-based mesh from point cloud vertices
3. ~~Octree~~ → `behaviors/octree.py` — spatial indexing for point cloud queries
4. ~~Point cloud management~~ → consolidation (merge dense clusters, cap count), decay + reinforcement
5. ~~Tests~~ → `test_mapping.py`, `test_wall_mesh.py` — CLI test suite for mapping pipeline

### Mesh Refinement ✅

1. ~~Corner detection~~ → extend walls to meet via line intersection, snap endpoints
2. ~~Chain splitting~~ → split chains at corners, preserve long chains
3. ~~Gap filling~~ → bridge isolated wall endpoints with short segments
4. ~~Collinear merge~~ → merge collinear wall segments for connected rendering
5. ~~Parameter tuning~~ → sweep across 3 room geometries for optimal merge constants

### 3D Visualization ✅

1. ~~Three.js dog model~~ → `web/dog3d/model.js` — 3D MechDog with joint markers (hip=cyan, knee=orange, foot=magenta)
2. ~~Gait animation~~ → `web/dog3d/gait.js` — leg animation synced to movement
3. ~~Camera control~~ → `web/dog3d/camera.js` — orbiting/zoom camera
4. ~~Sonar visualization~~ → `web/dog3d/sonar.js` — ultrasonic beam display
5. ~~Wall rendering~~ → `web/dog3d/walls.js` — 3D wall mesh from scan data
6. ~~Overlay~~ → `web/dog3d/overlay.js` — kinematics overlay with K shortcut
7. ~~Pose system~~ → action poses (wave, sit, lie down) with interpolation

### Web UI Modernization ✅

1. ~~ES modules~~ → refactored JS monoliths into `web/modules/` (ws, controls, map, panels) + `web/dog3d/`
2. ~~Server management~~ → restart from UI button and CLI (SIGTERM + subprocess)

## Custom Firmware (Primary) ✅ built, deploying

The custom C++ firmware is implemented and tested but awaiting final hardware deployment (servo pin verification).

### Firmware Implementation ✅

1. ~~JSON/NDJSON protocol~~ → `protocol.h` — structured message types for commands, telemetry, acks, events
2. ~~Main loop~~ → `main.cpp` — serial/WiFi RX, heartbeat timeout, telemetry streaming, gait tick
3. ~~Gait engine~~ → `gait.cpp` — parametric walk (hip/knee amplitude, frequency, phase), PID-ready balance
4. ~~IMU driver~~ → `imu.cpp` — QMI8658 via I2C, pitch/roll/yaw + accel + gyro
5. ~~Sonar driver~~ → `sonar.cpp` — I2C ultrasonic + RGB LED control
6. ~~Servo control~~ → `servos.cpp` — 8-channel PWM, soft-start ramp, idle timeout + detach
7. ~~Hardware config~~ → `config.h` — pin assignments (I2C verified, servos gated by `PINS_VERIFIED`), timing, gait params
8. ~~Firmware tests~~ → kinematics, balance PID, gait parameter sweep, pose validation

### WiFi Transport (In Progress)

1. Custom firmware WiFi listener on TCP port 9000
2. `firmware_transport.py` — Python host transport for JSON/NDJSON over WiFi TCP
3. Integration test: host connects over WiFi, sends commands, receives telemetry

### Hardware Deployment (In Progress)

1. Verify servo GPIO pins via stock firmware REPL introspection
2. Set `PINS_VERIFIED=1` and flash custom firmware
3. End-to-end test: WiFi connection → gait → telemetry → web UI
4. Validate all behaviors (remote, balance, patrol, scan) on custom firmware

## Future (Planned)

### Multi-Scan & Patrol Integration

- Scan at each patrol waypoint to build composite map
- Merge overlapping scans with position correction
- Map persistence (save/load)

### Obstacle-Aware Planning

- Use map data to identify obstacles
- Generate obstacle-aware waypoint paths
- Real-time obstacle avoidance during patrol

### Advanced Gait

- Profile-based gait optimization
- Custom gaits (trot, gallop) via parametric gait engine

## Verification Matrix

| Capability | Test |
|---|---|
| Remote Control | Open web UI, D-pad controls move the dog |
| Balance | Enable balance → push dog → firmware corrects |
| Telemetry | IMU gauges + battery + ultrasonic update in real-time in web UI |
| Patrol | Start patrol → dog navigates waypoints via dead reckoning |
| Connection Loss | Disconnect WiFi → UI shows disconnected → reconnect → resumes |
| Composability | Patrol + balance active simultaneously |
| Ultrasonic | Distance display updates, color warnings for close objects |
| Wall Mapping | Scan → point cloud → wall detection → 3D wall mesh in UI |
| Custom Firmware | Flash → WiFi connect → gait walks → telemetry streams |

## Out of Scope

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, SLAM.
