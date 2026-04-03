# Implementation Plan

## Scope

Remote control, balance, patrol, ultrasonic mapping with wall detection, and physics simulation — all using the MechDog's stock firmware and REPL protocol. No custom firmware needed.

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

## Milestone 2: Ultrasonic Mapping

### Phase 1: Scan Behavior & Map Store ✅

1. ~~ScanBehavior~~ → `behaviors/scan.py` — 360° sweep in 15° steps, median-filtered readings
2. ~~MapStore~~ → `behaviors/map_store.py` — accumulates scan results into point cloud
3. ~~Server integration~~ → `cmd_scan` (start/stop), `cmd_map` (get/clear) WebSocket commands
4. ~~Web UI~~ → canvas-based 2D map with color-coded distance points, scan progress bar
5. ~~Tests~~ → 10 checks: scan execute/cancel/coords/serialization, map add/bounds/clear/dict, WS scan/map

## Milestone 3: Physics Simulation Engine ✅

### Phase 1: SimTransport + URDF ✅

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

## Milestone 4: Wall Mesh & Visualization ✅

### Phase 1: Wall Detection Pipeline ✅

1. ~~Wall fitting~~ → `behaviors/wall_fit.py` — DBSCAN clustering + PCA line fitting as fallback
2. ~~Wall mesh~~ → `behaviors/wall_mesh.py` — chain-based mesh from point cloud vertices
3. ~~Octree~~ → `behaviors/octree.py` — spatial indexing for point cloud queries
4. ~~Point cloud management~~ → consolidation (merge dense clusters, cap count), decay + reinforcement
5. ~~Tests~~ → `test_mapping.py`, `test_wall_mesh.py` — CLI test suite for mapping pipeline

### Phase 2: Mesh Refinement ✅

1. ~~Corner detection~~ → extend walls to meet via line intersection, snap endpoints
2. ~~Chain splitting~~ → split chains at corners, preserve long chains
3. ~~Gap filling~~ → bridge isolated wall endpoints with short segments
4. ~~Collinear merge~~ → merge collinear wall segments for connected rendering
5. ~~Parameter tuning~~ → sweep across 3 room geometries for optimal merge constants

### Phase 3: 3D Visualization ✅

1. ~~Three.js dog model~~ → `web/dog3d/model.js` — 3D MechDog with joint markers (hip=cyan, knee=orange, foot=magenta)
2. ~~Gait animation~~ → `web/dog3d/gait.js` — leg animation synced to movement
3. ~~Camera control~~ → `web/dog3d/camera.js` — orbiting/zoom camera
4. ~~Sonar visualization~~ → `web/dog3d/sonar.js` — ultrasonic beam display
5. ~~Wall rendering~~ → `web/dog3d/walls.js` — 3D wall mesh from scan data
6. ~~Overlay~~ → `web/dog3d/overlay.js` — kinematics overlay with K shortcut
7. ~~Pose system~~ → action poses (wave, sit, lie down) with interpolation

### Phase 4: Web UI Modernization ✅

1. ~~ES modules~~ → refactored JS monoliths into `web/modules/` (ws, controls, map, panels) + `web/dog3d/`
2. ~~Server management~~ → restart from UI button and CLI (SIGTERM + subprocess)

## Current Work

- **Hardware integration** — verify all behaviors on physical MechDog
- **WiFi transport** — `webrepl_transport.py` testing with real WebREPL
- **UI refinement** — polish 3D visualization, improve wall rendering fidelity

## Future (Planned)

### Milestone 2 Phase 2: Multi-Scan & Patrol Integration

- Scan at each patrol waypoint to build composite map
- Merge overlapping scans with position correction
- Map persistence (save/load)

### Milestone 2 Phase 3: Obstacle-Aware Planning

- Use map data to identify obstacles
- Generate obstacle-aware waypoint paths
- Real-time obstacle avoidance during patrol

### Other Goals

- Custom firmware for advanced gaits and fine-grained servo control
- Profile-based gait optimization

## Out of Scope

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits (trot/gallop), SLAM.
