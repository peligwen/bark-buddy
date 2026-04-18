# Design Decisions

Captured during project scoping and evolution (April 2026).

## Firmware Approach

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Primary firmware | Custom C++ via PlatformIO | Full control over gait engine, servo PWM, sensor streaming, and balance. JSON protocol is cleaner than stock CMD. WiFi TCP enables wireless operation |
| Stock firmware role | ~~Fallback and bootstrapping~~ — resolved | Stock paths deleted in Custom-Firmware-Only Refactor. Pin discovery via stock REPL is complete; `PINS_VERIFIED=1` is set. Custom firmware is the only supported path. |
| Servo pin verification | `PINS_VERIFIED` gate in config.h | Custom firmware won't drive servos until GPIO pins are confirmed. Pin discovery was done via stock REPL introspection during bootstrapping; pins are now verified. |

## Communication

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Primary transport | WiFi TCP (port 9000) with JSON/NDJSON | Wireless operation, structured protocol, bidirectional telemetry streaming. Custom firmware listens, host connects |
| Debug transport | USB serial (115200 baud) | Used for development, log inspection, and flashing. Also supported as a live transport by `FirmwareTransport`. |
| Fallback transport | ~~Serial REPL / WiFi WebREPL~~ — removed | Stock firmware REPL transport deleted in Custom-Firmware-Only Refactor. `FirmwareTransport` is the sole transport. |
| Firmware protocol | JSON/NDJSON (newline-delimited) | ArduinoJson on firmware side. Structured, extensible, same format as WebSocket protocol. No translation layer needed. |
| ~~Stock firmware protocol~~ | ~~CMD text protocol (`CMD\|func\|data\|$`)~~ | ~~Built into stock firmware.~~ Removed — CMD protocol and stock paths deleted. |
| Browser protocol | WebSocket + JSON | Clean, decoupled from firmware protocol. |
| Connection loss | Heartbeat + retry with backoff | 5s heartbeat timeout triggers safe stop (engage latch released). Reconnect retries with backoff. |

## Engage Switch (April 2026)

The lifecycle FSM (multi-state BOOTING/IDLE/ACTIVE/RESTING/LOW_BATTERY) was replaced by a simple two-state engage model. `cmd_engage {enabled:true/false}` is the sole operator switch. Battery cutoff latches until reboot; heartbeat detach is recoverable. The old `lifecycle.cpp`/`lifecycle.h` were deleted.

## Firmware / Algorithms

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Gait engine | C++ gait with IK foot-position pipeline, hip/knee amplitude, frequency, phase | Full parametric control. PID balance, soft-start servos. Tested via PlatformIO test suite. |
| ~~Stock gait engine~~ | ~~`_dog.move(speed, direction)` only~~ | ~~Stock firmware's gait.~~ Removed — stock firmware deleted. |
| Behavior model | Composable layers | Balance runs as always-on layer; remote, patrol, and scan stack on top. Not exclusive modes |
| Wall detection | Chain-based mesh + DBSCAN/PCA fallback | Point cloud chains are primary (vertices become mesh). DBSCAN clustering + PCA line fitting as fallback for sparse data |

## Patrol / Navigation

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Navigation | Dead reckoning + IMU heading with waypoints | Define waypoints as (x, y, heading) in local frame; good enough for short routes |
| Future navigation | Sensor-based room mapping | Ultrasonic scan map + wall detection to build spatial model; obstacle-aware path planning |

## Web UI

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Remote controls | On-screen D-pad | Simple, works on mobile and desktop; discrete directions sufficient |
| Visualization | Three.js 3D dog model + 2D scan map | 3D model shows pose/gait in real-time, 2D canvas shows ultrasonic map with walls |
| Module system | Vanilla ES modules (no build step) | `web/modules/` (ws, controls, map, panels) + `web/dog3d/` (model, gait, camera, sonar, walls, overlay) |
| Web serving | Python serves everything | Single process serves static files + WebSocket. No build step, no extra dependencies |

## Development

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Testing strategy | Mock transport + pure-Python sim + real hardware | Mock for unit tests, sim for physics/mapping, serial/WiFi for integration |
| Firmware testing | PlatformIO test suite | Kinematics, balance PID, gait parameter sweep, pose validation — all run on host |
