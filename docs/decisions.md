# Design Decisions

Captured during project scoping and evolution (April 2026).

## Firmware Approach

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Primary firmware | Custom C++ via PlatformIO | Full control over gait engine, servo PWM, sensor streaming, and balance. JSON protocol is cleaner than stock CMD. WiFi TCP enables wireless operation |
| Stock firmware role | Fallback and bootstrapping | Used for pin discovery (REPL introspection), initial hardware validation, and as a fallback when custom firmware isn't flashed. Not the long-term path |
| Servo pin verification | `PINS_VERIFIED` gate in config.h | Custom firmware won't drive servos until GPIO pins are confirmed via stock REPL introspection. Safety guard against wrong-pin damage |

## Communication

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Primary transport | WiFi TCP (port 9000) with JSON/NDJSON | Wireless operation, structured protocol, bidirectional telemetry streaming. Custom firmware listens, host connects |
| Debug transport | USB serial (115200 baud) | Available on both firmware paths. Used for development, log inspection, and flashing |
| Fallback transport | Serial REPL / WiFi WebREPL | For stock firmware only — REPL command injection over serial or WebREPL. Bootstrapping and fallback |
| Custom firmware protocol | JSON/NDJSON (newline-delimited) | ArduinoJson on firmware side. Structured, extensible, same format as WebSocket protocol. No translation layer needed |
| Stock firmware protocol | CMD text protocol (`CMD\|func\|data\|$`) | Built into stock firmware. Requires REPL command translation on host side |
| Browser protocol | WebSocket + JSON | Clean, decoupled from firmware protocol. Same format regardless of which firmware path is active |
| Connection loss | Heartbeat + retry with backoff | Custom FW: 5s heartbeat timeout triggers safe stop. Stock FW: 500ms serial timeout, exponential backoff retry |

## Firmware / Algorithms

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Custom gait engine | C++ gait with hip/knee amplitude, frequency, phase | Full parametric control. PID balance, soft-start servos, idle timeout. Tested via PlatformIO test suite |
| Stock gait engine | `_dog.move(speed, direction)` only | Stock firmware's gait works but is opaque. Don't modify `set_gait_params()` or `homeostasis()` — breaks defaults |
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
| Testing strategy | Mock transport + PyBullet sim + real hardware | Mock for unit tests, PyBullet for physics/mapping, serial/WiFi for integration |
| Firmware testing | PlatformIO test suite | Kinematics, balance PID, gait parameter sweep, pose validation — all run on host |
