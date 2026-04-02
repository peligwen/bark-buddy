# Design Decisions — Milestone 1

Captured during project scoping (April 2026).

## Firmware Approach

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Firmware | Use stock firmware + CMD protocol | MechDog's ESP32-S3 ships with working motion, balance, and IMU. No need to flash custom firmware for MVP. Python host sends CMD strings over serial |
| Custom firmware | Deferred to post-MVP | Full servo/gait control available later via Hiwonder Arduino libraries (MechDog_Arduino, MPU6050) |

## Communication

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Transport | USB serial (115200 baud) | Direct UART connection to ESP32-S3. WiFi deferred |
| Dog protocol | Stock CMD text protocol | `CMD|func|data|$` format. Already implemented on the dog |
| Browser protocol | WebSocket + JSON | Clean decoupled protocol for the web UI side |
| Connection loss | Retry with backoff | Detect serial timeout (500ms), retry open with exponential backoff |

## Firmware / Algorithms

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Gait engine | Stock firmware motion commands | Forward/backward/turn/shift via CMD|3. Custom parametric gait/IK deferred to custom firmware phase |
| Balance | Stock self-balance toggle | CMD|1|3|1 enables built-in MPU6050 balance. Custom complementary filter + PID deferred |
| Behavior model | Composable layers | Balance runs as always-on layer; remote and patrol stack on top. Not exclusive modes |

## Patrol / Navigation

| Decision | Choice | Rationale |
|----------|--------|-----------|
| MVP navigation | Dead reckoning + IMU heading with waypoints | Define waypoints as (x, y, heading) in local frame; good enough for short routes |
| Future navigation | Sensor-based room mapping | Add sensors to map environment; place waypoints on a spatial map of the room |

## Web UI

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Remote controls | On-screen D-pad | Simple, works on mobile and desktop; discrete directions sufficient for MVP |
| Telemetry display | 2D gauges + status bar (3D model later) | Gauges for pitch/roll, status bar for mode/connection. 3D orientation model is a late goal |
| Web serving | Python serves everything | Single process serves static files + WebSocket. No build step, no extra dependencies |

## Development

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Testing strategy | Mock serial for host, real hardware for integration | Python mock stub enables host/web dev without dog; physics sim is a late goal |
