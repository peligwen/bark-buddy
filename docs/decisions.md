# Design Decisions — Milestone 1

Captured during project scoping (April 2026).

## Communication

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Transport | Serial first, WiFi later | USB serial for early dev; abstract transport layer so WiFi swaps in without protocol changes |
| Protocol pattern | Full bidirectional tagged messages | Both sides send anytime, type-tagged JSON. Mirrors WebSocket pattern on browser side |
| Connection loss | Auto-reconnect with freeze | Firmware freezes servos on link drop; both sides attempt reconnection; resume on restore |

## Firmware / Algorithms

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Gait engine | Parametric gait / IK | Build on MechDog's existing IK locomotion; parameterize step height, stride, speed |
| Balance | Complementary filter + PID | Fuse accel + gyro for stable tilt estimate; PID loop drives continuous leg corrections |
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
| Testing strategy | Mock serial for host, real hardware for firmware | Python mock stub enables host/web dev without dog; physics sim is a late goal |
