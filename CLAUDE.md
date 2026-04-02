# Bark-Buddy

Firmware for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Current Milestone: Thin MVP

Validate full architecture with minimal working versions of:
- **Remote Control** — Web UI → walk, turn, stop
- **Balance & Recovery** — IMU tilt correction via stock self-balance
- **Patrol** — Execute predefined waypoint sequences autonomously

## Architecture

Python host drives the stock MechDog firmware via CMD protocol over USB serial:

- **MechDog (stock firmware):** ESP32-S3, 8 PWM servos, MPU6050 IMU, CMD protocol
- **Local dev machine (Python):** Web server, behavior engine, CMD comms layer
- **Browser:** Web UI for remote control and status

Flow: Browser → WebSocket (JSON) → Python host → serial (CMD protocol) → stock firmware → servos

## Key Design Decisions

- **Firmware:** Use stock firmware + CMD protocol (no custom firmware for MVP)
- **Transport:** USB serial at 115200 baud
- **Dog protocol:** Stock CMD text format (`CMD|func|data|$`)
- **Browser protocol:** WebSocket + JSON (decoupled from CMD format)
- **Balance:** Stock self-balance toggle (CMD|1|3|1)
- **Behaviors:** Composable layers — balance runs beneath remote or patrol
- **Patrol (MVP):** Dead reckoning + IMU heading with waypoints
- **Web UI:** D-pad controls, 2D gauges, Python serves everything
- **Testing:** Mock serial stub for host dev, real hardware for integration

See `docs/decisions.md` for full rationale.

## Project Layout

- `firmware/` — C/C++ reference for future custom firmware (not used for MVP)
- `host/` — Python host application
  - `server.py` — web server + WebSocket
  - `comms.py` — CMD protocol layer + serial transport
  - `mock_serial.py` — mock transport for dev without hardware
  - `behaviors/` — remote.py, balance.py, patrol.py
- `web/` — static web UI (index.html, app.js, style.css)
- `docs/` — architecture, decisions, implementation plan, protocol spec

## Conventions

- Host: Python 3.11+, asyncio, pyserial-asyncio
- Protocol: Stock CMD text over serial (see docs/protocol.md)
- Web: Vanilla HTML/CSS/JS, no framework

## MechDog CMD Protocol Quick Reference

- Motion: `CMD|3|<1-8>|$` (1=stop, 2=stand, 3=fwd, 4=back, 5=left, 6=right)
- Balance: `CMD|1|3|<0/1>|$` (toggle self-balance)
- IMU: `CMD|5|$` → `CMD|5|<x>|<y>|$`
- Battery: `CMD|6|$` → `CMD|6|<mv>|$`
- Actions: `CMD|2|1|<1-5>|$` (wave, stretch, turn, sit, lie)

## Workflow Guidelines

- **Commit early and often.** Small, focused commits. Push after each logical unit of work — don't batch up large changes.
- **If `git push` fails once, immediately fall back to MCP `push_files`.** Do not retry git push with exponential backoff — it wastes time and the proxy may be temporarily down.
- **Minimize file re-reads.** If you just wrote a file, use its content from memory for MCP push — don't re-read it. Only re-read if a linter/formatter may have changed it.
- **Avoid rewriting all docs at once.** When pivoting architecture, update one doc at a time, commit, push. Don't batch 9 file rewrites into a single push.
- **Keep MCP `push_files` calls small.** If pushing more than 4-5 files, split into multiple commits/pushes to avoid token-heavy operations and timeouts.
- **Ask before major pivots.** If research reveals the planned approach is wrong (e.g., stock firmware already does what we planned to build), confirm the pivot with the user before rewriting everything.

## Out of Scope (Milestone 1)

Camera/vision, object carrying, runtime AI, mobile app, Pi integration, advanced gaits, obstacle avoidance, SLAM, custom firmware.
