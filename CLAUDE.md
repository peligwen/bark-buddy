# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

**Current work:** IK-based gait pipeline (foot-position IK, body transforms, active balance, stride config)
**Next milestone:** SLAM-based localization, composite mapping, waypoint navigation UI

## Core Docs

| Doc | Contents |
|---|---|
| [docs/overview.md](docs/overview.md) | Mission, kernel scope, deferred work |
| [docs/architecture.md](docs/architecture.md) | Components, layers, data flow, transport, mock parity |
| [docs/protocol.md](docs/protocol.md) | All commands and telemetry with field specs |
| [docs/hardware.md](docs/hardware.md) | Pins, servos, IMU, I2C buses, battery thresholds |
| [docs/design-principles.md](docs/design-principles.md) | Prescriptive rules governing future growth |

## CLI

- `bark` — auto-detect hardware (USB → mDNS → exit with guidance)
- `bark mock` — build + launch mock firmware, connect server to it
- `bark flash` — flash firmware via PlatformIO
- `bark test` — run firmware unit tests
- `bark kill` — kill any running bark server

## Layout

- `firmware/` — C++ firmware (PlatformIO, ESP32); `src/` motion + hardware, `include/` headers, `mock/` platform shims + physics model, `test/` unit tests + Makefile
- `host/` — Python host; `server.py` web server + WebSocket, `firmware_transport.py` sole transport, `comms.py` Transport ABC, `dead_reckoning.py` odometry mixin, `behaviors/balance.py` balance layer
- `web/` — static UI (ES modules); `dog3d/` Three.js 3D viz, `modules/` ws/controls/panels/diag
- `bark_cli.py` — CLI entry point
- `docs/` — five core docs + attic.md + superpowers/

## Conventions

- Firmware: C++ (PlatformIO), ArduinoJson, ESP32-WROOM-32D (D0WD chip)
- Mock firmware build: `cd firmware/test && make bark-mock` (clang++, C++17, MOCK_FIRMWARE=1)
- Host: Python 3.11+, asyncio, pyserial-asyncio, websockets
- Web: Vanilla HTML/CSS/JS (ES modules), Three.js r128 via CDN
- Transport: auto-detected (USB serial → hardware, mDNS → WiFi hardware). Override: `--fw-tcp HOST[:PORT]`.

## Workflow

- Commit early and often. Small, focused commits.
- Ask before major pivots.
- `bark test` and `make bark-mock` must pass before merging.
- Mock firmware: `cd firmware/test && make bark-mock` to build; `./bark-mock --tcp-port 9001` to run standalone.
