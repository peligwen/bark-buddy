# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog. The dog is a WiFi actuator with a camera; perception runs on a GPU host on the same network.

**Current work:** Prove the camera — stop-and-snap capture, offline reconstruction, go/no-go (runbook: `docs/superpowers/plans/2026-11-10-prove-the-camera.md`)
**Next milestone:** Simulated world in the mock, then live map, waypoint navigation UI (roadmap in `docs/overview.md`)

## Core Docs

| Doc | Contents |
|---|---|
| [docs/overview.md](docs/overview.md) | Mission, hardware and kernel scope, perception roadmap |
| [docs/architecture.md](docs/architecture.md) | Components, layers, data flow, transport, mock parity |
| [docs/protocol.md](docs/protocol.md) | All commands and telemetry with field specs |
| [docs/hardware.md](docs/hardware.md) | Pins, servos, IMU, I2C buses, battery thresholds |
| [docs/design-principles.md](docs/design-principles.md) | Prescriptive rules governing future growth |

## CLI

- `bark` — auto-detect hardware (USB → mDNS → exit with guidance)
- `bark mock` — build + launch mock firmware, connect server to it
- `bark flash` — flash firmware via PlatformIO
- `bark flash camera` — flash the vision module firmware
- `bark --cam-url URL --record [DIR]` — also save a keyframe each time the dog stops
- `bark reconstruct [RUN] --cam-height-mm N` — metric map and report from a capture run (GPU)
- `bark test` — run firmware unit tests
- `bark kill` — kill any running bark server

## Layout

- `firmware/` — C++ firmware (PlatformIO, ESP32); `src/` motion + hardware, `include/` headers, `mock/` platform shims + physics model, `test/` unit tests + Makefile
- `camera/` — C++ firmware for the ESP32-S3 vision module (separate PlatformIO project)
- `host/` — Python host; `server.py` web server + WebSocket, `dog/` transport package (Dog class, io, discover), `camera.py` vision module client, `ota.py` firmware OTA, `lock.py` WS control lock, `behaviors/` balance layer + capture recorder, `perception/` offline reconstruction
- `web/` — static UI (ES modules); `dog3d/` Three.js 3D viz, `modules/` ws/controls/panels/diag
- `bark_cli.py` — CLI entry point
- `docs/` — five core docs + attic.md + superpowers/

## Conventions

- Firmware: C++ (PlatformIO, arduino-esp32 3.3.8), ArduinoJson, ESP32-WROOM-32D (D0WD chip)
- Camera firmware: C++ (PlatformIO, espressif32 8.0.0 / arduino-esp32 3.3.12), ESP32-S3 with OPI PSRAM
- Mock firmware build: `cd firmware/test && make bark-mock` (clang++, C++17, MOCK_FIRMWARE=1; needs `cd firmware && pio pkg install` once for ArduinoJson)
- Host: Python 3.11+, asyncio, pyserial-asyncio, websockets
- Perception: numpy on the host; torch + Depth Anything 3 only inside `host/perception/da3.py`
- Web: Vanilla HTML/CSS/JS (ES modules), Three.js r128 via CDN
- Transport: auto-detected (USB serial → hardware, mDNS → WiFi hardware). Override: `--fw-tcp HOST[:PORT]`.

## Workflow

- Commit early and often. Small, focused commits.
- Ask before major pivots.
- `bark test`, `make bark-mock`, and `pytest host/tests` must pass before merging.
- Mock firmware: `cd firmware/test && make bark-mock` to build; `./bark-mock --tcp-port 9001` to run standalone.
