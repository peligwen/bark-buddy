# Phase 2 — Core Docs Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish the five authoritative `docs/` files, `docs/attic.md`, and a rewritten `CLAUDE.md` that together serve as the complete source of truth for the kernel.

**Architecture:** Each doc is written in full and committed separately. No doc duplicates another. `CLAUDE.md` becomes a ≤100-line pointer file; all protocol/hardware/architecture detail lives in the five core docs.

**Tech Stack:** Markdown, git. No code changes.

---

## File Structure

Files created or replaced:

| File | Action | Responsibility |
|------|--------|----------------|
| `docs/overview.md` | Create | Mission, kernel scope, deferred work, pointer map to other four docs |
| `docs/architecture.md` | Create | Components, layers, data flow, transport boundary, behavior pattern |
| `docs/protocol.md` | Create | Authoritative wire format: every command and telemetry message |
| `docs/hardware.md` | Create | Pins, peripherals, IMU axes, servo layout, pose tables |
| `docs/design-principles.md` | Create | Prescriptive rules governing the project |
| `docs/attic.md` | Create | One-line-per-file index of code removed in Phase 1 |
| `CLAUDE.md` | Rewrite | ≤100-line pointer to core docs + minimal workflow rules |

---

### Task 0: Remove untracked attic remnants

Some files deleted from git tracking in Phase 1 may still exist in the working tree as untracked files.

**Files:**
- Delete (if present): `host/map_servos.py`, `host/probe_stock_firmware.py`, `host/sweep/`

- [ ] **Step 1: Remove untracked attic files**

```bash
rm -f host/map_servos.py host/probe_stock_firmware.py
rm -rf host/sweep/
```

- [ ] **Step 2: Verify gone**

```bash
ls host/ | grep -E "map_servos|probe_stock|sweep" && echo "STILL PRESENT" || echo "Clean"
```

Expected: `Clean`

- [ ] **Step 3: Commit if anything changed**

```bash
git status
# Only commit if git shows changes (these were untracked so git status should show clean)
```

If `git status` is clean, no commit needed — proceed to Task 1.

---

### Task 1: docs/overview.md

**Files:**
- Create: `docs/overview.md`

- [ ] **Step 1: Write the file**

```bash
cat > docs/overview.md << 'EOF'
# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog.

## Mission

Give the MechDog autonomous navigation on a single stock hardware platform — no Raspberry Pi, no extra sensors. The operator sets goals from a browser UI; the dog handles navigation using its continuously updated world model.

## Kernel Scope

The kernel is the minimal set of components that must work before any autonomous behavior is possible:

- **Firmware** — motion engine on ESP32: IK gait, servo control, IMU/sonar telemetry, active balance, command dispatch
- **Mock firmware** — the same firmware C++ compiled as a native binary for desktop dev
- **Host** — Python server: transport to firmware, WebSocket to browser, composable behavior layers
- **Web** — primary control UI: D-pad, 3D dog view, diagnostics
- **CLI** — `bark` entry point

## Deferred Work

Not in scope until the kernel is stable:

- SLAM-based localization (IMU + sonar scan matching)
- Composite multi-scan mapping
- Waypoint navigation UI
- Camera/vision (ESP32-S3 integration)
- Semantic understanding / LLM goals
- Multi-platform robot support

## Core Docs

| Doc | Contents |
|-----|----------|
| [architecture.md](architecture.md) | Components, layers, data flow, transport boundary |
| [protocol.md](protocol.md) | Authoritative wire format: every command and telemetry message |
| [hardware.md](hardware.md) | Pins, peripherals, IMU axes, servo layout |
| [design-principles.md](design-principles.md) | Rules that govern how the project grows |
| [attic.md](attic.md) | Index of code removed in Phase 1; lives in `pre-reshape` branch |
EOF
```

- [ ] **Step 2: Verify key sections exist**

```bash
grep -c "^## " docs/overview.md
```

Expected: `4` (Mission, Kernel Scope, Deferred Work, Core Docs)

- [ ] **Step 3: Commit**

```bash
git add docs/overview.md
git commit -m "docs: add overview.md — mission, kernel scope, deferred work"
```

---

### Task 2: docs/architecture.md

**Files:**
- Create: `docs/architecture.md`

- [ ] **Step 1: Write the file**

```bash
cat > docs/architecture.md << 'EOF'
# Architecture

## Components

```
Browser (vanilla JS + Three.js)
    │ WebSocket JSON
    ▼
Python host (aiohttp)
    ├── server.py             — WS session, command routing, telemetry relay, static files
    ├── firmware_transport.py — sole transport to firmware
    └── behaviors/
        ├── balance.py        — active pitch/roll compensation via cmd_transform
        └── button_engage.py  — K1 button → engage/disengage
    │ NDJSON over TCP (WiFi or USB serial)
    ▼
Firmware (C++ / ESP32-WROOM-32D)
    ├── command_handlers.cpp  — JSON dispatch table
    ├── gait.cpp, servos.cpp  — IK gait engine, servo control
    ├── imu.cpp, sonar.cpp    — sensor drivers
    └── sensor_task.cpp       — FreeRTOS sensor loop (50 Hz IMU, 20 Hz sonar)
```

**Mock firmware** (`firmware/test/bark-mock`) — the same C++ source compiled as a native binary
with platform shims in `firmware/mock/`. The host connects to it identically to real hardware
via TCP on localhost. This is the primary dev loop.

## Layer Responsibilities

| Layer | Owns | Does NOT own |
|-------|------|--------------|
| Firmware | Motion, hardware I/O, sensor streaming, command execution | Coordination, session state |
| Host | Behavior composition, WebSocket session, telemetry relay | Servo math, sensor fusion |
| Web | Presentation, user input, 3D visualization | Business logic, transport |

No layer reaches past its neighbor. The browser never speaks TCP; firmware never speaks WebSocket.

## Data Flow

**Command path (browser → firmware):**
1. Browser sends JSON over WebSocket to `server.py`
2. `server.py` routes to a behavior or calls `FirmwareTransport.send_json()`
3. Firmware receives NDJSON, dispatches via handler table, actuates servos

**Telemetry path (firmware → browser):**
1. Firmware sensor task streams NDJSON (IMU 50 Hz, sonar 20 Hz, status 1 Hz, battery 1 Hz)
2. `FirmwareTransport` parses and fires Python callbacks
3. `server.py` relays to all connected WebSocket clients as JSON

## Transport Boundary

`FirmwareTransport.send_json(msg: dict)` is the **only** host→firmware command path.
No text protocols, no direct serial writes, no alternate transports.

Auto-detection order on startup: USB serial JSON ping → mDNS `_mechdog._tcp` → exit with guidance.
Override: `--fw-tcp HOST[:PORT]`.

## Composable Behavior Pattern

Behaviors take a `FirmwareTransport` instance and run as asyncio tasks alongside the main
WebSocket loop. They register telemetry callbacks and send commands through the transport.

```python
self._balance = BalanceLayer(transport)           # reads IMU, writes cmd_transform
self._button_engage = ButtonEngageBehavior(...)   # reads K1 button, writes cmd_engage
```

When the transport is replaced (e.g., after OTA flash), `server.py` re-creates all behaviors
against the new transport via `_replace_transport()`.

## Mock Firmware Internals

`firmware/mock/` contains host-friendly replacements for Arduino/ESP-IDF headers, resolved via
include-path prepend. IMU and sonar drivers are replaced at link time:

- `imu_mock.cpp` — reads from physics model (`physics.cpp`)
- `sonar_mock.cpp` — reads from physics model
- `physics.cpp` — servo duty → FK via `pulses_to_foot()` → pitch/roll from foot heights → low-pass filter → gyro derivative → gravity projection
EOF
```

- [ ] **Step 2: Verify key sections**

```bash
grep -c "^## " docs/architecture.md
```

Expected: `5` (Components, Layer Responsibilities, Data Flow, Transport Boundary, Composable Behavior Pattern, Mock Firmware Internals — actually 6)

```bash
grep "^## " docs/architecture.md
```

Expected output includes: Components, Layer Responsibilities, Data Flow, Transport Boundary, Composable Behavior Pattern, Mock Firmware Internals

- [ ] **Step 3: Commit**

```bash
git add docs/architecture.md
git commit -m "docs: add architecture.md — components, layers, data flow, transport boundary"
```

---

### Task 3: docs/protocol.md

**Files:**
- Create: `docs/protocol.md`

- [ ] **Step 1: Write the file**

```bash
cat > docs/protocol.md << 'EOF'
# Protocol

All messages are JSON, newline-delimited (NDJSON) over TCP or USB serial.
The host sends commands; firmware streams telemetry and acks.
Maximum message size: 512 bytes (firmware JSON parse buffer).

---

## Commands (host → firmware)

### cmd_move

Move in a direction using the IK gait engine.

```json
{"type": "cmd_move", "direction": "forward", "speed": 1.0}
```

| Field | Type | Values |
|-------|------|--------|
| direction | string | `"forward"` \| `"backward"` \| `"left"` \| `"right"` \| `"rotate_left"` \| `"rotate_right"` \| `"stop"` |
| speed | float | 0.0–1.0; scales gait frequency |

### cmd_stand

Stop gait and return to standing pose.

```json
{"type": "cmd_stand"}
```

### cmd_balance

Enable or disable active balance (pitch/roll compensation via IMU feedback).

```json
{"type": "cmd_balance", "enabled": true}
```

### cmd_engage

Engage or disengage all servos. Engage ramps from lying-down to standing over 2 s.
Disengage ramps back to rest pose and then cuts PWM.

```json
{"type": "cmd_engage", "enabled": true}
```

Rejected (ack ok=false) when `battery_cutoff` is latched.

### cmd_servo

Direct servo pulse width, bypassing IK. Works only when engaged.

```json
{"type": "cmd_servo", "index": 0, "pulse_us": 1500}
```

| Field | Type | Range |
|-------|------|-------|
| index | int | 0–7 |
| pulse_us | int | 500–2500 μs |

### cmd_offset

Set a persistent trim offset for a servo. Saved to NVS (survives reboot).

```json
{"type": "cmd_offset", "index": 0, "offset_us": 0}
```

| Field | Type | Range |
|-------|------|-------|
| index | int | 0–7 |
| offset_us | int | −500 to +500 μs |

### cmd_servo_pin

Reassign a servo index to a different GPIO at runtime.

```json
{"type": "cmd_servo_pin", "index": 0, "pin": 26}
```

Firmware detaches old GPIO, attaches new GPIO to LEDC channel, writes current pulse.
If the new pin is already assigned to another index, they swap automatically.

### cmd_transform

Apply a body-frame transform on top of standing pose.
All fields are optional; omitted fields default to 0.

```json
{"type": "cmd_transform", "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| roll, pitch, yaw | degrees | body rotation |
| x, y, z | mm | body translation |

### cmd_gait_params

Set gait stride parameters. Changes take effect on next gait cycle.

```json
{"type": "cmd_gait_params", "stride_length": 20, "stride_height": 10, "speed": 1.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| stride_length | mm | foot swing distance per step |
| stride_height | mm | foot lift height per step |
| speed | 0.0–1.0 | scales gait frequency |

### cmd_led

Set onboard LED color. `led: 0` is the onboard blue LED (GPIO 18, active-LOW).

```json
{"type": "cmd_led", "led": 0, "r": 0, "g": 15, "b": 0}
```

r, g, b range: 0–255. Firmware applies `LED_BRIGHTNESS` scale (default 40/255).

### cmd_buzzer

Play a tone on the piezo buzzer.

```json
{"type": "cmd_buzzer", "freq_hz": 2400, "duration_ms": 200}
```

`freq_hz: 0` stops immediately. `duration_ms: 0` plays indefinitely until next cmd_buzzer.

### cmd_gpio

GPIO expansion. Valid pins: 32, 33, 1 (UART TX), 3 (UART RX).

```json
{"type": "cmd_gpio", "op": "mode",        "pin": 32, "mode": "input"}
{"type": "cmd_gpio", "op": "write",       "pin": 32, "value": 1}
{"type": "cmd_gpio", "op": "read",        "pin": 32}
{"type": "cmd_gpio", "op": "analog",      "pin": 32}
{"type": "cmd_gpio", "op": "subscribe",   "pin": 32, "interval_ms": 100}
{"type": "cmd_gpio", "op": "unsubscribe", "pin": 32}
```

Read/analog results arrive as `telem_gpio`. Subscribe sends `telem_gpio` at the given interval.

### cmd_i2c

I2C bus operations. `bus: 1` = primary (SDA GPIO 22, SCL GPIO 23); `bus: 2` = secondary (SDA GPIO 19, SCL GPIO 13).

```json
{"type": "cmd_i2c", "op": "scan",  "bus": 1}
{"type": "cmd_i2c", "op": "read",  "bus": 1, "addr": 119, "reg": 0, "len": 2}
{"type": "cmd_i2c", "op": "write", "bus": 1, "addr": 119, "reg": 0, "data": [1, 2]}
```

Results arrive as `telem_i2c`.

### ping

No-op keepalive. Firmware replies with `ack`.

```json
{"type": "ping"}
```

---

## Telemetry (firmware → host)

### telem_imu

50 Hz. Complementary-filter fused orientation.

```json
{"type": "telem_imu", "pitch": 2.1, "roll": -0.3, "yaw": 45.0,
 "ax": 0.02, "ay": -0.01, "az": 9.81, "gx": 0.1, "gy": -0.2, "gz": 0.0}
```

| Field | Unit | Description |
|-------|------|-------------|
| pitch | degrees | positive = nose up |
| roll | degrees | positive = left side up |
| yaw | degrees | integrated gyro; drifts; not suitable for localization |
| ax, ay, az | m/s² | body-frame acceleration |
| gx, gy, gz | deg/s | body-frame angular rate |

### telem_sonar

20 Hz.

```json
{"type": "telem_sonar", "distance_mm": 250}
```

### telem_battery

1 Hz.

```json
{"type": "telem_battery", "voltage_mv": 7400, "pct": 80, "low": false}
```

| Field | Description |
|-------|-------------|
| voltage_mv | ADC-derived millivolts (4:1 divider, GPIO 34) |
| pct | 0–100, mapped from 6000–8400 mV |
| low | true when below 6700 mV |

Battery cutoff latches at 6400 mV (3.2 V/cell for 2S LiPo). Requires reboot to clear.

### telem_status

1 Hz. Overall system state.

```json
{"type": "telem_status", "engaged": true, "ramping": false,
 "balance": true, "battery_cutoff": false, "mode": "remote"}
```

| Field | Description |
|-------|-------------|
| engaged | servos powered and at standing pose |
| ramping | engage or disengage ramp in progress |
| balance | active balance enabled |
| battery_cutoff | latched low-battery cutoff; servos off; requires reboot |
| mode | always `"remote"` post Phase 1 |

### telem_button

Physical button K1 (GPIO 5) events.

```json
{"type": "telem_button", "event": "press"}
```

`event`: `"press"` | `"release"` | `"long_press"` (≥ 1000 ms)

### telem_gpio

GPIO subscription tick or read result.

```json
{"type": "telem_gpio", "pin": 32, "digital": 1, "analog": -1}
```

`analog`: ADC reading in raw counts, or -1 if not requested / pin not ADC-capable.

### telem_i2c

I2C operation result.

```json
{"type": "telem_i2c", "bus": 1, "op": "scan",  "addrs": [106, 119]}
{"type": "telem_i2c", "bus": 1, "op": "read",   "data": [0, 15]}
{"type": "telem_i2c", "bus": 1, "op": "write",  "ok": true}
```

### telem_servo_pins

Current servo-index → GPIO mapping. Sent on client connect and after `cmd_servo_pin`.

```json
{"type": "telem_servo_pins", "pins": [25, 26, 27, 14, 16, 17, 15, 2]}
```

Array index matches servo index (0 = FL_hip … 7 = RR_knee).

### ack

Sent after every command. `ok: false` means the command was rejected.

```json
{"type": "ack", "ref_type": "cmd_move",   "ok": true}
{"type": "ack", "ref_type": "cmd_engage", "ok": false, "error": "battery_cutoff"}
```

Common rejection reasons: `"battery_cutoff"`, `"not_engaged"`, `"bad_index"`, `"bad_pin"`.
EOF
```

- [ ] **Step 2: Verify all commands and telemetry types are present**

```bash
grep "^### " docs/protocol.md
```

Expected (13 commands + 9 telemetry = 22 sections):
```
### cmd_move
### cmd_stand
### cmd_balance
### cmd_engage
### cmd_servo
### cmd_offset
### cmd_servo_pin
### cmd_transform
### cmd_gait_params
### cmd_led
### cmd_buzzer
### cmd_gpio
### cmd_i2c
### ping
### telem_imu
### telem_sonar
### telem_battery
### telem_status
### telem_button
### telem_gpio
### telem_i2c
### telem_servo_pins
### ack
```

- [ ] **Step 3: Commit**

```bash
git add docs/protocol.md
git commit -m "docs: add protocol.md — authoritative wire format for all commands and telemetry"
```

---

### Task 4: docs/hardware.md

**Files:**
- Create: `docs/hardware.md`

- [ ] **Step 1: Write the file**

```bash
cat > docs/hardware.md << 'EOF'
# Hardware

Reference: `firmware/include/config.h` is the authoritative source for pin numbers and timing constants.
Manufacturer PDFs: `docs/manufacturer/`.

## Main Controller

**ESP32-WROOM-32D** (D0WD chip, dual-core Xtensa LX6, 240 MHz, 4 MB flash).
The separate ESP32-S3 on the vision module is for the camera only and is not connected.

## Servo Layout

Eight PWM servos controlled via ESP32 LEDC (14-bit, 50 Hz). Servo index `i` → LEDC channel `i`.
Default GPIO assignments from `firmware/src/servos.cpp:SERVO_PINS[8]`.

| Index | Name | Default GPIO | Standing (μs) | Rest (μs) | Polarity |
|-------|------|--------------|---------------|-----------|----------|
| 0 | FL_hip | 25 | 2096 | 1800 | auto (+1) |
| 1 | FL_knee | 26 | 1621 | 1500 | auto (+1) |
| 2 | FR_hip | 27 | 2170 | 1870 | auto (+1) |
| 3 | FR_knee | 14 | 1611 | 1500 | auto (+1) |
| 4 | RL_hip | 16 | 904 | 1202 | auto (−1) |
| 5 | RL_knee | 17 | 1379 | 1500 | auto (−1) |
| 6 | RR_hip | 15 | 830 | 1165 | override +1 |
| 7 | RR_knee | 2 | 1389 | 1500 | override −1 |

**Polarity:** derived from standing pulse relative to 1500 μs (>1500 → +1, <1500 → −1).
RR_hip/RR_knee are physically mounted inverted; polarity overrides correct the IK.

**Lying-down pose:** all servos at 1500 μs (safe neutral on boot and shutdown).

**Pulse range:** 500–2500 μs. Center: 1500 μs.
Softstart: 2000 ms ramp from lying-down to standing on engage.
Disengage ramp: 1500 ms ramp to rest pose, then PWM cut.

GPIO assignments are mutable at runtime via `cmd_servo_pin`. Changes are not persisted.

## I2C Buses

| Bus | Instance | SDA | SCL | Devices |
|-----|----------|-----|-----|---------|
| 1 | Wire | GPIO 22 | GPIO 23 | QMI8658 IMU (0x6A), ultrasonic sonar (0x77) |
| 2 | Wire1 | GPIO 19 | GPIO 13 | J4 expansion connector |

Both buses at 400 kHz. I2C address 0x6A confirmed for QMI8658 (not 0x6B).

## IMU Axis Convention

QMI8658 at I2C address 0x6A on bus 1. Output after complementary filter:

| Axis | Positive direction |
|------|-------------------|
| Pitch | Nose up |
| Roll | Left side up |
| Yaw | Integrated gyro — drifts; not used for localization |

Accelerometer: m/s² body frame. Gyroscope: deg/s body frame. IMU data-ready interrupt: GPIO 35 (input-only).

## Other Peripherals

| Peripheral | GPIO | Notes |
|------------|------|-------|
| Battery ADC | 34 | ~4:1 resistor divider; cutoff at 6400 mV (3.2 V/cell, 2S LiPo) |
| Button K1 | 5 | Active-LOW, 10 kΩ pullup; debounce 20 ms; long-press ≥ 1000 ms |
| IMU interrupt | 35 | QMI8658 INT2 data-ready; input-only pin |
| Onboard LED | 18 | Active-LOW; `cmd_led {led:0}` controls it; brightness capped at 40/255 |
| Buzzer | 21 | NPN transistor drive; LEDC channel 9 |

## Auxiliary GPIO

J5 expansion header: GPIO 32, 33. UART0 header: GPIO 1 (TX), 3 (RX).
All four are accessible via `cmd_gpio`. Enabled by `AUX_GPIO_ENABLED=1` (default on).

## Auxiliary Servo Ports

Expansion connectors SERVO7/9/11 mapped to GPIO 15, 0, 12.
Disabled by default (`AUX_SERVOS_ENABLED=0`).
GPIO 0 is a boot-mode strapping pin (must be HIGH at power-on).
GPIO 12 is a flash-voltage strapping pin (series 5.1 kΩ resistor present).

## LEDC PWM Configuration

14-bit resolution: 16384 ticks per 20 ms period (50 Hz).

| Channel | Use |
|---------|-----|
| 0–7 | Main servos (indices 0–7) |
| 8 | Pin probe (servo-pin verification) |
| 9 | Buzzer |
| 10–12 | Aux servos (if enabled) |

## Battery Thresholds

| Level | Voltage | Behavior |
|-------|---------|----------|
| Low | < 6700 mV | LED blinks 1 Hz |
| Critical | < 6500 mV | LED blinks 2 Hz |
| Cutoff | < 6400 mV | Servos detach; state latched until reboot |
| Hysteresis | 100 mV | Prevents rapid threshold crossing |

## WiFi

TCP port 9000. Credentials in `firmware/include/config_local.h` (gitignored).
Copy `config_local.h.example` → `config_local.h` and fill in SSID/password.
mDNS service name: `_mechdog._tcp`.
EOF
```

- [ ] **Step 2: Verify key sections**

```bash
grep "^## " docs/hardware.md
```

Expected:
```
## Main Controller
## Servo Layout
## I2C Buses
## IMU Axis Convention
## Other Peripherals
## Auxiliary GPIO
## Auxiliary Servo Ports
## LEDC PWM Configuration
## Battery Thresholds
## WiFi
```

- [ ] **Step 3: Commit**

```bash
git add docs/hardware.md
git commit -m "docs: add hardware.md — servo layout, I2C buses, IMU axes, peripherals"
```

---

### Task 5: docs/design-principles.md

**Files:**
- Create: `docs/design-principles.md`

- [ ] **Step 1: Write the file**

```bash
cat > docs/design-principles.md << 'EOF'
# Design Principles

Prescriptive rules. If code violates them, it's wrong — not a guideline to weigh.

## Multi-year horizon

Durable shapes over clever shortcuts. Favor small, single-purpose files and classes from
the start, not as an afterthought. Every structural decision should be one you'd still
be happy with in a year.

## Architecture boundaries

**Single transport.** `FirmwareTransport.send_json()` is the only host→firmware path.
JSON only — no text protocols, no direct serial writes.

**Layer discipline.** Firmware owns motion and hardware. Host owns coordination and
behaviors. Web owns presentation. No layer reaches past its neighbor.

**Mock parity.** Mock shares source with real firmware; diverges only at platform shims
(`firmware/mock/`) and link-time driver substitution. New drivers require a mock
implementation before kernel code uses them.

## Code discipline

**Split early at natural seams.** A class with two reasons to change is two classes.
File line count is a lagging signal; responsibility is the leading one.

**No speculative abstractions.** One-implementation ABCs, options no code sets, hooks
with no callers — delete.

**No speculative features** or defensive checks at internal boundaries. Trust internal
code; validate only at system boundaries (user input, external APIs).

**No dead code, no ghost comments.** Delete or don't.

**Comments state WHY** when non-obvious: a hidden constraint, a subtle invariant, a
workaround for a specific bug, behavior that would surprise a reader. Never narrate WHAT.

## Docs as source of truth

The five `docs/` files are the contract. If a behavior isn't described in them,
it isn't committed to.

Specs in `docs/superpowers/specs/`, plans in `docs/superpowers/plans/` — both
ephemeral, pruned when complete.

`CLAUDE.md` points to core docs; never duplicates them.

## Dev loop

Mock firmware is the primary dev loop. Real hardware for integration only.

`bark test` + `make bark-mock` must pass before commit.
EOF
```

- [ ] **Step 2: Verify key sections**

```bash
grep "^## " docs/design-principles.md
```

Expected:
```
## Multi-year horizon
## Architecture boundaries
## Code discipline
## Docs as source of truth
## Dev loop
```

- [ ] **Step 3: Commit**

```bash
git add docs/design-principles.md
git commit -m "docs: add design-principles.md — prescriptive rules for the project"
```

---

### Task 6: docs/attic.md

**Files:**
- Create: `docs/attic.md`

- [ ] **Step 1: Write the file**

All code below was removed from `main` in Phase 1 and is preserved in the `pre-reshape` branch.

```bash
cat > docs/attic.md << 'EOF'
# Attic

Code removed from `main` in Phase 1 (kernel cut). All preserved in the `pre-reshape` branch.

Format: `path — reason`

## Host

### Mapping / navigation behaviors

- `host/behaviors/scan.py` — sonar scan behavior; mapping rewrite pending SLAM
- `host/behaviors/map_store.py` — scan point storage and persistence; pending SLAM rewrite
- `host/behaviors/wall_fit.py` — RANSAC wall geometry from scan points; pending SLAM rewrite
- `host/behaviors/wall_mesh.py` — wall mesh generation for 3D view; pending SLAM rewrite
- `host/behaviors/octree.py` — 3D point cloud with decay; pending SLAM rewrite

### Calibration and sweep tooling

- `host/sweep/__init__.py` — automated servo sweep package init
- `host/sweep/__main__.py` — sweep package entry point
- `host/sweep/results.py` — sweep result storage
- `host/sweep/runner.py` — sweep orchestration and parallel execution
- `host/sweep/sampler.py` — IMU sampling during sweeps
- `host/sweep/scenarios.py` — sweep scenario definitions
- `host/sweep/scoring.py` — sweep result scoring
- `host/sweep/worker.py` — sweep worker process
- `host/capture_pose.py` — interactive servo offset capture tool; offsets now in config.h
- `host/calibrate_servos.py` — servo calibration workflow; calibration complete
- `host/servo_test.py` — manual servo test script; superseded by `bark test`
- `host/identify_servos.py` — servo pin identification tool; pins verified in config.h
- `host/apply_to_firmware.py` — apply calibration results to firmware; offsets baked in
- `host/map_servos.py` — servo mapping script (was untracked); pins verified
- `host/probe_stock_firmware.py` — stock firmware probing tool (was untracked); analysis complete

### Ad-hoc test scripts

- `host/test_mapping.py` — mapping integration test; pending SLAM rewrite
- `host/test_scan.py` — scan integration test; pending SLAM rewrite
- `host/test_wall_mesh.py` — wall mesh test; pending SLAM rewrite
- `host/test_button_engage.py` — button engage ad-hoc test; superseded by unit tests

## Web

### Secondary UIs

- `web/tuning.html` — servo offset / pin tuning UI; offset UX deferred to Phase 3 primary UI
- `web/gait.html` — gait parameter tuning UI; gait controls in primary D-pad UI

### Attic modules

- `web/modules/map.js` — 2D scan map canvas; pending SLAM rewrite
- `web/dog3d/sonar.js` — 3D sonar beam visualization; pending SLAM rewrite
- `web/dog3d/walls.js` — 3D wall mesh rendering; pending SLAM rewrite

## Docs

- `docs/architecture.md` — old; superseded by rewritten `docs/architecture.md` (Phase 2)
- `docs/protocol.md` — old; superseded by rewritten `docs/protocol.md` (Phase 2)
- `docs/hardware-schematic.md` — content absorbed into `docs/hardware.md` (Phase 2)
- `docs/decisions.md` — ad-hoc decision log; superseded by `docs/design-principles.md`
- `docs/implementation-plan.md` — original milestone plan; all milestones complete
- `docs/stock-firmware-analysis.md` — stock firmware REPL analysis; findings in hardware.md

### Completed plans

- `docs/superpowers/plans/2026-04-13-firmware-foundation.md` — complete
- `docs/superpowers/plans/2026-04-13-ik-balance-transform-gait.md` — complete
- `docs/superpowers/plans/2026-04-15-bark-cli-ui-trim.md` — complete
- `docs/superpowers/plans/2026-04-15-remove-dead-transport-code.md` — complete
- `docs/superpowers/plans/2026-04-15-startup-lifecycle.md` — complete
- `docs/superpowers/plans/2026-04-16-custom-firmware-only-refactor.md` — complete

### Superseded specs

- `docs/superpowers/specs/2026-04-13-firmware-foundation-design.md` — superseded
- `docs/superpowers/specs/2026-04-15-auto-connect-detect-update-design.md` — superseded
- `docs/superpowers/specs/2026-04-15-startup-lifecycle-design.md` — superseded
- `docs/superpowers/specs/2026-04-15-web-ui-simplification-design.md` — superseded
EOF
```

- [ ] **Step 2: Verify line count and structure**

```bash
wc -l docs/attic.md
grep "^- \`" docs/attic.md | wc -l
```

Expected: > 40 entries. Every line should have a `— reason` part.

```bash
grep "^- \`" docs/attic.md | grep -v " — " | head -5
```

Expected: no output (all entries have reasons).

- [ ] **Step 3: Commit**

```bash
git add docs/attic.md
git commit -m "docs: add attic.md — index of Phase 1 removals with reasons"
```

---

### Task 7: CLAUDE.md rewrite

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Replace CLAUDE.md with the new pointer version**

The new CLAUDE.md must be ≤ 100 lines and must not duplicate content from the five core docs.

```bash
cat > CLAUDE.md << 'EOF'
# Bark-Buddy

Semi-autonomous control system for Hiwonder MechDog robot dog. Stock hardware (no Pi/extra sensors).

## Core Docs (source of truth)

| Doc | Contents |
|-----|----------|
| [`docs/overview.md`](docs/overview.md) | Mission, kernel scope, deferred work |
| [`docs/architecture.md`](docs/architecture.md) | Components, layers, data flow, transport boundary |
| [`docs/protocol.md`](docs/protocol.md) | Every command and telemetry message with types |
| [`docs/hardware.md`](docs/hardware.md) | Pins, servo layout, IMU axes, battery thresholds |
| [`docs/design-principles.md`](docs/design-principles.md) | Prescriptive rules governing the project |

## Status

- **Current work** — IK-based gait pipeline (foot-position IK, body transforms, active balance, stride config)
- **Next milestone** — SLAM-based localization, composite mapping, waypoint navigation UI
- Complete milestones and removed code indexed in [`docs/attic.md`](docs/attic.md)

## CLI

```bash
bark           # start server (auto-detect: USB serial → mDNS → error)
bark mock      # build + launch mock firmware, start server against it
bark flash     # build + upload firmware via PlatformIO
bark test      # build + run all firmware native unit tests
bark kill      # kill any running bark server on port 8456
```

## Project Layout

```
firmware/
  src/         — C++ firmware source (ESP32)
  include/     — headers; config.h has all pin/timing constants
  mock/        — platform shims for native mock build
  test/        — unit tests (test_ik, test_transform, test_balance, …) + make bark-mock
host/
  server.py              — web server, WebSocket, telemetry relay
  firmware_transport.py  — sole transport (USB serial or TCP)
  behaviors/             — balance.py, button_engage.py
bark_cli.py              — CLI entry point
web/
  index.html, style.css, app.module.js
  modules/    — ws.js, controls.js, panels.js, diag.js
  dog3d/      — Three.js 3D visualization
docs/          — five core docs + manufacturer PDFs + superpowers/
```

## Conventions

- **Firmware:** C++ (PlatformIO), ArduinoJson, ESP32-WROOM-32D (D0WD chip)
- **Mock build:** `cd firmware/test && make bark-mock` (clang++, C++17, MOCK_FIRMWARE=1)
- **Host:** Python 3.11+, asyncio, aiohttp, pyserial-asyncio
- **Web:** Vanilla JS (ES modules), Three.js r128 via CDN
- **Transport:** `FirmwareTransport.send_json(msg: dict)` is the only host→firmware command path

## Workflow

- Mock firmware is the primary dev loop. Real hardware for integration only.
- `bark test` + `make bark-mock` must pass before commit.
- Small, focused commits. Ask before major pivots.
EOF
```

- [ ] **Step 2: Verify line count and no stale references**

```bash
wc -l CLAUDE.md
```

Expected: ≤ 100

```bash
grep -n "ScanBehavior\|scan\.py\|map\.js\|tuning\.html\|gait\.html\|sonar\.js\|walls\.js\|dead_reckoning\|capture_pose\|calibrate\|identify_servos\|behaviors/scan\|behaviors/map" CLAUDE.md
```

Expected: no matches.

```bash
grep -n "Milestone [1-4]\|Scanning\|Wall Mesh\|Physics Simulation\|lifecycle" CLAUDE.md
```

Expected: no matches (stale milestone history removed).

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: rewrite CLAUDE.md as pointer doc — core docs are source of truth"
```

---

### Task 8: Verify docs structure

- [ ] **Step 1: Confirm docs/ contains exactly the right files**

```bash
ls docs/
ls docs/superpowers/plans/
ls docs/superpowers/specs/
```

Expected:
- `docs/`: `attic.md`, `architecture.md`, `design-principles.md`, `hardware.md`, `manufacturer/`, `overview.md`, `protocol.md`, `superpowers/`
- `docs/superpowers/plans/`: only `2026-04-18-ota-owner-auth.md` and `2026-04-19-phase1-kernel-cut.md` and `2026-04-19-phase2-core-docs.md` (this plan)
- `docs/superpowers/specs/`: only `2026-04-19-kernel-reshape-design.md`

- [ ] **Step 2: Verify CLAUDE.md is ≤ 100 lines**

```bash
wc -l CLAUDE.md
```

Expected: < 100

- [ ] **Step 3: Verify every core doc has the right top-level sections**

```bash
grep "^## " docs/overview.md
grep "^## " docs/architecture.md
grep "^### " docs/protocol.md | wc -l   # 23 message types
grep "^## " docs/hardware.md
grep "^## " docs/design-principles.md
```

- [ ] **Step 4: Run firmware tests to confirm no regressions**

```bash
bark test
```

Expected: All 6 test binaries pass (103 tests).

- [ ] **Step 5: Final commit log review**

```bash
git log --oneline -10
```

Expected: a clean run of doc commits — overview, architecture, protocol, hardware, design-principles, attic, CLAUDE.md.
