# Auto-Connect, Auto-Detect & OTA Update Design

**Date:** 2026-04-15
**Status:** Draft

## Context

Today, connecting Bark-Buddy to hardware requires manual steps: the server must be running before USB is plugged in, WiFi requires knowing the IP address, and firmware updates require a physical USB connection with PlatformIO. This design adds seamless auto-detection, auto-connection, and over-the-air firmware updates so the experience is closer to "power on and go."

## Overview

Four incremental subsystems, each building on the previous:

1. **Firmware version tracking** — version constant in firmware, reported in boot message
2. **USB hot-plug detection** — host detects USB attach/detach at runtime
3. **mDNS WiFi discovery** — firmware advertises itself, host finds it automatically
4. **HTTP OTA firmware updates** — one-click firmware update from the web UI

Transport priority cascade (highest to lowest):
```
USB serial (custom FW) → USB serial (stock FW) → WiFi TCP (mDNS) → Simulation
```

---

## 1. Firmware Version Tracking

### Problem
Neither the C++ firmware nor the host track firmware versions. There's no way to know what's running on the robot or whether it's outdated.

### Design

**Firmware (`config.h`):**
- Add `FW_VERSION_MAJOR`, `FW_VERSION_MINOR`, `FW_VERSION_PATCH` constants
- Add `FW_VERSION` string macro (e.g., `"1.0.0"`)
- Add `FW_BUILD_TIMESTAMP` macro using `__DATE__` and `__TIME__`

**Boot message (`main.cpp`):**
- Add `"fw_version"` and `"fw_build"` fields to the JSON boot message
- Example: `{"type":"boot", "fw_version":"1.0.0", "fw_build":"Apr 15 2026 14:30:00", ...}`

**Host (`json_transport_base.py`):**
- Parse and store `fw_version` and `fw_build` from boot messages in `_firmware_info`
- Expose via existing `firmware_info` property

**Web UI:**
- Add firmware version to the `telem_status` broadcast
- Display firmware version in a small badge near the transport badge in the header
- Grey when unknown, green when current, amber when outdated

### Files Modified
- `firmware/include/config.h` — version constants
- `firmware/src/main.cpp` — boot message fields
- `host/json_transport_base.py` — parse version from boot message
- `host/server.py` — include fw_version in telem_status WebSocket broadcasts
- `web/index.html` — firmware version badge element
- `web/style.css` — badge styling
- `web/modules/panels.js` or `web/app.module.js` — update badge from telem_status

---

## 2. USB Hot-Plug Detection

### Problem
If the robot is plugged in after the server starts, or unplugged and re-plugged, the server doesn't notice. The user must restart the server.

### Design

**Device monitor (`host/device_monitor.py` — new file):**
- Async loop polling `serial.tools.list_ports.comports()` every 2 seconds
- Tracks known USB serial ports matching patterns: `/dev/cu.usbserial*`, `/dev/ttyUSB*`, `COM*`
- On new device: emit `device_added(port)` callback
- On device removal: emit `device_removed(port)` callback
- Lightweight — just port enumeration, no connection logic

**Server integration (`host/server.py`):**
- Start `DeviceMonitor` alongside the server
- On `device_added`: if currently on sim or WiFi, probe the new port (reuse existing `_detect_serial_transport()`) and auto-connect if firmware detected. Respect priority: USB > WiFi > sim.
- On `device_removed`: if the removed port matches the active serial transport, gracefully disconnect and fall back to WiFi (if mDNS-discovered) or sim.
- Broadcast transport change to WebSocket clients via existing status messages.

**Existing reconnect loop (`_reconnect_loop`):**
- Remains as-is for handling transient connection drops on the *same* transport
- Hot-plug handles transport *switching*, reconnect handles transport *recovery*

### Files Modified
- `host/device_monitor.py` — new file, ~60 lines
- `host/server.py` — integrate DeviceMonitor callbacks, adjust transport switching

---

## 3. mDNS WiFi Discovery

### Problem
To connect over WiFi, the user must know the robot's IP address. There's no automatic discovery.

### Design

**Firmware (`main.cpp`):**
- Add `#include <ESPmDNS.h>`
- On WiFi connect: start mDNS with hostname `mechdog` (resolvable as `mechdog.local`)
- Register service: `_mechdog._tcp` on port 9000
- Add TXT records: `fw_version=1.0.0`, `board=esp32-d0wd`
- On WiFi disconnect: stop mDNS

**Host (`host/mdns_browser.py` — new file):**
- Use `zeroconf` Python library (add to requirements)
- Browse for `_mechdog._tcp.local.` services
- On service found: extract IP, port, and TXT records
- Callback to server with discovered device info
- On service removed: callback for device departure

**Server integration (`host/server.py`):**
- Start mDNS browser alongside the server
- On robot discovered via mDNS: if no USB connection active, auto-connect via `FirmwareTransport(host=ip, port=port)`
- On robot lost from mDNS: if that was the active WiFi transport, fall back to sim
- Store discovered firmware version from TXT records for version comparison

**CLI changes:**
- `--wifi` flag now optional — mDNS discovery is the default for WiFi
- `--wifi <ip>` still works as a manual override (skips mDNS)
- Add `--no-mdns` flag to disable discovery if needed

### Dependencies
- `zeroconf` Python package (pure Python, well-maintained)

### Files Modified
- `firmware/src/main.cpp` — mDNS setup/teardown
- `firmware/platformio.ini` — may need ESPmDNS lib (usually bundled with ESP32 Arduino)
- `host/mdns_browser.py` — new file, ~80 lines
- `host/server.py` — integrate mDNS browser, adjust transport selection
- `requirements.txt` or equivalent — add `zeroconf`

---

## 4. HTTP OTA Firmware Updates

### Problem
Updating firmware requires USB + PlatformIO CLI. No way to update over WiFi or from the web UI.

### Design

### 4a. Firmware OTA Client

**New command: `cmd_ota_update` (firmware side):**
- Receives `{"type":"cmd_ota_update", "url":"http://192.168.1.x:8080/api/firmware/binary"}`
- On receipt:
  1. Ack the command
  2. Transition lifecycle to `updating` state (new lifecycle state)
  3. Command servos to rest pose via existing `set_lifecycle_target(REST)` path
  4. Start rainbow breathing LED pattern (see below)
  5. Pull binary from URL using `HTTPClient` + `Update` library
  6. Stream binary to flash partition via `Update.begin()` / `Update.write()` / `Update.end()`
  7. On success: send `{"type":"ota_status", "status":"complete"}`, reboot after 1s delay
  8. On failure: send `{"type":"ota_status", "status":"failed", "error":"..."}`, restore normal LEDs, return to idle lifecycle

**Rainbow breathing LED pattern:**
- Cycle through HSV hue (0-360) at ~2s per full rotation
- Sinusoidal brightness envelope: min 20%, max 100%, period ~3s
- Applied to both ultrasonic LEDs (I2C sonar module LEDs)
- Runs in the main loop during OTA (LED update every 30ms)
- Pattern stops on OTA completion or failure

**New lifecycle state: `updating`**
- Added to the lifecycle state machine alongside existing states (booting, idle, active, sleeping, resting, waking)
- Entered only from `idle` or `active` states
- All motion commands rejected while in `updating`
- On OTA failure, transitions back to `idle`
- On OTA success, device reboots (lifecycle restarts from `booting`)

**Safety:**
- OTA write uses the ESP32's dual-partition scheme (OTA0/OTA1) — the current firmware remains intact until the new one is fully written and verified
- If OTA fails mid-write, the device continues running the old firmware
- Watchdog remains active during OTA to catch lockups

### 4b. Host OTA Server

**New API endpoints in `host/server.py`:**

`GET /api/firmware/status`
- Returns JSON: `{"current_version": "1.0.0", "available_version": "1.1.0", "update_available": true, "build_timestamp": "...", "transport": "wifi"}`
- `current_version` from the connected firmware's boot message
- `available_version` extracted from `firmware/include/config.h` source (what would be built)
- `update_available` true when versions differ and transport is WiFi

`POST /api/firmware/build`
- Runs `pio run` in the `firmware/` directory
- Returns build status and path to `.bin` file
- Streams build output via WebSocket for progress visibility

`GET /api/firmware/binary`
- Serves the built firmware binary from `.pio/build/esp32dev/firmware.bin`
- Only available after a successful build

`POST /api/firmware/update`
- Orchestrates the full update flow:
  1. Build firmware (`pio run`)
  2. Extract new version from built binary or config.h
  3. Compare with running version
  4. Send `cmd_ota_update` to the robot with the binary URL
  5. Monitor `ota_status` telemetry messages
  6. Report success/failure to the web UI

**Version extraction:**
- Parse `FW_VERSION` from `firmware/include/config.h` using a simple regex
- Compare with `fw_version` reported in the firmware's boot message

### 4c. Web UI

**Firmware version badge (header area):**
- Shows current firmware version (e.g., "FW 1.0.0")
- Green when current, amber with pulsing dot when update available
- Clicking the badge when an update is available opens the update panel

**Update panel (modal or expandable section):**
- Shows: current version, available version, changelog (if we add one later)
- "Update Firmware" button
- Progress indicator during build + flash
- Status messages: "Building...", "Uploading...", "Flashing...", "Rebooting..."
- Disabled when connected via USB (OTA requires WiFi) or when transport is sim

**Lifecycle badge update:**
- Add `updating` state: background `#1a0e2e`, text `#b388ff` (purple/violet), with a CSS pulse animation
- D-pad remains disabled during `updating` (same gating as non-active states)

### Files Modified
- `firmware/include/config.h` — add partition table reference
- `firmware/src/main.cpp` — OTA handler, rainbow LED pattern, updating lifecycle state
- `firmware/src/command_handlers.cpp` — register `cmd_ota_update` handler
- `firmware/include/command_handlers.h` — declare OTA handler
- `firmware/platformio.ini` — OTA partition table, HTTPClient dependency
- `host/server.py` — OTA API endpoints, build orchestration
- `web/index.html` — firmware badge, update panel markup
- `web/style.css` — update panel styling, updating lifecycle color
- `web/app.module.js` — handle ota_status messages, firmware badge logic
- `web/modules/panels.js` — update panel behavior

---

## Partition Table

The ESP32 needs a partition table that supports OTA (two app partitions). The default `default.csv` single-app partition won't work.

Use the built-in `min_spiffs.csv` or a custom table:
```
# Name,   Type, SubType, Offset,  Size
nvs,      data, nvs,     0x9000,  0x5000
otadata,  data, ota,     0xe000,  0x2000
app0,     app,  ota_0,   0x10000, 0x1E0000
app1,     app,  ota_1,   0x1F0000,0x1E0000
spiffs,   data, spiffs,  0x3D0000,0x30000
```

This gives ~1.9MB per app slot, which is plenty for the current firmware (~300KB).

Set in `platformio.ini`:
```ini
board_build.partitions = min_spiffs.csv
```

---

## Transport Priority & Auto-Switching

### Priority Order
1. USB serial (custom firmware) — fastest, most reliable
2. USB serial (stock firmware) — fallback serial path
3. WiFi TCP (mDNS discovered) — wireless
4. Simulation — no hardware

### Switching Rules
- **Upward switch (to higher priority):** Automatic. USB plugged in while on WiFi → probe, connect, switch. Notify UI.
- **Downward switch (to lower priority):** Automatic on disconnect. USB removed → fall to WiFi if available → fall to sim. Notify UI.
- **User override:** Manual transport selection in UI always wins. If user explicitly chose WiFi, don't auto-switch to USB.
- **Lock held:** If another operator holds the control lock, don't auto-switch transport (could disrupt them).

### Connection State Machine
```
                    ┌──────────────┐
                    │  Disconnected │
                    └──────┬───────┘
                           │ USB detected / mDNS found
                           ▼
                    ┌──────────────┐
                    │   Probing    │ (detect firmware type)
                    └──────┬───────┘
                           │ probe success
                           ▼
                    ┌──────────────┐
                    │  Connected   │
                    └──────┬───────┘
                           │ device removed / connection lost
                           ▼
                    ┌──────────────┐
                    │  Falling back│ (try next priority)
                    └──────────────┘
```

---

## Web UI Status Summary

The header will show connection info at a glance:

```
[Online ●] [USB ▾] [FW 1.0.0 ●]    [🔒 Lock] [Gwen]
```

- Connection indicator: existing green/red dot
- Transport badge: existing, now auto-updates on switches
- Firmware badge: new, shows version + update availability
- All badges are clickable for more detail/actions

---

## Build Order

Each phase is independently useful:

| Phase | Feature | Firmware Changes | Host Changes | Web UI Changes |
|-------|---------|-----------------|--------------|----------------|
| 1 | Version tracking | `config.h`, `main.cpp` boot msg | `json_transport_base.py`, `server.py` | Version badge |
| 2 | USB hot-plug | None | `device_monitor.py`, `server.py` | Transport switch notifications |
| 3 | mDNS discovery | `main.cpp` mDNS setup | `mdns_browser.py`, `server.py` | Auto-connect status |
| 4 | OTA updates | OTA handler, rainbow LEDs, partition table, `updating` lifecycle | OTA API endpoints, build orchestration | Update panel, progress UI |

---

## Dependencies

- `zeroconf` — Python mDNS browser (Phase 3)
- `ESPmDNS.h` — ESP32 Arduino mDNS (Phase 3, bundled with ESP32 platform)
- `HTTPClient.h` — ESP32 HTTP client for OTA pull (Phase 4, bundled)
- `Update.h` — ESP32 OTA flash library (Phase 4, bundled)

---

## Verification

### Phase 1 — Version Tracking
- Flash firmware, check boot message includes `fw_version` field
- Verify web UI shows firmware version badge
- Bump version in `config.h`, rebuild, verify host detects different available version

### Phase 2 — USB Hot-Plug
- Start server with no USB connected (should start in sim mode)
- Plug in robot → server auto-detects, probes, connects, UI switches from sim to live
- Unplug robot → server detects removal, falls back to sim, UI updates
- Re-plug → auto-reconnects

### Phase 3 — mDNS Discovery
- Flash firmware with WiFi enabled, connect to network
- Start host without `--wifi` flag → host discovers robot via mDNS, connects
- Verify `mechdog.local` resolves on the network
- Power off robot → host falls back to sim

### Phase 4 — OTA Updates
- Connect to robot over WiFi (USB or mDNS)
- Bump version in `config.h`
- Click "Update Firmware" in web UI
- Verify: dog goes to rest pose, rainbow breathing LEDs start
- Verify: build runs, binary transfers, flash completes
- Verify: robot reboots, reconnects with new version, LEDs return to normal
- Test failure case: serve a corrupt binary, verify firmware reports failure and stays on old version
