# Bark-Buddy Kernel Reshape — Design Spec

**Date:** 2026-04-19

## Context

The project has grown to ~17k lines across firmware, host, and web. Along the way we've accumulated mapping experiments, calibration tools, secondary tuning UIs, and milestone-specific scripts that are no longer on the path to the next goal (SLAM → composite mapping → waypoint navigation). Stale comments and references have also built up.

This reshape aggressively prunes the repo to a minimal motion kernel, establishes five curated documents in `docs/` as the new source of truth, and codifies design principles that will govern future growth. Target: ~50% reduction in code on `main`, with atticized code preserved in a `pre-reshape` branch and indexed in `docs/attic.md`.

This is a multi-year project. Durable shapes beat clever shortcuts.

## Goals

- Establish five core docs in `docs/` as the authoritative source of truth.
- Reduce code on `main` by ~50%; move non-kernel code to `pre-reshape` branch.
- Codify design principles that govern future growth.
- Rename `FirmwareTransport` → `Dog`; split into `host/dog/` package at natural seams.
- Preserve the in-progress IK gait pipeline as in-kernel and unaffected.

## Non-goals

- Rewriting mapping (SLAM + composite + waypoint nav) — later milestone.
- Adding new features during the reshape.
- OTA owner-auth implementation — separate plan (`docs/superpowers/plans/2026-04-18-ota-owner-auth.md`).
- Feature flags, compatibility shims, deprecation periods.

---

## Kernel (stays on `main`)

**Firmware**
- `firmware/src/*` — main, servos, gait, imu, sonar, balance, command_handlers, sensor_task, offsets, battery_led
- `firmware/include/*` — all current headers
- `firmware/mock/*` — full native build (shims, physics, net_tcp, mock_main, sensor_task, globals, imu_mock, sonar_mock)
- `firmware/test/*` — IK, transform, balance, offset, gait, servo unit tests
- `firmware/platformio.ini`

**Host**
- `host/server.py` (phase 3 split target)
- `host/firmware_transport.py` → `host/dog/` package (phase 3)
- `host/comms.py` → collapsed/deleted (phase 3)
- `host/behaviors/balance.py` + `__init__.py`
- `bark_cli.py`

**Web**
- `web/index.html`, `web/style.css`, `web/app.module.js`
- `web/modules/ws.js`, `controls.js`, `panels.js`, `diag.js`
- `web/dog3d/index.js`, `model.js`, `gait.js`, `camera.js`, `state.js`

**Root:** `CLAUDE.md` (rewritten phase 2), `README.md`, new `docs/*`

---

## Attic (pre-reshape branch + delete on main)

**Mapping / navigation**
- `host/behaviors/scan.py`, `map_store.py`, `wall_fit.py`, `wall_mesh.py`, `octree.py`
- `host/dead_reckoning.py`

**Calibration / sweep tooling**
- `host/sweep/` (entire directory)
- `host/capture_pose.py`, `calibrate_servos.py`, `servo_test.py`, `identify_servos.py`, `map_servos.py`, `apply_to_firmware.py`, `probe_stock_firmware.py`

**Ad-hoc test scripts**
- `host/test_mapping.py`, `test_scan.py`, `test_wall_mesh.py`, `test_button_engage.py`

**Secondary web UIs**
- `web/tuning.html`, `web/gait.html`, `web/modules/map.js`
- `web/dog3d/sonar.js`, `walls.js`, `overlay.js`

**Stale docs**
- `docs/stock-firmware-analysis.md`, `implementation-plan.md`, `decisions.md`
- `docs/hardware-schematic.md` (content folded into new `hardware.md`)
- Old `docs/architecture.md` and `protocol.md` (rewritten, not kept verbatim)
- Completed files in `docs/superpowers/plans/` and `docs/superpowers/specs/`

**Deferred (not kernel, not attic yet)**
- `host/ota_flash.py` — revisit during phase 3 alongside OTA owner-auth plan.

**Attic mechanics:** branch `pre-reshape` at current HEAD before any deletion. `docs/attic.md` holds a one-line-per-file index (`path — reason`).

---

## Core docs (five files)

1. **`docs/overview.md`** — mission, kernel scope, deferred work, pointer map to other four docs.
2. **`docs/architecture.md`** — components (firmware, mock, host, web), layer responsibilities, data flow, transport boundary, composable-behavior pattern.
3. **`docs/protocol.md`** — authoritative wire format. Every command and telemetry message with fields, types, semantics. Rewritten from scratch.
4. **`docs/hardware.md`** — pins, peripherals, I2C buses, IMU axis convention, servo indexing + offsets. Absorbs `hardware-schematic.md`.
5. **`docs/design-principles.md`** — prescriptive rules below.

---

## Design principles

**Meta**
- **Multi-year horizon.** Durable shapes over clever shortcuts. Favor small, single-purpose files and classes from the start, not as an afterthought.

**Architecture boundaries**
- **Single transport.** `Dog.send_json()` is the only host→firmware path. JSON only; no text protocols.
- **Layer discipline.** Firmware owns motion + hardware; host owns coordination/behaviors; web owns presentation. No layer reaches past its neighbor.
- **Mock parity.** Mock shares source with real firmware; diverges only at platform shims (`firmware/mock/`) and link-time driver substitution. New drivers require a mock implementation before kernel code uses them.

**Code discipline**
- **Split early at natural seams.** A class with two reasons to change is two classes. File line count is a lagging signal; responsibility is the leading one.
- **No speculative abstractions.** One-implementation ABCs, options no code sets, hooks with no callers — delete.
- **No speculative features** or defensive checks at internal boundaries. Trust internal code; validate at system boundaries.
- **No dead code, no ghost comments.** Delete or don't.
- **Comments state WHY** when non-obvious. Never narrate WHAT.

**Docs as source of truth**
- The five `docs/` files are the contract. If a behavior isn't in them, it isn't committed to.
- Specs in `docs/superpowers/specs/`, plans in `docs/superpowers/plans/` — both ephemeral, pruned when complete.
- `CLAUDE.md` points to core docs; never duplicates them.

**Dev loop**
- Mock firmware is the primary dev loop. Real hardware for integration.
- `bark test` + `make bark-mock` must pass before commit.

---

## Phases

### Phase 1 — Obvious cuts

1. `git branch pre-reshape` at current HEAD.
2. Import-graph sweep: grep for imports of every attic file across `host/`, `web/`, `bark_cli.py`. Address surviving references.
3. Delete attic files on `main` (full list above).
4. Update `bark_cli.py` to drop references to deleted tools.
5. Verify: `bark test`, `make bark-mock`, smoke run via `bark mock`.
6. Commit in small clusters, one concern per commit.

### Phase 2 — Core docs

1. Write five `docs/` files.
2. Rewrite `CLAUDE.md` as a short pointer to core docs + minimal workflow rules.
3. Write `docs/attic.md` (one line per removed file).
4. Commit each doc separately.

### Phase 3 — Principle enforcement

1. Rename `FirmwareTransport` → `Dog`. Create `host/dog/` package:
   - `__init__.py` — re-exports `Dog`
   - `dog.py` — public API (send_json, telemetry callbacks, lifecycle)
   - `io.py` — serial + TCP I/O
   - `discover.py` — auto-detect (USB → mDNS)
2. Collapse `Transport` ABC in `host/comms.py`; migrate constants (`DIRECTIONS`, `SERIAL_BAUD`) into `host/dog/`; delete `comms.py` if empty.
3. Split `host/server.py` (1118 lines) by responsibility: WS session handling, telemetry relay, command routing, static file serving, behavior lifecycle.
4. Strip stale comments; collapse duplicated logic.
5. Fold minimal offset + servo-pin UX into primary UI. Reuses `cmd_offset`, `cmd_servo_pin`, `telem_servo_pins`.
6. Decide `host/ota_flash.py`: kernel or attic, alongside OTA owner-auth plan.
7. Resume IK gait pipeline under the new rules.

---

## Known gaps (to address in phase 3)

- **Offset + servo-pin UX:** `tuning.html` is atticized. `cmd_offset` and `cmd_servo_pin` have no UI until phase 3 adds a minimal panel to the primary UI.
- **`web/diag.js` + `panels.js` imports:** may reference atticized modules — import-graph sweep in phase 1 will catch these.

---

## Verification

**After phase 1**
- `git branch --list pre-reshape` exists.
- Line count reduction ~50% vs. pre-cut.
- `bark test` passes. `make bark-mock` clean. `bark mock` runs: browser connects, commands reach mock, telemetry streams, 3D view updates.

**After phase 2**
- `docs/` contains exactly the five core files + `attic.md` + `docs/superpowers/`.
- `CLAUDE.md` < 100 lines with no duplicated architecture/protocol content.
- Every kernel feature described in at least one core doc.

**After phase 3**
- No `firmware_transport.py` or `comms.py` on main.
- `host/dog/` package exists with four focused files.
- No kernel file > 400 lines (soft target; natural-seams rule governs).
- Offset + servo-pin UX in primary UI.
- Full integration smoke test passes.
