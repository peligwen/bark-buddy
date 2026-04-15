# Web UI Simplification & Cleanup — Design Spec

**Date:** 2026-04-15
**Status:** Approved

## Problem

The bark-buddy web UI has accumulated dead code, broken features, and small bugs. Key issues:

- UI buttons that silently fail (patrol demo, set-default-pose send messages the server never handles)
- Sim+ transport option that always returns an error (no server switch case)
- Unused module exports bloating the public API surface
- Write-only JS state fields that serve no purpose
- A client-side wave animation arm that can never be triggered
- Legacy wall segment rendering path kept alongside the current chain-based path
- Three places using `window._` properties as cross-module communication instead of the module system
- A bug where opening the WiFi modal more than once stacks keydown listeners on the password input
- A CSS selector mismatch that silently prevents ultrasonic warn/danger colors from appearing
- Missing `lock_denied` browser handler — users who click "Take Control" while another operator holds it get zero feedback

## Goals

1. Delete all dead/broken code cleanly — no stubs, no backwards-compat shims
2. Fix identified bugs
3. Improve code quality by removing anti-patterns without over-engineering
4. Add `lock_denied` notification — minimal, reuses existing UI patterns

## Non-Goals

- Visual redesign
- New features beyond `lock_denied`
- Refactoring the server
- Adding tests

---

## Design

### What Gets Removed

| Item | Location | Why |
|---|---|---|
| Patrol Demo / Stop Patrol buttons + HTML | `index.html`, `app.module.js` | `cmd_patrol` has no server handler; patrol-status div is permanently hidden |
| Set Default button | `index.html`, `controls.js` | `cmd_set_default_pose` has no server handler; silently fails |
| Sim+ transport option | `index.html` | Always hits the server error path; no switch case |
| `getPoseNames` | `gait.js`, `index.js` | Exported and imported but never called |
| `POSES` export | `gait.js` | Not imported anywhere externally |
| `mapPoints`, `mapScans`, `mapWalls`, `mapChains`, `mapBounds` exports | `map.js` | Written internally, never imported — make module-internal |
| `state.animationId` | `state.js`, `index.js` | Set each RAF tick, never passed to `cancelAnimationFrame` |
| `state.isFallen` | `state.js` | Set by `setFallen()`, never read; emissive traversal is inline |
| `state.currentAction` | `state.js`, `gait.js` | Never set to non-null from client JS; firmware handles wave |
| Wave arm (`if (a === 1)`) in `animateAction` | `gait.js` | Dead branch — `currentAction` is always null client-side |
| `jointMarkers` object | `overlay.js` | Populated in `buildOverlay`, never read in `updateOverlay` |
| `buildWallsFromSegments` | `walls.js` | Legacy path; server always emits chains |
| Legacy `mapWalls` fallback branch | `map.js` | Paired with `buildWallsFromSegments` removal |

### Dead Animation Code (gait.js)

`animateAction` contains `if (a === 1)` wave pose code that can never execute — nothing in client-side JS sets `state.currentAction` to 1 (the wave is handled by firmware; the browser only receives joint updates via telemetry). Remove:
- The `if (a === 1)` branch from `animateAction`
- All reads/writes of `state.currentAction` in `setPose`, `clearPose`, and `animateAction`
- `state.currentAction` from `state.js`
- Simplify the guard in `animateGait` from `if (state.currentAction != null || state.targetPose)` → `if (state.targetPose)`

`animateAction` itself stays — it also handles pose interpolation when `state.targetPose` is set (via `setPose`).

### Bug Fixes

**panels.js — keydown listener accumulation**
The `passInput.addEventListener("keydown", ...)` call is inside `showWifiModal()`. Each time the modal opens, a new listener accumulates. Fix: move the listener registration to `setupTransport()` so it runs once on module load.

**style.css — ultrasonic warn/danger colors silent no-op**
`.stat .warn-text` and `.stat .danger-text` never match `#ultra-val` (which is inside `#telem-overlay`, not `.stat`). Fix: change the selectors to `.warn-text` and `.danger-text` globally (only one element uses these classes).

**app.module.js — stale `updateGauge` comment**
Change `// --- Gauge (hidden elements for compat) ---` to `// IMU pitch/roll text display`.

### Code Quality

**ws.js — `window._onWsOpen` → callback parameter**
`connect(url)` becomes `connect(url, onOpen)`. `ws.onopen` calls `onOpen()` directly. Removes the global property used as a side channel.

**controls.js — `window._balanceEnabled` → module-scoped var**
`window._balanceEnabled` is set and read within the same module. A module-scoped `let balanceEnabled` already exists. Remove all `window._balanceEnabled` usages; use the existing variable.

**controls.js — `.action-btn` selector scope**
`querySelectorAll(".action-btn")` catches buttons without `data-action` attributes (patrol, scan, reset), attaching no-op listeners. Change to `querySelectorAll(".action-btn[data-action]")`.

**walls.js — duplicate material setup**
`buildWallsFromChains` and `buildWallsFromSegments` each construct identical `wallMat`/`edgeMat` blocks. After removing `buildWallsFromSegments`, the remaining setup block is only used once — no extraction needed. Just remove the dead copy.

**map.js — two-phase `setupScan` init**
Change `setupScan()` to `setupScan(send)`, accepting `send` directly as all other modules do. Removes the `.init(send)` second step in `app.module.js`.

### New: `lock_denied` Handler

Server sends `{"type": "lock_denied", "operator": "..."}` when a client requests control but someone else holds it. The browser currently has no handler.

**Implementation:** Add a `lock_denied` case in `handleMessage` in `app.module.js`. Reuse the `#fall-alert` element pattern — temporarily set its text to `"Control held by [operator]"` (or `"Control request denied"` if no operator is provided), show it with `classList.remove("hidden")`, and auto-hide after 3 seconds. This avoids any new HTML or CSS.

---

## File Impact Summary

12 files modified, 0 files created, 0 files deleted.

Files: `web/index.html`, `web/style.css`, `web/app.module.js`, `web/modules/ws.js`, `web/modules/controls.js`, `web/modules/map.js`, `web/modules/panels.js`, `web/dog3d/gait.js`, `web/dog3d/index.js`, `web/dog3d/state.js`, `web/dog3d/overlay.js`, `web/dog3d/walls.js`

---

## Verification

1. `python host/server.py --sim` — start in sim mode
2. Open `http://localhost:8765`
3. Confirm patrol buttons and Set Default are gone
4. Confirm Sim+ is absent from transport menu
5. Confirm D-pad, balance toggle, wave action, scan all still work
6. Confirm 3D dog and wall mesh render correctly
7. Open WiFi modal twice — pressing Enter should fire connect only once
8. Trigger low ultrasonic distance in sim — verify warn/danger color appears on distance display
9. Open two browser tabs — in tab 2, click Take Control while tab 1 holds it — confirm `lock_denied` notification appears in tab 2
