# Web UI Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove dead code (broken buttons, unused exports, write-only state, legacy fallback), fix three bugs, clean up three code quality issues, and add a `lock_denied` notification.

**Architecture:** Pure deletion/cleanup pass on 12 vanilla JS/HTML/CSS files. No new files. No new abstractions. Each task is self-contained. The `sim+` transport option, patrol feature, and Set Default button are deleted entirely; the `lock_denied` notification reuses the existing `#fall-alert` element.

**Tech Stack:** Vanilla HTML/CSS/ES modules, Three.js r128 (CDN), Python host server (`host/server.py --sim` for testing)

---

## Task 1: Remove dead HTML — patrol, Set Default, Sim+

**Files:**
- Modify: `web/index.html`

- [ ] **Step 1: Remove patrol buttons and status div**

In `web/index.html`, in the Operations section (lines 130–145), replace:

```html
            <div class="ctrl-section">
                <h2>Operations</h2>
                <div class="action-bar">
                    <button class="action-btn" id="btn-patrol-demo">Patrol Demo</button>
                    <button class="action-btn" id="btn-patrol-stop" disabled>Stop Patrol</button>
                    <button class="action-btn" id="btn-scan-start">Scan Area</button>
                    <button class="action-btn" id="btn-scan-stop" disabled>Stop Scan</button>
                    <button class="action-btn" id="btn-map-clear">Clear Map</button>
                    <button class="action-btn" id="btn-reset">Reset</button>
                    <button class="action-btn" id="btn-restart" style="color:var(--red)">Restart Server</button>
                </div>
                <div id="patrol-status" class="hidden">
                    <span id="patrol-pos"></span>
                    <span id="patrol-wp"></span>
                </div>
            </div>
```

with:

```html
            <div class="ctrl-section">
                <h2>Operations</h2>
                <div class="action-bar">
                    <button class="action-btn" id="btn-scan-start">Scan Area</button>
                    <button class="action-btn" id="btn-scan-stop" disabled>Stop Scan</button>
                    <button class="action-btn" id="btn-map-clear">Clear Map</button>
                    <button class="action-btn" id="btn-reset">Reset</button>
                    <button class="action-btn" id="btn-restart" style="color:var(--red)">Restart Server</button>
                </div>
            </div>
```

- [ ] **Step 2: Remove Set Default button**

In the Actions section (lines 110–128), replace:

```html
                    <button class="action-btn" id="btn-pose-go">Pose</button>
                    <button class="action-btn" id="btn-pose-default">Set Default</button>
                    <button class="action-btn" id="btn-balance" data-action="balance-toggle">Balance: OFF</button>
```

with:

```html
                    <button class="action-btn" id="btn-pose-go">Pose</button>
                    <button class="action-btn" id="btn-balance" data-action="balance-toggle">Balance: OFF</button>
```

- [ ] **Step 3: Remove Sim+ from transport menu**

In the transport menu (lines 19–27), replace:

```html
                    <div id="transport-menu" class="transport-menu hidden">
                        <button data-transport="usb">USB (stock)</button>
                        <button data-transport="usb-fw">USB (firmware)</button>
                        <button data-transport="wifi">WiFi (stock)</button>
                        <button data-transport="wifi-fw">WiFi (firmware)</button>
                        <button data-transport="wifi-setup">Setup WiFi...</button>
                        <button data-transport="sim+">Sim+</button>
                        <button data-transport="sim">Sim (classic)</button>
                    </div>
```

with:

```html
                    <div id="transport-menu" class="transport-menu hidden">
                        <button data-transport="usb">USB (stock)</button>
                        <button data-transport="usb-fw">USB (firmware)</button>
                        <button data-transport="wifi">WiFi (stock)</button>
                        <button data-transport="wifi-fw">WiFi (firmware)</button>
                        <button data-transport="wifi-setup">Setup WiFi...</button>
                        <button data-transport="sim">Sim (classic)</button>
                    </div>
```

- [ ] **Step 4: Commit**

```bash
git add web/index.html
git commit -m "chore(web): remove patrol buttons, Set Default button, and Sim+ transport option"
```

---

## Task 2: Remove patrol JS from app.module.js + fix stale comment

**Files:**
- Modify: `web/app.module.js`

- [ ] **Step 1: Remove setupPatrol function and call**

In `web/app.module.js`, delete the entire `setupPatrol` function (lines 162–173):

```js
// --- Patrol ---
function setupPatrol() {
    var btnDemo = document.getElementById("btn-patrol-demo");
    var btnStop = document.getElementById("btn-patrol-stop");
    btnDemo.addEventListener("click", function () {
        send({ type: "cmd_patrol", action: "start", waypoints: [
            { x: 0.5, y: 0, heading: 0 }, { x: 0.5, y: 0.5, heading: 90 },
            { x: 0, y: 0.5, heading: 180 }, { x: 0, y: 0, heading: 270 },
        ]});
    });
    btnStop.addEventListener("click", function () { send({ type: "cmd_patrol", action: "stop" }); });
}
```

Also remove the `setupPatrol();` call in the Init section (line 196).

- [ ] **Step 2: Fix stale updateGauge comment**

Replace (line 46):

```js
// --- Gauge (hidden elements for compat) ---
```

with:

```js
// IMU pitch/roll text display
```

- [ ] **Step 3: Commit**

```bash
git add web/app.module.js
git commit -m "chore(web): remove setupPatrol and fix stale updateGauge comment"
```

---

## Task 3: Replace window._onWsOpen with onOpen callback

**Files:**
- Modify: `web/modules/ws.js`
- Modify: `web/app.module.js`

- [ ] **Step 1: Update ws.js connect() signature**

In `web/modules/ws.js`, replace the `connect` function (lines 9–27):

```js
export function connect() {
    if (ws && ws.readyState <= WebSocket.OPEN) return;
    ws = new WebSocket(WS_URL);

    ws.onopen = function () {
        setConnected(true);
        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
        if (window._onWsOpen) window._onWsOpen();
    };
```

with:

```js
export function connect(onOpen) {
    if (ws && ws.readyState <= WebSocket.OPEN) return;
    ws = new WebSocket(WS_URL);

    ws.onopen = function () {
        setConnected(true);
        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
        if (onOpen) onOpen();
    };
```

- [ ] **Step 2: Update app.module.js to use the callback**

In `web/app.module.js`, replace (lines 219–225):

```js
// On WS open, identify and request map
window._onWsOpen = function () {
    send({ type: "cmd_identify", name: operatorName });
    send({ type: "cmd_map", action: "get" });
};

connect();
```

with:

```js
connect(function () {
    send({ type: "cmd_identify", name: operatorName });
    send({ type: "cmd_map", action: "get" });
});
```

- [ ] **Step 3: Commit**

```bash
git add web/modules/ws.js web/app.module.js
git commit -m "chore(web): replace window._onWsOpen global with connect(onOpen) callback"
```

---

## Task 4: Clean up controls.js

**Files:**
- Modify: `web/modules/controls.js`

- [ ] **Step 1: Lift balanceEnabled to module scope and remove window._balanceEnabled**

In `web/modules/controls.js`, add a module-level declaration after the `canControlFn` declaration (line 4):

```js
var canControlFn = null;
export function setCanControl(fn) { canControlFn = fn; }
function canControl() { return canControlFn ? canControlFn() : true; }
```

→ becomes:

```js
var canControlFn = null;
export function setCanControl(fn) { canControlFn = fn; }
function canControl() { return canControlFn ? canControlFn() : true; }

var balanceEnabled = false;
```

- [ ] **Step 2: Remove window._balanceEnabled from setupActions and update setBalanceState**

Replace (lines 82–127):

```js
export function setupActions(Dog3D) {
    var balanceEnabled = false;
    window._balanceEnabled = false;

    document.querySelectorAll(".action-btn").forEach(function (btn) {
```

with:

```js
export function setupActions(Dog3D) {
    document.querySelectorAll(".action-btn[data-action]").forEach(function (btn) {
```

Then in `setBalanceState` inside `setupActions`, replace:

```js
        setBalanceState: function (enabled) {
            balanceEnabled = enabled;
            window._balanceEnabled = enabled;
```

with:

```js
        setBalanceState: function (enabled) {
            balanceEnabled = enabled;
```

- [ ] **Step 3: Remove Set Default button handler**

In `setupActions`, remove these lines (lines 104–110):

```js
    var defaultPose = "rest";
    document.getElementById("btn-pose-default").addEventListener("click", function () {
        defaultPose = document.getElementById("pose-select").value;
        document.getElementById("btn-pose-default").textContent = "Default: " + defaultPose;
        send({ type: "cmd_set_default_pose", pose: defaultPose });
    });
    document.getElementById("btn-pose-default").textContent = "Set Default";
```

- [ ] **Step 4: Fix keyboard handler to use module-scoped balanceEnabled**

Replace (line 60):

```js
        if (e.key === "b") { send({ type: "cmd_balance", enabled: !window._balanceEnabled }); }
```

with:

```js
        if (e.key === "b") { send({ type: "cmd_balance", enabled: !balanceEnabled }); }
```

- [ ] **Step 5: Commit**

```bash
git add web/modules/controls.js
git commit -m "chore(web): remove window._balanceEnabled global and Set Default handler; fix .action-btn selector scope"
```

---

## Task 5: Clean up map.js — unused exports, legacy fallback, setupScan API

**Files:**
- Modify: `web/modules/map.js`
- Modify: `web/app.module.js`

- [ ] **Step 1: Make 4 arrays module-internal (remove exports); drop mapWalls entirely**

In `web/modules/map.js`, replace (lines 2–7):

```js
export var dogMapState = { x: 0, y: 0, heading: 0, motion: "stop" };
export var mapPoints = [];
export var mapScans = [];
export var mapWalls = [];
export var mapChains = [];
export var mapBounds = { min_x: -2, max_x: 2, min_y: -2, max_y: 2 };
```

with:

```js
export var dogMapState = { x: 0, y: 0, heading: 0, motion: "stop" };
var mapPoints = [];
var mapScans = [];
var mapChains = [];
var mapBounds = { min_x: -2, max_x: 2, min_y: -2, max_y: 2 };
```

(`mapWalls` is deleted entirely — it's only used by the legacy fallback branch being removed in step 2.)

- [ ] **Step 2: Remove legacy mapWalls fallback branch in drawMap**

In `drawMap`, replace (lines 104–116):

```js
    } else if (mapWalls && mapWalls.length > 0) {
        // Legacy fallback: wall segments
        ctx.strokeStyle = "#6366f1"; ctx.lineWidth = 3; ctx.lineCap = "round";
        for (var wi = 0; wi < mapWalls.length; wi++) {
            var wall = mapWalls[wi];
            ctx.globalAlpha = 0.3 + 0.7 * (wall.confidence || 0.5);
            ctx.beginPath();
            ctx.moveTo(toCanvasX(wall.x1), toCanvasY(wall.y1));
            ctx.lineTo(toCanvasX(wall.x2), toCanvasY(wall.y2));
            ctx.stroke();
        }
        ctx.globalAlpha = 1.0;
    }
```

with:

```js
    }
```

(Close the `if (mapChains...)` block directly, removing the else branch entirely.)

Also remove `mapWalls = data.walls || [];` from `renderFullMap` (line 22):

Replace:

```js
    mapPoints = data.points || [];
    mapScans = data.scans || [];
    mapWalls = data.walls || [];
    mapChains = data.chains || [];
```

with:

```js
    mapPoints = data.points || [];
    mapScans = data.scans || [];
    mapChains = data.chains || [];
```

And remove the `var mapWalls = [];` declaration (it was at module scope in step 1 above — already done).

- [ ] **Step 3: Simplify setupScan to accept send as a parameter**

Replace the entire `setupScan` function (lines 151–173):

```js
export function setupScan() {
    var btnStart = document.getElementById("btn-scan-start");
    var btnStop = document.getElementById("btn-scan-stop");
    var btnClear = document.getElementById("btn-map-clear");
    var sendFn = null;

    return {
        init: function(send) {
            sendFn = send;
            btnStart.addEventListener("click", function () { sendFn({ type: "cmd_scan", action: "start" }); });
            btnStop.addEventListener("click", function () { sendFn({ type: "cmd_scan", action: "stop" }); });
            btnClear.addEventListener("click", function () { sendFn({ type: "cmd_map", action: "clear" }); });
            drawMap();
        },
        setScanRunning: function(running) {
            btnStart.disabled = running;
            btnStop.disabled = !running;
            var progress = document.getElementById("scan-progress");
            if (running) progress.classList.remove("hidden");
            else progress.classList.add("hidden");
        }
    };
}
```

with:

```js
export function setupScan(send) {
    var btnStart = document.getElementById("btn-scan-start");
    var btnStop = document.getElementById("btn-scan-stop");
    var btnClear = document.getElementById("btn-map-clear");

    btnStart.addEventListener("click", function () { send({ type: "cmd_scan", action: "start" }); });
    btnStop.addEventListener("click", function () { send({ type: "cmd_scan", action: "stop" }); });
    btnClear.addEventListener("click", function () { send({ type: "cmd_map", action: "clear" }); });
    drawMap();

    return {
        setScanRunning: function(running) {
            btnStart.disabled = running;
            btnStop.disabled = !running;
            var progress = document.getElementById("scan-progress");
            if (running) progress.classList.remove("hidden");
            else progress.classList.add("hidden");
        }
    };
}
```

- [ ] **Step 4: Update app.module.js to use new setupScan(send) API**

In `web/app.module.js`, replace (lines 197–198):

```js
var scanCtrl = setupScan();
scanCtrl.init(send);
```

with:

```js
var scanCtrl = setupScan(send);
```

- [ ] **Step 5: Commit**

```bash
git add web/modules/map.js web/app.module.js
git commit -m "chore(web): remove unused map.js exports, legacy mapWalls fallback, simplify setupScan API"
```

---

## Task 6: Fix passInput keydown listener accumulation in panels.js

**Files:**
- Modify: `web/modules/panels.js`

- [ ] **Step 1: Move one-time modal listeners into setupTransport**

In `web/modules/panels.js`, replace the entire `setupTransport` function (lines 107–135):

```js
export function setupTransport() {
    var badge = document.getElementById("transport-badge");
    var menu = document.getElementById("transport-menu");

    badge.addEventListener("click", function (e) {
        e.stopPropagation();
        menu.classList.toggle("hidden");
    });
    document.addEventListener("click", function () { menu.classList.add("hidden"); });

    menu.addEventListener("click", function (e) {
        var btn = e.target.closest("[data-transport]");
        if (!btn) return;
        var mode = btn.dataset.transport;
        menu.classList.add("hidden");

        if (mode === "wifi" || mode === "wifi-fw") {
            var host = prompt("MechDog WiFi IP:", "192.168.1.163");
            if (!host) return;
            send({ type: "cmd_transport", mode: mode, wifi_host: host });
        } else if (mode === "wifi-setup") {
            showWifiModal();
            return;
        } else {
            send({ type: "cmd_transport", mode: mode });
        }
        badge.textContent = "..."; badge.className = "transport-badge switching";
    });
}
```

with:

```js
export function setupTransport() {
    var badge = document.getElementById("transport-badge");
    var menu = document.getElementById("transport-menu");
    var modal = document.getElementById("wifi-modal");
    var passInput = document.getElementById("wifi-pass");

    badge.addEventListener("click", function (e) {
        e.stopPropagation();
        menu.classList.toggle("hidden");
    });
    document.addEventListener("click", function () { menu.classList.add("hidden"); });

    menu.addEventListener("click", function (e) {
        var btn = e.target.closest("[data-transport]");
        if (!btn) return;
        var mode = btn.dataset.transport;
        menu.classList.add("hidden");

        if (mode === "wifi" || mode === "wifi-fw") {
            var host = prompt("MechDog WiFi IP:", "192.168.1.163");
            if (!host) return;
            send({ type: "cmd_transport", mode: mode, wifi_host: host });
        } else if (mode === "wifi-setup") {
            showWifiModal();
            return;
        } else {
            send({ type: "cmd_transport", mode: mode });
        }
        badge.textContent = "..."; badge.className = "transport-badge switching";
    });

    // One-time modal listeners — registered here, not inside showWifiModal
    document.getElementById("wifi-modal-cancel").addEventListener("click", function () {
        modal.classList.add("hidden");
    });
    passInput.addEventListener("keydown", function (e) {
        if (e.key === "Enter") document.getElementById("wifi-modal-connect").click();
    });
    modal.addEventListener("click", function (e) {
        if (e.target === modal) modal.classList.add("hidden");
    });
}
```

- [ ] **Step 2: Remove the duplicate listener registrations from showWifiModal**

In `showWifiModal`, replace (lines 160–193):

```js
function showWifiModal() {
    var modal = document.getElementById("wifi-modal");
    var ssidInput = document.getElementById("wifi-ssid");
    var passInput = document.getElementById("wifi-pass");
    var status = document.getElementById("wifi-modal-status");
    ssidInput.value = "";
    passInput.value = "";
    status.className = "hidden";
    status.textContent = "";
    modal.classList.remove("hidden");
    ssidInput.focus();

    document.getElementById("wifi-modal-cancel").onclick = function () {
        modal.classList.add("hidden");
    };

    document.getElementById("wifi-modal-connect").onclick = function () {
        var ssid = ssidInput.value.trim();
        if (!ssid) { ssidInput.focus(); return; }
        status.textContent = "Connecting...";
        status.className = "pending";
        send({ type: "cmd_wifi_setup", ssid: ssid, password: passInput.value });
        var badge = document.getElementById("transport-badge");
        badge.textContent = "SETUP..."; badge.className = "transport-badge switching";
    };

    passInput.addEventListener("keydown", function (e) {
        if (e.key === "Enter") document.getElementById("wifi-modal-connect").click();
    });

    modal.onclick = function (e) {
        if (e.target === modal) modal.classList.add("hidden");
    };
}
```

with:

```js
function showWifiModal() {
    var modal = document.getElementById("wifi-modal");
    var ssidInput = document.getElementById("wifi-ssid");
    var passInput = document.getElementById("wifi-pass");
    var status = document.getElementById("wifi-modal-status");
    ssidInput.value = "";
    passInput.value = "";
    status.className = "hidden";
    status.textContent = "";
    modal.classList.remove("hidden");
    ssidInput.focus();

    document.getElementById("wifi-modal-connect").onclick = function () {
        var ssid = ssidInput.value.trim();
        if (!ssid) { ssidInput.focus(); return; }
        status.textContent = "Connecting...";
        status.className = "pending";
        send({ type: "cmd_wifi_setup", ssid: ssid, password: passInput.value });
        var badge = document.getElementById("transport-badge");
        badge.textContent = "SETUP..."; badge.className = "transport-badge switching";
    };
}
```

- [ ] **Step 3: Commit**

```bash
git add web/modules/panels.js
git commit -m "fix(web): move wifi modal listeners to one-time setup to prevent keydown accumulation"
```

---

## Task 7: Remove dead animation code from gait.js

**Files:**
- Modify: `web/dog3d/gait.js`

- [ ] **Step 1: Remove getPoseNames export**

Delete the entire `getPoseNames` function (lines 18–20):

```js
export function getPoseNames() {
    return Object.keys(POSES);
}
```

- [ ] **Step 2: Remove POSES export**

Replace (line 7):

```js
export var POSES = {
```

with:

```js
var POSES = {
```

(The variable stays — it's used internally by `setPose`. Only the export keyword is removed.)

- [ ] **Step 3: Remove state.currentAction from setPose**

Replace the `setPose` function (lines 22–30):

```js
export function setPose(name) {
    var angles = POSES[name];
    if (!angles) return;
    state.currentPoseName = name;
    state.targetPose = angles.slice();
    state.activePose = angles.slice();
    state.currentAction = null;
    state.currentMotion = "stop";
}
```

with:

```js
export function setPose(name) {
    var angles = POSES[name];
    if (!angles) return;
    state.currentPoseName = name;
    state.targetPose = angles.slice();
    state.activePose = angles.slice();
    state.currentMotion = "stop";
}
```

- [ ] **Step 4: Remove wave arm from animateAction**

Replace the tail of `animateAction` (lines 99–108):

```js
    var a = state.currentAction;
    if (a === 1) {
        setLeg("fl", -0.8, -1.2);
        setLeg("fr", STAND_HIP, STAND_KNEE);
        setLeg("rl", STAND_HIP * 1.3, STAND_KNEE * 0.8);
        setLeg("rr", STAND_HIP * 1.3, STAND_KNEE * 0.8);
    } else {
        setAllLegs(STAND_HIP, STAND_KNEE);
    }
}
```

with:

```js
    setAllLegs(STAND_HIP, STAND_KNEE);
}
```

- [ ] **Step 5: Simplify animateGait guards — remove currentAction checks**

Replace (lines 116–132):

```js
    if (speed === 0) {
        state.walkPhase = 0;
        state.bodyBounce = 0;
        // Pose interpolation or action takes priority over standing reset
        if (state.targetPose || state.currentAction != null) {
            animateAction();
            return;
        }
        setAllLegs(STAND_HIP, STAND_KNEE);
        return;
    }

    if (state.currentAction != null || state.targetPose) {
        animateAction();
        state.bodyBounce = 0;
        return;
    }
```

with:

```js
    if (speed === 0) {
        state.walkPhase = 0;
        state.bodyBounce = 0;
        if (state.targetPose) {
            animateAction();
            return;
        }
        setAllLegs(STAND_HIP, STAND_KNEE);
        return;
    }

    if (state.targetPose) {
        animateAction();
        state.bodyBounce = 0;
        return;
    }
```

- [ ] **Step 6: Commit**

```bash
git add web/dog3d/gait.js
git commit -m "chore(web): remove getPoseNames, POSES export, dead wave arm, and currentAction from gait.js"
```

---

## Task 8: Clean up dog3d/index.js

**Files:**
- Modify: `web/dog3d/index.js`

- [ ] **Step 1: Remove getPoseNames and buildWallsFromSegments imports**

Replace (lines 4 and 6):

```js
import { animateGait, setPose, clearPose, getPoseNames, applySimJoints } from './gait.js';
```

with:

```js
import { animateGait, setPose, clearPose, applySimJoints } from './gait.js';
```

Replace (line 6):

```js
import { clearWalls, buildWallsFromSegments, buildWallsFromChains } from './walls.js';
```

with:

```js
import { clearWalls, buildWallsFromChains } from './walls.js';
```

- [ ] **Step 2: Stop storing animationId (keep the RAF call)**

Replace (line 88):

```js
    state.animationId = requestAnimationFrame(animate);
```

with:

```js
    requestAnimationFrame(animate);
```

- [ ] **Step 3: Remove state.currentAction from reset()**

In the `reset` function (line 177), remove:

```js
        state.currentAction = null;
```

- [ ] **Step 4: Remove state.isFallen from setFallen()**

In `setFallen`, replace (lines 203–205):

```js
    setFallen: function (fallen) {
        state.isFallen = fallen;
        if (!state.dogGroup) return;
```

with:

```js
    setFallen: function (fallen) {
        if (!state.dogGroup) return;
```

- [ ] **Step 5: Remove legacy walls branch from setMapData()**

Replace (lines 187–193):

```js
    setMapData: function (data) {
        if (data && data.chains && data.chains.length > 0) {
            buildWallsFromChains(data.chains);
        } else if (data && data.walls) {
            buildWallsFromSegments(data.walls);
        }
    },
```

with:

```js
    setMapData: function (data) {
        if (data && data.chains && data.chains.length > 0) {
            buildWallsFromChains(data.chains);
        }
    },
```

- [ ] **Step 6: Commit**

```bash
git add web/dog3d/index.js
git commit -m "chore(web): clean up dog3d/index.js — remove getPoseNames, buildWallsFromSegments, animationId, isFallen, currentAction"
```

---

## Task 9: Remove dead fields from state.js

**Files:**
- Modify: `web/dog3d/state.js`

- [ ] **Step 1: Remove animationId, isFallen, and currentAction from state object**

Replace (lines 24–66):

```js
export const state = {
    scene: null,
    camera: null,
    renderer: null,
    dogGroup: null,
    container: null,
    animationId: null,
    legs: {},

    // Camera
    isDragging: false,
    prevMouse: { x: 0, y: 0 },
    cameraAngle: { thetaOffset: 0, phi: Math.PI / 5, radius: 8 },
    camYaw: 0,
    camTargetX: 0,
    camTargetZ: 0,

    // Telemetry targets (smoothed)
    targetPitch: 0, targetRoll: 0,
    currentPitch: 0, currentRoll: 0,
    targetX: 0, targetZ: 0, targetYaw: 0,
    currentX: 0, currentZ: 0, currentYaw: 0,
    currentMotion: "stop",
    currentAction: null,
    ultraDistance: null,
    walkPhase: 0,
    isFallen: false,

    // Gait
    bodyBounce: 0,

    // Pose
    currentPoseName: "stand",
    activePose: null,
    targetPose: null,

    // Sim joints
    simJoints: null,

    // Ultrasonic
    ultraBeam: null,
    ultraHit: null,
};
```

with:

```js
export const state = {
    scene: null,
    camera: null,
    renderer: null,
    dogGroup: null,
    container: null,
    legs: {},

    // Camera
    isDragging: false,
    prevMouse: { x: 0, y: 0 },
    cameraAngle: { thetaOffset: 0, phi: Math.PI / 5, radius: 8 },
    camYaw: 0,
    camTargetX: 0,
    camTargetZ: 0,

    // Telemetry targets (smoothed)
    targetPitch: 0, targetRoll: 0,
    currentPitch: 0, currentRoll: 0,
    targetX: 0, targetZ: 0, targetYaw: 0,
    currentX: 0, currentZ: 0, currentYaw: 0,
    currentMotion: "stop",
    ultraDistance: null,
    walkPhase: 0,

    // Gait
    bodyBounce: 0,

    // Pose
    currentPoseName: "stand",
    activePose: null,
    targetPose: null,

    // Sim joints
    simJoints: null,

    // Ultrasonic
    ultraBeam: null,
    ultraHit: null,
};
```

- [ ] **Step 2: Commit**

```bash
git add web/dog3d/state.js
git commit -m "chore(web): remove write-only state fields: animationId, isFallen, currentAction"
```

---

## Task 10: Remove write-only jointMarkers from overlay.js

**Files:**
- Modify: `web/dog3d/overlay.js`

- [ ] **Step 1: Remove the jointMarkers variable declaration**

Replace (line 6):

```js
var jointMarkers = {};
```

Delete this line entirely.

- [ ] **Step 2: Remove jointMarkers assignment in buildOverlay**

In `buildOverlay`, replace (line 118):

```js
        jointMarkers[name] = { hip: hipMarker, knee: kneeMarker, foot: footMarker };
```

Delete this line entirely.

- [ ] **Step 3: Commit**

```bash
git add web/dog3d/overlay.js
git commit -m "chore(web): remove write-only jointMarkers object from overlay.js"
```

---

## Task 11: Remove buildWallsFromSegments from walls.js

**Files:**
- Modify: `web/dog3d/walls.js`

- [ ] **Step 1: Remove WALL_THICKNESS constant**

Delete (line 6):

```js
var WALL_THICKNESS = 0.08 * S;
```

- [ ] **Step 2: Remove buildWallsFromSegments function**

Delete the entire function (lines 136–172):

```js
// Legacy: build from line segments (fallback)
export function buildWallsFromSegments(walls) {
    clearWalls();
    if (!walls || walls.length < 1) return;
    if (!wallTexture) wallTexture = createWallTexture();

    var wallMat = new THREE.MeshStandardMaterial({
        map: wallTexture, roughness: 0.85, metalness: 0.05, side: THREE.DoubleSide,
    });
    var edgeMat = new THREE.LineBasicMaterial({
        color: 0x555555, transparent: true, opacity: 0.4,
    });

    for (var i = 0; i < walls.length; i++) {
        var w = walls[i];
        var sx1 = w.x1 * S, sz1 = w.y1 * S;
        var sx2 = w.x2 * S, sz2 = w.y2 * S;
        var dx = sx2 - sx1, dz = sz2 - sz1;
        var len = Math.sqrt(dx * dx + dz * dz);
        if (len < 0.01) continue;

        var cx = (sx1 + sx2) / 2, cz = (sz1 + sz2) / 2;
        var angle = Math.atan2(dz, dx);
        var height = (w.height || 0.2) * S;

        var geo = new THREE.BoxGeometry(len, height, WALL_THICKNESS);
        var mesh = new THREE.Mesh(geo, wallMat);
        mesh.position.set(cx, height / 2, cz);
        mesh.rotation.y = -angle;
        mesh.castShadow = true; mesh.receiveShadow = true;
        state.scene.add(mesh); wallMeshes.push(mesh);

        var edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat);
        edges.position.copy(mesh.position); edges.rotation.copy(mesh.rotation);
        state.scene.add(edges); wallMeshes.push(edges);
    }
}
```

- [ ] **Step 3: Commit**

```bash
git add web/dog3d/walls.js
git commit -m "chore(web): remove legacy buildWallsFromSegments and WALL_THICKNESS"
```

---

## Task 12: Fix CSS selector for ultrasonic warn/danger text colors

**Files:**
- Modify: `web/style.css`

- [ ] **Step 1: Fix selector to match #ultra-val**

`#ultra-val` lives inside `#telem-overlay`, not `.stat`. The selectors `.stat .warn-text` and `.stat .danger-text` never match it. Broaden them to global class selectors.

In `web/style.css`, replace (lines 82–83):

```css
.stat .warn-text { color: var(--amber); }
.stat .danger-text { color: var(--red); }
```

with:

```css
.stat .warn-text, .warn-text { color: var(--amber); }
.stat .danger-text, .danger-text { color: var(--red); }
```

- [ ] **Step 2: Commit**

```bash
git add web/style.css
git commit -m "fix(web): broaden warn-text/danger-text selectors to match #ultra-val in telem-overlay"
```

---

## Task 13: Add lock_denied handler

**Files:**
- Modify: `web/app.module.js`

- [ ] **Step 1: Add lock_denied case to handleMessage**

In `web/app.module.js`, in `handleMessage`, after the `version` case, add a new `else if` before the closing brace. The version block currently ends at line 159. Add after it:

Replace (lines 156–160):

```js
    } else if (msg.type === "version") {
        if (window._appVersion && msg.hash !== window._appVersion) location.reload();
        window._appVersion = msg.hash;
    }
}
```

with:

```js
    } else if (msg.type === "version") {
        if (window._appVersion && msg.hash !== window._appVersion) location.reload();
        window._appVersion = msg.hash;
    } else if (msg.type === "lock_denied") {
        var alertEl = document.getElementById("fall-alert");
        alertEl.textContent = msg.operator ? "Control held by " + msg.operator : "Control request denied";
        alertEl.classList.remove("hidden");
        setTimeout(function () {
            alertEl.classList.add("hidden");
            alertEl.textContent = "FALL DETECTED";
        }, 3000);
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add web/app.module.js
git commit -m "feat(web): add lock_denied handler — show 3-second notification using fall-alert element"
```

---

## Final Verification

- [ ] **Start the server**

```bash
cd /Users/gwen/workspace/bark-buddy/host
python server.py --sim
```

Open `http://localhost:8765` in a browser. Check the browser console for errors (should be zero).

- [ ] **Check deleted UI elements are gone**

- Patrol Demo button — absent from Operations section
- Stop Patrol button — absent
- Set Default button — absent from Actions section
- Sim+ — absent from transport dropdown

- [ ] **Check core features still work**

- D-pad buttons move the simulated dog (3D view updates)
- Balance toggle button shows Balance: ON/OFF and updates the header stat
- `b` key toggles balance (keyboard shortcut via module-scoped `balanceEnabled`)
- Wave button sends `cmd_action` (visible in browser network tab or server log)
- Scan Area button starts a scan; Stop Scan stops it; Clear Map clears it
- 3D dog renders; kinematics overlay (K key) toggles on/off
- Wall mesh from `map_data` renders correctly

- [ ] **Check transport switcher**

Click the SIM badge — dropdown shows: USB (stock), USB (firmware), WiFi (stock), WiFi (firmware), Setup WiFi..., Sim (classic). **No Sim+ option.**

- [ ] **Check WiFi modal**

Click transport badge → Setup WiFi... → modal opens. Type in password field, press Enter → connect fires. Close and reopen modal, press Enter again → fires exactly once (no accumulation).

- [ ] **Check ultrasonic warn/danger colors**

In browser console: `document.getElementById("ultra-val").className = "status-value danger-text"` — the text should turn red. `"status-value warn-text"` — amber. (In sim mode with actual sonar noise enabled, values below 100mm will trigger automatically.)

- [ ] **Check lock_denied notification**

Open two browser tabs at `http://localhost:8765`. In tab 1, click Take Control. In tab 2, click Take Control — tab 2 should show a temporary banner: "Control held by Operator" (or the name from `?name=`). It should auto-hide after 3 seconds and the fall-alert text should revert to "FALL DETECTED".
