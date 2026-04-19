# Phase 1 — Kernel Cut Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cut the repo to the minimal motion kernel by removing all scan/map/calibration code, secondary UIs, and stale docs — preserving a `pre-reshape` branch for reference.

**Architecture:** Branch first, clean up all live references to attic code, verify the server starts clean, then delete the attic files. Dead reckoning (`dead_reckoning.py` + transport usages) is deferred to Phase 3 to avoid touching `FirmwareTransport` here.

**Tech Stack:** Python 3.11+, aiohttp, vanilla JS (ES modules), Three.js r128, Make/clang++ (mock build)

---

### Task 1: Create branch + baseline verification

**Files:** none modified

- [ ] **Step 1: Create the pre-reshape branch**

```bash
git branch pre-reshape
git branch --list pre-reshape   # confirm it exists
```

Expected: `pre-reshape` listed with no errors.

- [ ] **Step 2: Run firmware unit tests as baseline**

```bash
cd firmware/test && make
./test_ik && ./test_transform && ./test_balance && ./test_offsets && ./test_gait_ik && ./test_servos
cd ../..
```

Expected: all 6 binaries exit 0. If any fail, stop and fix before proceeding.

- [ ] **Step 3: Build the native mock**

```bash
cd firmware/test && make bark-mock && cd ../..
```

Expected: `firmware/test/bark-mock` exists and build exits 0.

---

### Task 2: Import-graph sweep

**Files:** read-only (just grepping)

- [ ] **Step 1: Find all references to attic modules in Python**

```bash
grep -rn "from behaviors.scan\|from behaviors.map_store\|from behaviors.wall_fit\|from behaviors.wall_mesh\|from behaviors.octree\|dead_reckoning\|import ScanBehavior\|import MapStore\|import WallFit\|import WallMesh\|import Octree" host/ bark_cli.py
```

Expected: matches only in `host/server.py` (lines 22-23). No others.

- [ ] **Step 2: Find JS imports of attic modules**

```bash
grep -rn "from.*map\.js\|from.*sonar\.js\|from.*walls\.js\|modules/map\|dog3d/sonar\|dog3d/walls" web/
```

Expected: matches in `web/app.module.js` (line 6) and `web/dog3d/index.js` (lines 5-6).

- [ ] **Step 3: Find remaining references to overlay lifecycle indicator**

```bash
grep -rn "updateLifecycleIndicator\|lifecycle-badge\|lifecycle_badge" web/
```

Expected: matches in `web/dog3d/overlay.js` (line 134) and `web/app.module.js` (line 3, 69).

- [ ] **Step 4: Find stock-flash references in bark_cli.py**

```bash
grep -n "stock\|STOCK_FIRMWARE" bark_cli.py
```

Expected: `STOCK_FIRMWARE_BIN` constant, `_do_stock_flash`, and stock subparser.

---

### Task 3: server.py — strip scan/map imports and __init__

**Files:**
- Modify: `host/server.py`

- [ ] **Step 1: Remove MapStore and ScanBehavior imports**

Find lines 22-23:
```python
from behaviors.map_store import MapStore
from behaviors.scan import ScanBehavior
```

Delete both lines. Save.

- [ ] **Step 2: Remove scan/map attributes from `__init__`**

Find in `Server.__init__` (around lines 95-96):
```python
        self._scan = ScanBehavior(transport)
        self._map = MapStore()
```

Delete both lines. The line `self._mode = "remote"  # remote | scan` becomes:
```python
        self._mode = "remote"
```

(just remove the trailing comment)

- [ ] **Step 3: Remove `_tuning_handler` and `_gait_handler` methods**

Find and delete these two methods:
```python
    async def _tuning_handler(self, request):
        return web.FileResponse(os.path.join(self._web_dir, "tuning.html"))

    async def _gait_handler(self, request):
        path = os.path.join(self._web_dir, "gait.html")
        if not os.path.exists(path):
            raise web.HTTPNotFound()
        return web.FileResponse(path)
```

- [ ] **Step 4: Remove `/tuning` and `/gait` routes from `start()`**

Find in `start()`:
```python
        app.router.add_get("/tuning", self._tuning_handler)
        app.router.add_get("/gait", self._gait_handler)
```

Delete both lines.

- [ ] **Step 5: Remove scan callback registration from `_on_startup`**

Find in `_on_startup`:
```python
        # Register scan callbacks
        self._scan.on_point(self._on_scan_point)
        self._scan.on_complete(self._on_scan_complete)
```

Delete all three lines (including the comment).

- [ ] **Step 6: Remove scan cancel from `_on_shutdown`**

Find in `_on_shutdown`:
```python
        if self._scan.running:
            await self._scan.cancel()
```

Delete both lines.

- [ ] **Step 7: Remove scan re-init from `_replace_transport`**

Find in `_replace_transport`:
```python
            # Swap in new transport, balance, scan, button_engage
            from behaviors.balance import BalanceLayer
            from behaviors.button_engage import ButtonEngageBehavior
            from behaviors.scan import ScanBehavior
            self._transport = new_transport
            self._register_transport_callbacks()
            self._balance = BalanceLayer(new_transport)
            self._scan = ScanBehavior(new_transport)
            self._button_engage = ButtonEngageBehavior(
                new_transport,
                lambda: self._engaged,
                lambda v: setattr(self, '_engaged', v),
                self._is_locked,
            )
            self._scan.on_point(self._on_scan_point)
            self._scan.on_complete(self._on_scan_complete)
```

Replace with:
```python
            # Swap in new transport, balance, button_engage
            from behaviors.balance import BalanceLayer
            from behaviors.button_engage import ButtonEngageBehavior
            self._transport = new_transport
            self._register_transport_callbacks()
            self._balance = BalanceLayer(new_transport)
            self._button_engage = ButtonEngageBehavior(
                new_transport,
                lambda: self._engaged,
                lambda v: setattr(self, '_engaged', v),
                self._is_locked,
            )
```

- [ ] **Step 8: Delete `_add_live_point` method (dead code — never called)**

Find and delete the entire method:
```python
    async def _add_live_point(self, distance_mm: int) -> None:
        """Add an ultrasonic reading as a map point using current position/heading."""
        import math
        pos = self._transport.get_position()
        heading = self._transport.get_heading()
        if pos is None or heading is None:
            return
        rad = math.radians(heading)
        dist_m = distance_mm / 1000.0
        x = pos[0] + dist_m * math.cos(rad)
        y = pos[1] + dist_m * math.sin(rad)
        point = self._map.add_point(x=x, y=y, z=0.09, distance_mm=distance_mm, source="ultrasonic")
        # Broadcast live point for real-time 2D map
        if point and self._ws_clients:
            await self._broadcast({
                "type": "scan_point",
                "x": round(x, 3),
                "y": round(y, 3),
                "distance_mm": distance_mm,
                "progress": 0,
                "confidence": round(point.confidence, 2),
            })
```

- [ ] **Step 9: Delete `_on_scan_point`, `_scan_task_done`, `_on_scan_complete` methods**

Find and delete:
```python
    async def _on_scan_point(self, point, progress: int):
        """Broadcast each scan point as it's captured."""
        await self._broadcast({
            "type": "scan_point",
            "angle": round(point.angle, 1),
            "distance_mm": point.distance_mm,
            "x": round(point.x, 3),
            "y": round(point.y, 3),
            "progress": progress,
        })

    def _scan_task_done(self, task: asyncio.Task):
        """Handle scan task completion, including unexpected errors."""
        exc = task.exception() if not task.cancelled() else None
        if exc:
            logger.error("Scan task failed: %s", exc)
            self._mode = "remote"
            asyncio.create_task(self._broadcast_status())

    async def _on_scan_complete(self, result):
        """Store scan result and broadcast the full map."""
        self._map.add_scan(result)
        self._mode = "remote"
        await self._broadcast({"type": "scan_complete"})
        await self._broadcast_status()
        await self._broadcast({
            "type": "map_data",
            **self._map.to_dict(),
        })
```

- [ ] **Step 10: Verify Python syntax**

```bash
cd host && python3 -c "import server" 2>&1 && echo OK
```

Expected: `OK` (no ImportError or SyntaxError). If it errors, the attic files are still present — that's fine, just check for SyntaxError only.

```bash
cd host && python3 -m py_compile server.py && echo OK
```

Expected: `OK`.

- [ ] **Step 11: Commit**

```bash
git add host/server.py
git commit -m "refactor(server): remove scan/map behaviors — imports, init, methods, routes"
```

---

### Task 4: server.py — strip scan from message handler and status broadcast

**Files:**
- Modify: `host/server.py`

- [ ] **Step 1: Remove `"cmd_scan"` from the lock-gated commands tuple**

Find:
```python
        if msg_type in ("cmd_move", "cmd_stand", "cmd_balance",
                         "cmd_engage", "cmd_scan"):
```

Replace with:
```python
        if msg_type in ("cmd_move", "cmd_stand", "cmd_balance",
                         "cmd_engage"):
```

- [ ] **Step 2: Remove scan mode guard from `cmd_move`**

Find:
```python
        if msg_type == "cmd_move":
            if self._mode == "scan":
                return
            direction = msg.get("direction", "stop")
```

Replace with:
```python
        if msg_type == "cmd_move":
            direction = msg.get("direction", "stop")
```

- [ ] **Step 3: Remove scan mode guard from `cmd_stand`**

Find:
```python
        elif msg_type == "cmd_stand":
            if self._mode == "scan":
                return
            await self._transport.send_json({"type": "cmd_stand"})
```

Replace with:
```python
        elif msg_type == "cmd_stand":
            await self._transport.send_json({"type": "cmd_stand"})
```

- [ ] **Step 4: Delete `cmd_scan` handler block**

Find and delete the entire block:
```python
        elif msg_type == "cmd_scan":
            action = msg.get("action", "start")
            if action == "start" and not self._scan.running:
                # Use transport's dead-reckoned position as scan origin
                ox, oy, heading = 0.0, 0.0, 0.0
                pos = self._transport.get_position()
                if pos is not None:
                    ox, oy = pos[0], pos[1]
                h = self._transport.get_heading()
                if h is not None:
                    heading = h
                self._mode = "scan"
                await self._broadcast_status()
                self._scan.start(
                    origin_x=ox, origin_y=oy,
                    origin_heading=heading,
                    done_callback=self._scan_task_done,
                )
            elif action == "stop":
                await self._scan.cancel()
                self._mode = "remote"
                await self._broadcast_status()
```

- [ ] **Step 5: Delete `cmd_map` handler block**

Find and delete:
```python
        # --- Non-gated commands ---
        elif msg_type == "cmd_map":
            action = msg.get("action", "get")
            if action == "get":
                await self._broadcast({
                    "type": "map_data",
                    **self._map.to_dict(),
                })
            elif action == "clear":
                self._map.clear()
                await self._broadcast({
                    "type": "map_data",
                    **self._map.to_dict(),
                })
```

- [ ] **Step 6: Remove `"scanning"` from the initial status in `_ws_handler`**

Find (around line 484 in `_ws_handler`):
```python
            "scanning": self._scan.running,
```

Delete that line.

- [ ] **Step 7: Strip scan-related fields from `_broadcast_status`**

Find in `_broadcast_status`:
```python
            "scanning": self._scan.running,
```
Delete that line.

Find and delete the lifecycle block:
```python
        if self._transport:
            _r = self._transport.get_ramping()
            _e = self._transport.get_engaged()
            status["lifecycle"] = "ramping" if _r else ("active" if _e else "disengaged")
        else:
            status["lifecycle"] = "unknown"
        if self._scan.running:
            status["scan_progress"] = self._scan.progress
```

- [ ] **Step 8: Clean `_telemetry_loop` — remove point cloud and wall regen**

Find in `_telemetry_loop`:
```python
                wall_regen_interval = 1.0
```
and
```python
                last_wall_regen = 0.0
```
Delete both variable declarations.

Find and delete the point cloud maintenance block:
```python
                # Point cloud maintenance
                if not self._scan.running:
                    self._map.consolidate()
                    self._map.decay_tick()

                # Wall regen broadcast
                if now - last_wall_regen >= wall_regen_interval and self._ws_clients:
                    await self._broadcast({
                        "type": "map_data",
                        **self._map.to_dict(),
                    })
                    last_wall_regen = now
```

- [ ] **Step 9: Verify Python syntax**

```bash
cd host && python3 -m py_compile server.py && echo OK
```

Expected: `OK`.

- [ ] **Step 10: Commit**

```bash
git add host/server.py
git commit -m "refactor(server): strip cmd_scan, cmd_map, scan status from message handler and telemetry loop"
```

---

### Task 5: bark_cli.py — remove stock flash subcommand

**Files:**
- Modify: `bark_cli.py`

- [ ] **Step 1: Remove `STOCK_FIRMWARE_BIN` constant**

Find at the top of `bark_cli.py`:
```python
STOCK_FIRMWARE_BIN = FIRMWARE_DIR / "stock_firmware.bin"
```

Delete that line.

- [ ] **Step 2: Delete `_do_stock_flash` function**

Find and delete the entire function:
```python
def _do_stock_flash(args):
    if not STOCK_FIRMWARE_BIN.exists():
        print(f"[flash stock] ERROR: {STOCK_FIRMWARE_BIN} not found.")
        print("[flash stock] Expected Hiwonder Mechdog_250505_0x000.bin (~1.7 MB) at that path.")
        sys.exit(1)
    _ensure_host_importable()
    from server import find_serial_port
    port = getattr(args, "serial", None) or find_serial_port()
    if not port:
        print("[flash stock] ERROR: no USB serial port found. "
              "Pass --serial /dev/cu.usbserial-XXXX.")
        sys.exit(1)
    baud = getattr(args, "baud", 460800)
    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32",
        "--port", port,
        "--baud", str(baud),
        "write_flash",
        "--flash_mode", "dio",
        "--flash_size", "4MB",
        "0x0", str(STOCK_FIRMWARE_BIN),
    ]
    print(f"[flash stock] Writing {STOCK_FIRMWARE_BIN.name} "
          f"({STOCK_FIRMWARE_BIN.stat().st_size} bytes) @ 0x0 on {port} ...")
    print("[flash stock] Saved actions (LittleFS/vfs) not included; servos will still engage.")
    print("[flash stock] To recover custom firmware afterwards: bark flash")
    result = subprocess.run(cmd)
    sys.exit(result.returncode)
```

- [ ] **Step 3: Remove stock flash branch from `cmd_flash`**

Find in `cmd_flash`:
```python
    if getattr(args, "flash_target", None) == "stock":
        _do_stock_flash(args)
        return
```

Delete all three lines.

- [ ] **Step 4: Remove the `flash stock` subparser**

Find:
```python
    flash_targets = p_flash.add_subparsers(dest="flash_target")
    p_flash_stock = flash_targets.add_parser(
        "stock",
        help="Flash Hiwonder vendor stock firmware via esptool (servo sanity check)",
    )
    p_flash_stock.add_argument("--serial", default=None,
                               help="Serial port override (e.g. /dev/cu.usbserial-10)")
    p_flash_stock.add_argument("--baud", type=int, default=460800,
                               help="esptool baud rate (default: 460800)")
```

Delete all those lines.

- [ ] **Step 5: Verify**

```bash
python3 bark_cli.py --help
python3 bark_cli.py flash --help
```

Expected: `bark flash` help shows `--usb`, `--wifi`, `--fw-tcp` but no `stock` subcommand.

- [ ] **Step 6: Commit**

```bash
git add bark_cli.py
git commit -m "refactor(cli): remove bark flash stock subcommand"
```

---

### Task 6: web/dog3d/overlay.js — remove dead lifecycle indicator

**Files:**
- Modify: `web/dog3d/overlay.js`

- [ ] **Step 1: Delete `updateLifecycleIndicator` function**

Find and delete the entire function (lines 134-140):
```javascript
export function updateLifecycleIndicator(lifecycle) {
    var badge = document.getElementById('lifecycle-badge');
    if (!badge) return;
    badge.className = badge.className.replace(/lifecycle-\w+/g, '').trim();
    badge.classList.add('lifecycle-badge', 'lifecycle-' + lifecycle);
    badge.textContent = lifecycle;
}
```

- [ ] **Step 2: Verify overlay.js still exports `toggleOverlay` and `updateOverlay`**

```bash
grep "^export function" web/dog3d/overlay.js
```

Expected:
```
export function updateOverlay() {
export function toggleOverlay(show) {
```

Only those two. `updateLifecycleIndicator` should not appear.

- [ ] **Step 3: Commit**

```bash
git add web/dog3d/overlay.js
git commit -m "refactor(web): remove dead updateLifecycleIndicator from overlay.js"
```

---

### Task 7: web/dog3d/index.js — remove sonar and walls

**Files:**
- Modify: `web/dog3d/index.js`

- [ ] **Step 1: Remove sonar and walls imports**

Find lines 5-6:
```javascript
import { initUltraHit, updateUltraBeam } from './sonar.js';
import { clearWalls, buildWallsFromChains } from './walls.js';
```

Delete both lines.

- [ ] **Step 2: Remove `initUltraHit()` call in `init()`**

Find:
```javascript
    initUltraHit();
```

Delete that line.

- [ ] **Step 3: Remove `updateUltraBeam()` call in `animate()`**

Find:
```javascript
        updateUltraBeam();
```

Delete that line.

- [ ] **Step 4: Remove `clearWalls()` call from `reset()`**

Find in the `reset` method:
```javascript
        clearWalls();
```

Delete that line.

- [ ] **Step 5: Stub out `setMapData` (remove walls dependency)**

Find the `setMapData` method:
```javascript
    setMapData: function (data) {
        if (data && data.chains && data.chains.length > 0) {
            buildWallsFromChains(data.chains);
        }
    },
```

Replace with empty stub:
```javascript
    setMapData: function (_data) {
        // mapping wall rendering removed — will be rebuilt with SLAM
    },
```

- [ ] **Step 6: Verify no remaining sonar/walls references**

```bash
grep -n "sonar\|walls\|initUltra\|updateUltra\|clearWalls\|buildWalls" web/dog3d/index.js
```

Expected: no matches.

- [ ] **Step 7: Commit**

```bash
git add web/dog3d/index.js
git commit -m "refactor(web): remove sonar/walls from dog3d — stub setMapData"
```

---

### Task 8: web/app.module.js — strip scan/map/lifecycle UI

**Files:**
- Modify: `web/app.module.js`

- [ ] **Step 1: Remove attic imports**

Find line 3:
```javascript
import { updateLifecycleIndicator } from './dog3d/overlay.js';
```
Delete it.

Find line 6:
```javascript
import { dogMapState, addScanPoint, renderFullMap, drawMap, setupScan } from './modules/map.js';
```
Delete it.

- [ ] **Step 2: Remove lifecycle indicator call from `updateStatus`**

Find in `updateStatus`:
```javascript
    if (msg.lifecycle != null) {
        updateLifecycleIndicator(msg.lifecycle);
    }
```
Delete all three lines.

- [ ] **Step 3: Remove scan_progress display from `updateStatus`**

Find:
```javascript
    if (msg.scan_progress != null) {
        document.getElementById("scan-progress-fill").style.width = msg.scan_progress + "%";
        document.getElementById("scan-progress-text").textContent = msg.scan_progress + "%";
    }
```
Delete all four lines.

- [ ] **Step 4: Remove scan mode indicator from `updateStatus`**

Find:
```javascript
    if (msg.mode != null) {
        document.getElementById("mode-val").textContent = msg.mode;
        scanCtrl.setScanRunning(msg.mode === "scan");
    }
```
Replace with:
```javascript
    if (msg.mode != null) {
        document.getElementById("mode-val").textContent = msg.mode;
    }
```

- [ ] **Step 5: Strip map state from `telem_odometry` handler**

Find:
```javascript
    } else if (msg.type === "telem_odometry") {
        Dog3D.updateOdometry(msg);
        dogMapState.x = msg.x || 0;
        dogMapState.y = msg.y || 0;
        dogMapState.heading = msg.heading || 0;
        dogMapState.motion = msg.motion || "stop";
        updateMotionIndicator(msg.motion);
        if (msg.heading != null) {
            var h = Math.round(msg.heading) % 360;
            if (h < 0) h += 360;
            document.getElementById("heading-val").textContent = h + "°";
        }
        drawMap();
```
Replace with:
```javascript
    } else if (msg.type === "telem_odometry") {
        Dog3D.updateOdometry(msg);
        updateMotionIndicator(msg.motion);
        if (msg.heading != null) {
            var h = Math.round(msg.heading) % 360;
            if (h < 0) h += 360;
            document.getElementById("heading-val").textContent = h + "\u00B0";
        }
```

- [ ] **Step 6: Remove `scan_point`, `scan_complete`, `map_data` message handlers**

Find and delete:
```javascript
    } else if (msg.type === "scan_point") {
        addScanPoint(msg);
    } else if (msg.type === "scan_complete") {
        scanCtrl.setScanRunning(false);
    } else if (msg.type === "map_data") {
        renderFullMap(msg, Dog3D);
```

- [ ] **Step 7: Remove `scanCtrl` from reset handler**

Find:
```javascript
    } else if (msg.type === "reset") {
        Dog3D.reset(); scanCtrl.setScanRunning(false);
```
Replace with:
```javascript
    } else if (msg.type === "reset") {
        Dog3D.reset();
```

- [ ] **Step 8: Remove `scanCtrl` setup and cmd_map connect send**

Find:
```javascript
var scanCtrl = setupScan(send);
```
Delete that line.

Find in the `connect` callback:
```javascript
    send({ type: "cmd_map", action: "get" });
```
Delete that line.

- [ ] **Step 9: Verify no remaining attic references**

```bash
grep -n "scanCtrl\|setupScan\|dogMapState\|addScanPoint\|renderFullMap\|drawMap\|updateLifecycleIndicator\|map\.js" web/app.module.js
```

Expected: no matches.

- [ ] **Step 10: Commit**

```bash
git add web/app.module.js
git commit -m "refactor(web): strip scan/map/lifecycle UI from app.module.js"
```

---

### Task 9: Smoke test — verify server starts with mock

- [ ] **Step 1: Start mock server**

```bash
python3 bark_cli.py mock &
sleep 3
```

Expected: output like `Server running at http://0.0.0.0:8456` with no ImportError or traceback.

- [ ] **Step 2: Check WebSocket connects**

```bash
python3 -c "
import asyncio, websockets, json
async def check():
    async with websockets.connect('ws://localhost:8456/ws') as ws:
        msg = await asyncio.wait_for(ws.recv(), timeout=3)
        print(json.loads(msg))
asyncio.run(check())
"
```

Expected: prints a JSON dict (telem_status or similar) without errors.

- [ ] **Step 3: Stop mock**

```bash
python3 bark_cli.py kill
```

---

### Task 10: Delete host attic files

**Files:**
- Delete: `host/behaviors/scan.py`, `map_store.py`, `wall_fit.py`, `wall_mesh.py`, `octree.py`
- Delete: `host/dead_reckoning.py` — **DEFERRED to Phase 3** (used by `firmware_transport.py`; remove when restructuring into `host/dog/`)
- Delete: `host/sweep/` (entire directory)
- Delete: `host/capture_pose.py`, `calibrate_servos.py`, `servo_test.py`, `identify_servos.py`, `map_servos.py`, `apply_to_firmware.py`, `probe_stock_firmware.py`
- Delete: `host/test_mapping.py`, `test_scan.py`, `test_wall_mesh.py`, `test_button_engage.py`

- [ ] **Step 1: Delete mapping behaviors**

```bash
git rm host/behaviors/scan.py host/behaviors/map_store.py host/behaviors/wall_fit.py host/behaviors/wall_mesh.py host/behaviors/octree.py
```

- [ ] **Step 2: Delete sweep directory**

```bash
git rm -r host/sweep/
```

- [ ] **Step 3: Delete calibration and probe tools**

```bash
git rm host/capture_pose.py host/calibrate_servos.py host/servo_test.py host/identify_servos.py
git rm host/apply_to_firmware.py
# map_servos.py and probe_stock_firmware.py may be untracked — handle both cases:
git rm --ignore-unmatch host/map_servos.py host/probe_stock_firmware.py
rm -f host/map_servos.py host/probe_stock_firmware.py
```

- [ ] **Step 4: Delete ad-hoc test scripts**

```bash
git rm host/test_mapping.py host/test_scan.py host/test_wall_mesh.py host/test_button_engage.py
```

- [ ] **Step 5: Verify host directory**

```bash
ls host/
ls host/behaviors/
```

Expected: `host/` contains `server.py`, `firmware_transport.py`, `comms.py`, `behaviors/`, `dead_reckoning.py` (deferred), `ota_flash.py` (deferred), and support modules. `host/behaviors/` contains only `balance.py`, `button_engage.py`, `__init__.py`.

- [ ] **Step 6: Commit**

```bash
git commit -m "chore: delete attic — mapping behaviors, sweep tooling, calibration tools, ad-hoc tests"
```

---

### Task 11: Delete web attic files

**Files:**
- Delete: `web/tuning.html`, `web/gait.html`
- Delete: `web/modules/map.js`
- Delete: `web/dog3d/sonar.js`, `web/dog3d/walls.js`

Note: `web/dog3d/overlay.js` is **not deleted** — it stays in kernel (provides kinematics debug overlay; `updateLifecycleIndicator` was already removed in Task 6).

- [ ] **Step 1: Delete secondary HTML pages**

```bash
git rm web/tuning.html web/gait.html
```

- [ ] **Step 2: Delete attic JS modules**

```bash
git rm web/modules/map.js web/dog3d/sonar.js web/dog3d/walls.js
```

- [ ] **Step 3: Verify no remaining imports of deleted files**

```bash
grep -rn "map\.js\|sonar\.js\|walls\.js\|tuning\.html\|gait\.html" web/
```

Expected: no matches (we cleaned these up in Tasks 7-8).

- [ ] **Step 4: Commit**

```bash
git commit -m "chore: delete attic web — tuning.html, gait.html, map.js, sonar.js, walls.js"
```

---

### Task 12: Delete stale docs

**Files:**
- Delete: `docs/stock-firmware-analysis.md`, `docs/implementation-plan.md`, `docs/decisions.md`
- Delete: `docs/hardware-schematic.md` (content preserved in new `docs/hardware.md` — Phase 2)
- Delete: `docs/architecture.md`, `docs/protocol.md` (rewritten in Phase 2)
- Delete: completed plans and specs in `docs/superpowers/`

- [ ] **Step 1: Delete root-level stale docs**

```bash
git rm docs/stock-firmware-analysis.md docs/implementation-plan.md docs/decisions.md
git rm docs/hardware-schematic.md docs/architecture.md docs/protocol.md
```

- [ ] **Step 2: List completed plans/specs to confirm deletions**

```bash
ls docs/superpowers/plans/
ls docs/superpowers/specs/
```

Plans to delete (all pre-reshape plans — they describe completed work):
- `2026-04-13-firmware-foundation.md`
- `2026-04-13-ik-balance-transform-gait.md`
- `2026-04-15-bark-cli-ui-trim.md`
- `2026-04-15-remove-dead-transport-code.md`
- `2026-04-15-startup-lifecycle.md`
- `2026-04-16-custom-firmware-only-refactor.md`

Plans to keep (still active):
- `2026-04-18-ota-owner-auth.md` — pending
- `2026-04-19-phase1-kernel-cut.md` — this plan (in progress)

Specs to delete (superseded):
- `2026-04-13-firmware-foundation-design.md`
- `2026-04-15-auto-connect-detect-update-design.md`
- `2026-04-15-startup-lifecycle-design.md`
- `2026-04-15-web-ui-simplification-design.md`

Specs to keep:
- `2026-04-19-kernel-reshape-design.md` — active spec

- [ ] **Step 3: Delete completed plans**

```bash
git rm docs/superpowers/plans/2026-04-13-firmware-foundation.md
git rm docs/superpowers/plans/2026-04-13-ik-balance-transform-gait.md
git rm docs/superpowers/plans/2026-04-15-bark-cli-ui-trim.md
git rm docs/superpowers/plans/2026-04-15-remove-dead-transport-code.md
git rm docs/superpowers/plans/2026-04-15-startup-lifecycle.md
git rm docs/superpowers/plans/2026-04-16-custom-firmware-only-refactor.md
```

- [ ] **Step 4: Delete superseded specs**

```bash
git rm docs/superpowers/specs/2026-04-13-firmware-foundation-design.md
git rm docs/superpowers/specs/2026-04-15-auto-connect-detect-update-design.md
git rm docs/superpowers/specs/2026-04-15-startup-lifecycle-design.md
git rm docs/superpowers/specs/2026-04-15-web-ui-simplification-design.md
```

- [ ] **Step 5: Commit**

```bash
git commit -m "chore: delete stale docs — old arch, protocol, hardware-schematic, completed plans/specs"
```

---

### Task 13: Final verification

- [ ] **Step 1: Line count — confirm ~50% reduction**

```bash
find firmware/src firmware/include firmware/mock firmware/test host web -type f \( -name '*.py' -o -name '*.cpp' -o -name '*.h' -o -name '*.js' -o -name '*.css' -o -name '*.html' \) | xargs wc -l | tail -1
```

Pre-cut was ~17,126 lines. Expected: < 9,000 lines.

- [ ] **Step 2: Firmware unit tests pass**

```bash
cd firmware/test && make && ./test_ik && ./test_transform && ./test_balance && ./test_offsets && ./test_gait_ik && ./test_servos
```

Expected: all exit 0.

- [ ] **Step 3: Mock build passes**

```bash
cd firmware/test && make bark-mock
```

Expected: exits 0.

- [ ] **Step 4: Server starts clean with mock**

```bash
python3 bark_cli.py mock &
sleep 3
curl -s http://localhost:8456/ | head -5
python3 bark_cli.py kill
```

Expected: HTML from `index.html` returned; no Python traceback in server output.

- [ ] **Step 5: No remaining references to deleted files**

```bash
grep -rn "ScanBehavior\|MapStore\|WallFit\|WallMesh\|Octree\|dead_reckoning\|tuning\.html\|gait\.html\|map\.js\|sonar\.js\|walls\.js\|_do_stock_flash\|STOCK_FIRMWARE_BIN\|scan_point\|map_data\|updateLifecycleIndicator\|setupScan\|dogMapState" host/ web/ bark_cli.py | grep -v ".pyc"
```

Expected: no matches.

- [ ] **Step 6: Confirm pre-reshape branch exists**

```bash
git branch --list pre-reshape
```

Expected: `  pre-reshape` printed.

- [ ] **Step 7: Final commit if any loose ends**

```bash
git status
```

If any modified files remain, add and commit them. Otherwise:

```bash
git log --oneline -15
```

Review the commit log — should show a clean set of focused commits from this phase.
