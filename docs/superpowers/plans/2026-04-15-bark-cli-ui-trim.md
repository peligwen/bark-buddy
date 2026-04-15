# bark CLI + Web UI Trim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `bark` CLI entry point that starts the server and opens the browser, with subcommands for sim, flash, test, and WiFi setup — and strip transport/WiFi implementation details from the web UI.

**Architecture:** A single top-level `bark_cli.py` module is registered as a console script in `pyproject.toml`. It adds `host/` to `sys.path` before importing server code, preserving all existing bare imports. Server gets a minimal `open_browser` flag. Web UI has the transport dropdown, WiFi banner, and WiFi setup modal removed; the transport badge becomes a read-only status indicator.

**Tech Stack:** Python 3.11+, setuptools, argparse, subprocess, webbrowser; vanilla JS (ES modules), aiohttp

---

## File Map

| File | Status | Change |
|------|--------|--------|
| `pyproject.toml` | Create | Package definition with `bark` entry point |
| `bark_cli.py` | Create | CLI: subcommands, sys.path setup, browser open |
| `host/server.py` | Modify | Add `open_browser` param; remove `cmd_transport` and `cmd_wifi_setup` handlers |
| `web/index.html` | Modify | Remove `#transport-wrapper` div (keep badge span), `#wifi-banner`, `#wifi-modal` |
| `web/modules/panels.js` | Modify | Remove transport/WiFi functions; export stays clean |
| `web/app.module.js` | Modify | Update imports; remove transport/WiFi message handling |
| `web/style.css` | Modify | Remove `.transport-menu`, `#wifi-banner`, `.modal-content`, `#wifi-modal-status` blocks |

---

## Task 1: Create `pyproject.toml`

**Files:**
- Create: `pyproject.toml`

- [ ] **Step 1: Create the file**

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "bark-buddy"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "aiohttp>=3.9",
    "websockets>=12.0",
    "pyserial-asyncio>=0.6",
    "numpy>=1.24",
    "zeroconf>=0.131",
]

[project.scripts]
bark = "bark_cli:main"

[tool.setuptools]
py-modules = ["bark_cli"]
```

- [ ] **Step 2: Install in editable mode**

```bash
cd /Users/gwen/workspace/bark-buddy
pip install -e .
```

Expected: No errors. `bark` appears in the output of `which bark` (or similar PATH confirmation).

- [ ] **Step 3: Verify entry point is on PATH**

```bash
bark --help
```

Expected: `error: argument -h/--help ...` or `usage: bark ...` — it will error because `bark_cli.py` doesn't exist yet, but the command should be found. Proceed to Task 2.

- [ ] **Step 4: Commit**

```bash
git add pyproject.toml
git commit -m "chore: add pyproject.toml with bark entry point"
```

---

## Task 2: Create `bark_cli.py`

**Files:**
- Create: `bark_cli.py`

- [ ] **Step 1: Create `bark_cli.py` at the repo root**

```python
"""bark — Bark-Buddy CLI entry point."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
HOST_DIR = PROJECT_ROOT / "host"
FIRMWARE_DIR = PROJECT_ROOT / "firmware"


def _ensure_host_importable():
    """Add host/ to sys.path so server.py's bare imports work."""
    host = str(HOST_DIR)
    if host not in sys.path:
        sys.path.insert(0, host)


def _add_server_flags(parser):
    parser.add_argument("--host", default="0.0.0.0",
                        help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8456,
                        help="HTTP port (default: 8456)")
    parser.add_argument("--serial", default=None,
                        help="Serial port override (e.g. /dev/cu.usbserial-10)")
    parser.add_argument("--wifi", default=None,
                        help="WiFi address override (e.g. 192.168.1.163)")
    parser.add_argument("--wifi-password", default=None,
                        help="WebREPL password")
    parser.add_argument("--no-mdns", action="store_true",
                        help="Disable mDNS auto-discovery")
    parser.add_argument("--no-browser", action="store_true",
                        help="Don't auto-open browser")


def cmd_serve(args):
    _ensure_host_importable()
    import asyncio
    # Patch args to match what server.main() expects
    args.restart = False
    args.open_browser = not getattr(args, "no_browser", False)
    from server import main as server_main
    asyncio.run(server_main(args))


def cmd_flash(args):
    result = subprocess.run(
        ["pio", "run", "-t", "upload"],
        cwd=FIRMWARE_DIR,
    )
    sys.exit(result.returncode)


def cmd_test(args):
    test_dir = FIRMWARE_DIR / "test"
    result = subprocess.run(["make", "-C", str(test_dir)])
    if result.returncode != 0:
        sys.exit(result.returncode)
    # Run each test binary
    targets = [
        "test_ik", "test_transform", "test_balance",
        "test_offsets", "test_gait_ik", "test_servos", "test_lifecycle",
    ]
    failed = []
    for name in targets:
        binary = test_dir / name
        if binary.exists():
            r = subprocess.run([str(binary)], cwd=test_dir)
            if r.returncode != 0:
                failed.append(name)
    if failed:
        print(f"\n{len(failed)} test(s) FAILED: {', '.join(failed)}")
        sys.exit(1)
    print(f"\nAll {len(targets)} test(s) passed.")


def cmd_wifi_setup(args):
    _ensure_host_importable()
    from setup_wifi import main as wifi_main
    wifi_main()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    parser = argparse.ArgumentParser(
        prog="bark",
        description="Bark-Buddy robot dog CLI",
    )
    sub = parser.add_subparsers(dest="command")

    # Default (no subcommand) — start server
    _add_server_flags(parser)
    parser.set_defaults(sim=False)

    # bark sim
    p_sim = sub.add_parser("sim", help="Start server in simulation mode")
    _add_server_flags(p_sim)
    p_sim.set_defaults(sim=True)

    # bark flash
    sub.add_parser("flash", help="Build + upload firmware via PlatformIO")

    # bark test
    sub.add_parser("test", help="Build and run firmware native tests")

    # bark wifi-setup
    sub.add_parser("wifi-setup", help="Interactive WiFi + WebREPL setup")

    args = parser.parse_args()

    if args.command == "flash":
        cmd_flash(args)
    elif args.command == "test":
        cmd_test(args)
    elif args.command == "wifi-setup":
        cmd_wifi_setup(args)
    else:
        # "sim" or no subcommand — both start the server
        cmd_serve(args)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Reinstall to pick up the new module**

```bash
pip install -e .
```

- [ ] **Step 3: Verify help output**

```bash
bark --help
```

Expected output:
```
usage: bark [-h] [--host HOST] [--port PORT] [--serial SERIAL] [--wifi WIFI]
            [--wifi-password WIFI_PASSWORD] [--no-mdns] [--no-browser]
            {sim,flash,test,wifi-setup} ...

Bark-Buddy robot dog CLI
...
```

- [ ] **Step 4: Verify sim subcommand help**

```bash
bark sim --help
```

Expected: shows same server flags plus description "Start server in simulation mode"

- [ ] **Step 5: Commit**

```bash
git add bark_cli.py
git commit -m "feat: add bark CLI entry point with sim/flash/test/wifi-setup subcommands"
```

---

## Task 3: Add `open_browser` to `host/server.py`

**Files:**
- Modify: `host/server.py:97-99` (Server.__init__ signature)
- Modify: `host/server.py:128-129` (Server.__init__ body)
- Modify: `host/server.py:144` (Server.start signature has the port, browser opens after site.start)
- Modify: `host/server.py:1219-1221` (main() Server construction)

- [ ] **Step 1: Add `open_browser` param to `Server.__init__`**

Change line 97 from:
```python
    def __init__(self, dog: DogComms, web_dir: str, transport=None, transport_label="sim",
                 wifi_host: str | None = None, wifi_password: str | None = None,
                 no_mdns: bool = False):
```
To:
```python
    def __init__(self, dog: DogComms, web_dir: str, transport=None, transport_label="sim",
                 wifi_host: str | None = None, wifi_password: str | None = None,
                 no_mdns: bool = False, open_browser: bool = False):
```

- [ ] **Step 2: Store `open_browser` in `__init__` body**

After line 129 (`self._mdns_browser: "MdnsBrowser | None" = None`), add:
```python
        self._open_browser = open_browser
```

- [ ] **Step 3: Open browser in `Server.start()` after server begins listening**

In `Server.start()`, change the block at lines 160-162 from:
```python
        await site.start()
        logger.info("Server running at http://%s:%d", host, port)
        await asyncio.Event().wait()
```
To:
```python
        await site.start()
        logger.info("Server running at http://%s:%d", host, port)
        if self._open_browser:
            import webbrowser
            webbrowser.open(f"http://localhost:{port}")
        await asyncio.Event().wait()
```

- [ ] **Step 4: Pass `open_browser` in `main(args)`**

Change line 1219 from:
```python
    server = Server(dog, web_dir, transport=transport, transport_label=transport_label,
                    wifi_host=wifi_host, wifi_password=wifi_password,
                    no_mdns=args.no_mdns)
```
To:
```python
    server = Server(dog, web_dir, transport=transport, transport_label=transport_label,
                    wifi_host=wifi_host, wifi_password=wifi_password,
                    no_mdns=args.no_mdns,
                    open_browser=getattr(args, 'open_browser', False))
```

- [ ] **Step 5: Smoke test — `bark sim --no-browser` starts server without opening browser**

```bash
bark sim --no-browser --port 8457
```

Expected: server starts, logs `Server running at http://0.0.0.0:8457`, no browser opens. Ctrl-C to stop.

- [ ] **Step 6: Smoke test — `bark sim` opens browser**

```bash
bark sim --port 8458
```

Expected: server starts, browser opens to `http://localhost:8458`. Ctrl-C to stop.

- [ ] **Step 7: Verify old entry point still works (no browser)**

```bash
cd host && python server.py --sim --port 8459
```

Expected: server starts, logs URL, no browser opens (backward compatible). Ctrl-C to stop.

- [ ] **Step 8: Commit**

```bash
git add host/server.py
git commit -m "feat(server): add open_browser param to Server for CLI-driven browser launch"
```

---

## Task 4: Remove `cmd_transport` and `cmd_wifi_setup` handlers from `host/server.py`

**Files:**
- Modify: `host/server.py:726-743`

The handlers occupy lines 726-743. They sit between `cmd_sim_noise` (ends ~725) and `cmd_reset` (starts ~745).

- [ ] **Step 1: Remove the two handler blocks**

Delete lines 726-743 (the two `elif` blocks):
```python
        elif msg_type == "cmd_transport":
            mode = msg.get("mode", "sim")
            if msg.get("wifi_host"):
                self._wifi_host = msg["wifi_host"]
            result = await self._switch_transport(mode)
            await ws.send_str(json.dumps({"type": "transport_result", **result}))

        elif msg_type == "cmd_wifi_setup":
            ssid = msg.get("ssid", "")
            password = msg.get("password", "")
            if not ssid:
                await ws.send_str(json.dumps({"type": "wifi_setup_result", "ok": False, "error": "No SSID"}))
            else:
                result = await self._transport.setup_wifi(ssid, password)
                if result.get("ok"):
                    self._wifi_host = result.get("ip")
                    self._detected_wifi = {"connected": True, "ip": result["ip"], "ssid": ssid}
                await ws.send_str(json.dumps({"type": "wifi_setup_result", **result}))
```

The `cmd_reset` block remains immediately after; no code changes needed there.

- [ ] **Step 2: Run existing server test to confirm nothing broke**

```bash
cd host && python test_server.py
```

Expected: all tests pass (the test doesn't send `cmd_transport` or `cmd_wifi_setup`).

- [ ] **Step 3: Commit**

```bash
git add host/server.py
git commit -m "feat(server): remove cmd_transport and cmd_wifi_setup handlers (moved to CLI)"
```

---

## Task 5: Trim `web/index.html`

**Files:**
- Modify: `web/index.html`

Remove three sections and unwrap the transport badge from its dropdown container.

- [ ] **Step 1: Replace `#transport-wrapper` div with bare badge span**

Change lines 18-28 from:
```html
                <span id="fw-badge" class="fw-badge unknown" title="Firmware version" role="button">FW --</span>
                <div id="transport-wrapper">
                    <span id="transport-badge" class="transport-badge sim">SIM</span>
                    <div id="transport-menu" class="transport-menu hidden">
                        <button data-transport="usb">USB (stock)</button>
                        <button data-transport="usb-fw">USB (firmware)</button>
                        <button data-transport="wifi">WiFi (stock)</button>
                        <button data-transport="wifi-fw">WiFi (firmware)</button>
                        <button data-transport="wifi-setup">Setup WiFi...</button>
                        <button data-transport="sim">Sim (classic)</button>
                    </div>
                </div>
```
To:
```html
                <span id="fw-badge" class="fw-badge unknown" title="Firmware version" role="button">FW --</span>
                <span id="transport-badge" class="transport-badge sim">SIM</span>
```

- [ ] **Step 2: Remove `#wifi-banner`**

Delete lines 45-49:
```html
    <div id="wifi-banner" class="hidden">
        <span id="wifi-banner-text">WiFi detected</span>
        <button id="wifi-banner-switch">Switch to WiFi</button>
        <button class="dismiss" id="wifi-banner-dismiss">&#10005;</button>
    </div>
```

- [ ] **Step 3: Remove `#wifi-modal`**

Delete lines 52-65:
```html
    <div id="wifi-modal" class="modal hidden">
        <div class="modal-content">
            <h3>WiFi Setup</h3>
            <label for="wifi-ssid">Network Name</label>
            <input type="text" id="wifi-ssid" placeholder="SSID" autocomplete="off">
            <label for="wifi-pass">Password</label>
            <input type="password" id="wifi-pass" placeholder="Password">
            <div class="modal-buttons">
                <button id="wifi-modal-cancel" class="action-btn">Cancel</button>
                <button id="wifi-modal-connect" class="action-btn primary">Connect</button>
            </div>
            <div id="wifi-modal-status" class="hidden"></div>
        </div>
    </div>
```

- [ ] **Step 4: Commit**

```bash
git add web/index.html
git commit -m "feat(ui): remove transport dropdown, WiFi banner, WiFi setup modal from header"
```

---

## Task 6: Trim `web/modules/panels.js`

**Files:**
- Modify: `web/modules/panels.js`

Remove the entire "Transport Switcher" section (lines 106-207: `setupTransport`, `updateTransportUI`, `showWifiBanner`, `showWifiModal`, `handleWifiSetupResult`). Keep Battery Graph, Sim Noise Panel, and OTA Panel.

- [ ] **Step 1: Remove the Transport Switcher section**

Delete lines 106-207 — the entire block from `// --- Transport Switcher ---` through `handleWifiSetupResult` (inclusive). The file should jump from `syncNoiseSliders` (line 104) directly to `// --- OTA Firmware Update Panel ---` (currently line 209).

The final shape of the file after removal:

```js
// Sim noise panel, battery graph, OTA panel
import { send } from './ws.js';

// --- Battery Graph ---
// ... (lines 4-66 unchanged)

// --- Sim Noise Panel ---
// ... (lines 68-104 unchanged)

// --- OTA Firmware Update Panel ---
// ... (lines 209-291 unchanged)
```

- [ ] **Step 2: Update the module comment at top of file**

Change line 1 from:
```js
// Sim noise panel, battery graph, transport switcher
```
To:
```js
// Battery graph, sim noise panel, OTA panel
```

- [ ] **Step 3: Verify file has no references to removed functions**

```bash
grep -n "setupTransport\|updateTransportUI\|showWifiBanner\|handleWifiSetupResult\|showWifiModal\|wifi-modal\|wifi-banner\|transport-menu" web/modules/panels.js
```

Expected: no output (zero matches).

- [ ] **Step 4: Commit**

```bash
git add web/modules/panels.js
git commit -m "feat(ui): remove transport switcher and WiFi setup functions from panels.js"
```

---

## Task 7: Update `web/app.module.js`

**Files:**
- Modify: `web/app.module.js`

Remove imports of deleted functions, remove transport/WiFi message handling.

- [ ] **Step 1: Update the import from `panels.js`**

Change lines 7-9 from:
```js
import { setupBatteryGraph, recordBattery, setupNoisePanel, syncNoiseSliders,
         setupTransport, updateTransportUI, showWifiBanner, handleWifiSetupResult,
         setupOtaPanel, updateOtaStatus } from './modules/panels.js';
```
To:
```js
import { setupBatteryGraph, recordBattery, setupNoisePanel, syncNoiseSliders,
         setupOtaPanel, updateOtaStatus } from './modules/panels.js';
```

- [ ] **Step 2: Update `updateStatus` — remove WiFi banner call, keep sim panel and badge update**

In `updateStatus`, change lines 73-82 from:
```js
    if (msg.transport != null) {
        var badge = document.getElementById("transport-badge");
        var isSim = msg.transport === "sim";
        badge.textContent = isSim ? msg.transport.toUpperCase() : msg.transport;
        badge.className = "transport-badge " + (isSim ? "sim" : "live");
        var simPanel = document.getElementById("sim-panel");
        if (isSim) simPanel.classList.remove("hidden");
        else simPanel.classList.add("hidden");
    }
    if (msg.wifi_available && msg.wifi_ip) showWifiBanner(msg.wifi_ip, msg.wifi_ssid);
```
To:
```js
    if (msg.transport != null) {
        var badge = document.getElementById("transport-badge");
        var isSim = msg.transport === "sim";
        badge.textContent = isSim ? "SIM" : msg.transport;
        badge.className = "transport-badge " + (isSim ? "sim" : "live");
        var simPanel = document.getElementById("sim-panel");
        if (isSim) simPanel.classList.remove("hidden");
        else simPanel.classList.add("hidden");
    }
```

- [ ] **Step 3: Remove `transport_result` and `wifi_setup_result` message handlers**

In `handleMessage`, delete lines 174-183:
```js
    } else if (msg.type === "transport_result") {
        updateTransportUI(msg);
    } else if (msg.type === "wifi_setup_result") {
        handleWifiSetupResult(msg);
        var badge = document.getElementById("transport-badge");
        if (msg.ok) {
            badge.className = "transport-badge live";
            badge.textContent = "wifi:" + msg.ip;
            showWifiBanner(msg.ip, "");
        }
    }
```

- [ ] **Step 4: Remove `setupTransport()` call from init block**

Change lines 215-218 from:
```js
setupLock();
setupReset();
setupTransport();
setupNoisePanel();
setupBatteryGraph();
setupOtaPanel();
```
To:
```js
setupLock();
setupReset();
setupNoisePanel();
setupBatteryGraph();
setupOtaPanel();
```

- [ ] **Step 5: Verify no dead references remain**

```bash
grep -n "setupTransport\|updateTransportUI\|showWifiBanner\|handleWifiSetupResult\|wifi_setup_result\|transport_result" web/app.module.js
```

Expected: no output.

- [ ] **Step 6: Commit**

```bash
git add web/app.module.js
git commit -m "feat(ui): remove transport/WiFi imports and message handlers from app.module.js"
```

---

## Task 8: Trim `web/style.css`

**Files:**
- Modify: `web/style.css`

Remove transport dropdown menu styles, WiFi banner styles, and WiFi modal content styles. Keep `.transport-badge` (used as read-only status), `.modal` (used by OTA modal), `.modal-buttons` (used by OTA modal).

- [ ] **Step 1: Remove `#transport-wrapper` and transport dropdown caret/hover/switching styles**

Find and delete these lines (around lines 100-172):
```css
#transport-wrapper { position: relative; }
```
```css
.transport-badge::after { content: "\25BE"; font-size: 0.55rem; opacity: 0.6; }
.transport-badge:hover { opacity: 0.85; }
```
```css
.transport-badge.switching { background: var(--accent-dark); color: #fff; }
```
```css
.transport-menu {
    position: absolute;
    top: calc(100% + 6px);
    right: 0;
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    overflow: hidden;
    z-index: 200;
    min-width: 160px;
    box-shadow: 0 8px 24px rgba(0,0,0,0.5);
}
.transport-menu.hidden { display: none; }
.transport-menu button {
    display: block;
    width: 100%;
    padding: 8px 14px;
    border: none;
    background: transparent;
    color: var(--text);
    font-size: 0.75rem;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    text-align: left;
    cursor: pointer;
}
.transport-menu button:hover { background: var(--bg-card-hover); }
.transport-menu button[data-transport="wifi-setup"] { border-top: 1px solid var(--border); color: var(--text-muted); }
```

Also update the `.transport-badge` block: remove `cursor: pointer`, `display: flex`, `align-items: center`, `gap: 4px` (it's now a passive indicator):
```css
.transport-badge {
    font-size: 0.6rem;
    font-weight: 700;
    letter-spacing: 0.5px;
    padding: 3px 8px;
    border-radius: 3px;
    text-transform: uppercase;
    font-family: "SF Mono", "Fira Code", monospace;
}
```

- [ ] **Step 2: Remove `#wifi-banner` styles**

Delete lines 213-234:
```css
#wifi-banner {
    padding: 6px 16px;
    background: var(--green-muted);
    color: var(--green);
    font-size: 0.72rem;
    display: flex;
    align-items: center;
    gap: 10px;
    flex-shrink: 0;
}
#wifi-banner.hidden { display: none; }
#wifi-banner button {
    padding: 2px 10px;
    border: 1px solid var(--green-dark);
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--green);
    font-size: 0.65rem;
    cursor: pointer;
}
#wifi-banner button:hover { background: var(--green-dark); color: #fff; }
#wifi-banner .dismiss { border: none; color: var(--text-dim); margin-left: auto; cursor: pointer; background: none; }
```

- [ ] **Step 3: Remove WiFi modal content styles (keep `.modal`, `.modal-buttons` for OTA)**

Delete only the WiFi-specific lines (around 236-251):
```css
/* WiFi setup modal */
.modal-content { background: var(--bg-panel); border: 1px solid var(--border); border-radius: var(--radius); padding: 20px; min-width: 280px; max-width: 340px; }
.modal-content h3 { margin: 0 0 12px; font-size: 0.85rem; color: var(--text); }
.modal-content label { display: block; font-size: 0.7rem; color: var(--text-muted); margin: 8px 0 4px; }
.modal-content input { width: 100%; padding: 6px 8px; border: 1px solid var(--border); border-radius: var(--radius-sm); background: var(--bg); color: var(--text); font-size: 0.8rem; box-sizing: border-box; }
.modal-content input:focus { outline: none; border-color: var(--blue); }
#wifi-modal-status { margin-top: 10px; font-size: 0.72rem; padding: 6px 8px; border-radius: var(--radius-sm); }
#wifi-modal-status.hidden { display: none; }
#wifi-modal-status.error { background: rgba(255,80,80,0.15); color: var(--red); }
#wifi-modal-status.success { background: var(--green-muted); color: var(--green); }
#wifi-modal-status.pending { background: rgba(255,255,255,0.05); color: var(--text-muted); }
```

Keep `.modal`, `.modal.hidden`, `.modal-buttons`, `.modal-buttons .primary`, `.modal-buttons .primary:hover` — they are used by the OTA modal.

- [ ] **Step 4: Commit**

```bash
git add web/style.css
git commit -m "feat(ui): remove transport dropdown, WiFi banner, and WiFi modal CSS"
```

---

## Task 9: End-to-End Verification

- [ ] **Step 1: Start bark in sim mode and check UI**

```bash
bark sim
```

Expected:
- Terminal shows `Server running at http://0.0.0.0:8456`
- Default browser opens to `http://localhost:8456`
- Header shows `FW --` badge and a `SIM` status badge (no dropdown arrow, not clickable)
- No WiFi banner appears
- Sim Noise panel is visible in the right column
- OTA modal, restart button, firmware badge all still present and functional

- [ ] **Step 2: Verify bark subcommand help**

```bash
bark --help && bark sim --help && bark flash --help && bark test --help && bark wifi-setup --help
```

Expected: each prints usage without errors

- [ ] **Step 3: Run firmware native tests**

```bash
bark test
```

Expected: `make` builds all 7 targets, each binary runs and passes, final line: `All 7 test(s) passed.`

- [ ] **Step 4: Verify existing server tests still pass**

```bash
cd host && python test_server.py
```

Expected: all tests pass

- [ ] **Step 5: Verify `python host/server.py` standalone still works**

```bash
cd /Users/gwen/workspace/bark-buddy && python host/server.py --sim --port 8460 &
sleep 1
curl -s http://localhost:8460/ | grep -c "BARK-BUDDY"
kill %1
```

Expected: `1` (page still serves correctly, no browser auto-opened)
