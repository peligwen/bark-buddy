# Remove Dead Transport Code Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the dead `_switch_transport` method, the permanently-false `_user_override_transport` flag it controlled, and the now-ignored WiFi status fields from the initial WebSocket message.

**Architecture:** Pure deletion. No new code. `_switch_transport` had no callers after `cmd_transport` was removed. `_user_override_transport` was only ever set `True` inside `_switch_transport`, so it was always `False` — simplifying its three guard sites removes dead branches. The `wifi_available`/`wifi_ip`/`wifi_ssid` fields in the initial WS status were only consumed by the now-deleted `showWifiBanner` handler in the frontend.

**Tech Stack:** Python 3.11+, aiohttp, asyncio

---

## File Map

| File | Change |
|------|--------|
| `host/server.py` | Delete `_switch_transport`, remove `_user_override_transport` field + 3 reference sites, remove WiFi fields from initial WS status |

---

## Task 1: Remove `_switch_transport` and `_user_override_transport`

**Files:**
- Modify: `host/server.py`

### What's being removed and why

`_switch_transport` (lines ~298–353) was called only by the `cmd_transport` WebSocket handler, which was deleted in the previous cleanup. It is now completely unreachable code.

`_user_override_transport` was a flag that `_switch_transport` set to `True` when a user manually picked a transport from the UI dropdown. Since that dropdown no longer exists and `_switch_transport` is gone, the flag is permanently `False`. Three sites read or write it:

- **`_on_mdns_found` (line ~1022):** `if not self._user_override_transport and wifi_priority > current_priority:` — since the flag is always `False`, `not False` is always `True`, so the condition collapses to just `if wifi_priority > current_priority:`.
- **`_on_device_added` (line ~1047):** `if self._user_override_transport: logger.info(...); return` — the flag is always `False`, so this entire early-return block is dead. Remove it.
- **`_on_device_removed` (line ~1073):** `self._user_override_transport = False` — setting an always-False field to False. Remove the line.

- [ ] **Step 1: Delete `_switch_transport` method**

Find and delete the entire method from its docstring through the closing `except` block. The method starts at `async def _switch_transport(self, mode: str) -> dict:` and ends with `return {"ok": False, "error": str(e), "transport": "sim"}` followed by a blank line. The method that follows it is `async def _add_live_point`.

The full block to delete (search by its opening line):
```python
    async def _switch_transport(self, mode: str) -> dict:
        """Switch between transport modes: 'usb', 'wifi', 'sim'. Returns status dict."""
        self._user_override_transport = (mode != "sim")

        # Build the new transport object
        try:
            if mode == "usb":
                ...
            ...
            except Exception as e:
            logger.exception("Failed to switch transport to %s", mode)
            # Fall back to sim
            await self._replace_transport(SimTransport(), "sim")
            return {"ok": False, "error": str(e), "transport": "sim"}
```

After deletion, `_add_live_point` should follow directly after `_replace_transport` ends.

- [ ] **Step 2: Remove `_user_override_transport` field from `__init__`**

Find and delete line ~127:
```python
        self._user_override_transport: bool = False  # True when user manually selected transport
```

- [ ] **Step 3: Simplify `_on_mdns_found` — remove the flag guard**

Find in `_on_mdns_found` (line ~1022):
```python
        if not self._user_override_transport and wifi_priority > current_priority:
```
Change to:
```python
        if wifi_priority > current_priority:
```

- [ ] **Step 4: Remove dead early-return block from `_on_device_added`**

Find in `_on_device_added` (line ~1047):
```python
        if self._user_override_transport:
            logger.info("Hot-plug: ignoring %s (user override active)", port)
            return
```
Delete all three lines.

- [ ] **Step 5: Remove flag reset from `_on_device_removed`**

Find in `_on_device_removed` (line ~1073):
```python
        self._user_override_transport = False
```
Delete this line. (The comment on the next line, `# Fall back: mDNS-discovered WiFi...`, should remain.)

- [ ] **Step 6: Verify no remaining references**

```bash
grep -n "_switch_transport\|_user_override_transport" host/server.py
```
Expected: no output.

- [ ] **Step 7: Verify server imports cleanly**

```bash
cd /Users/gwen/workspace/bark-buddy && python -c "import sys; sys.path.insert(0,'host'); import server" && echo "OK"
```
Expected: `OK`

- [ ] **Step 8: Commit**

```bash
git add host/server.py
git commit -m "refactor(server): remove dead _switch_transport method and _user_override_transport flag"
```

---

## Task 2: Remove dead WiFi fields from initial WebSocket status

**Files:**
- Modify: `host/server.py`

### What's being removed and why

When a new WebSocket client connects, `_ws_handler` builds an initial `telem_status` message. Lines ~530–534 conditionally add `wifi_available`, `wifi_ip`, and `wifi_ssid` fields:

```python
        wifi_info = getattr(self, '_detected_wifi', None)
        if wifi_info and wifi_info.get("connected"):
            status["wifi_available"] = True
            status["wifi_ip"] = wifi_info.get("ip", "")
            status["wifi_ssid"] = wifi_info.get("ssid", "")
```

The frontend `app.module.js` previously consumed these in `updateStatus` to call `showWifiBanner()`. That handler was removed in the UI trim. The server still detects WiFi internally (used for mDNS auto-reconnect), but there's no longer a client-side consumer for these fields in the status message. Remove them.

Note: `_detected_wifi` itself is NOT removed — the server still uses it internally for `_on_mdns_found`, `_on_mdns_lost`, `_on_device_removed`, and `_broadcast_status` (line 886: `self._detected_wifi = None` after OTA complete). Only the three `status["wifi_*"]` assignments are dead.

- [ ] **Step 1: Remove the WiFi fields from the initial WS status in `_ws_handler`**

Find the block in `_ws_handler` (lines ~530–534):
```python
        wifi_info = getattr(self, '_detected_wifi', None)
        if wifi_info and wifi_info.get("connected"):
            status["wifi_available"] = True
            status["wifi_ip"] = wifi_info.get("ip", "")
            status["wifi_ssid"] = wifi_info.get("ssid", "")
```
Delete all 5 lines.

- [ ] **Step 2: Verify no wifi_available/wifi_ip/wifi_ssid status emission remains**

```bash
grep -n "wifi_available\|wifi_ip\|wifi_ssid" host/server.py
```
Expected: no output. (The `_detected_wifi` references at lines ~187, ~1016–1019, ~1037, ~1075, ~1086 are all internal server state and should remain — they're not status fields.)

- [ ] **Step 3: Verify server imports cleanly**

```bash
cd /Users/gwen/workspace/bark-buddy && python -c "import sys; sys.path.insert(0,'host'); import server" && echo "OK"
```
Expected: `OK`

- [ ] **Step 4: Run firmware tests to confirm nothing regressed**

```bash
bark test
```
Expected: `All 7 test(s) passed.`

- [ ] **Step 5: Commit**

```bash
git add host/server.py
git commit -m "refactor(server): remove dead wifi_available/ip/ssid fields from WS status"
```
