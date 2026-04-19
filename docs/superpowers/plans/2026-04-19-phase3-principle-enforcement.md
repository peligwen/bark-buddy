# Phase 3 — Principle Enforcement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enforce the design principles: rename `FirmwareTransport` → `Dog`, split into `host/dog/` package, delete `comms.py` + `dead_reckoning.py`, extract OTA and lock from `server.py`, fold offset/servo-pin UX into primary web UI.

**Architecture:** `host/dog/` replaces `firmware_transport.py` + `comms.py` + `dead_reckoning.py`. `DogIO` handles raw serial/TCP I/O; `Dog` provides the public API. `server.py` is trimmed by extracting `OtaManager` to `ota.py` and `ControlLock` to `lock.py`. The web UI gains a minimal offset + servo-pin panel.

**Tech Stack:** Python 3.11+, asyncio, aiohttp, pyserial-asyncio, vanilla JS ES modules.

---

## File map

**Create:**
- `host/dog/__init__.py` — re-exports `Dog`, `DIRECTIONS`, `SERIAL_BAUD`
- `host/dog/io.py` — `DogIO` class: raw serial/TCP open/close/send/receive loop
- `host/dog/dog.py` — `Dog` class: public API (send_json, telemetry cache, callbacks, helpers)
- `host/dog/discover.py` — `find_serial_port()`, `detect_serial_dog()` (probe + instantiate)
- `host/ota.py` — `OtaManager` class: firmware build, binary serve, OTA update HTTP handlers
- `host/lock.py` — `ControlLock` class: WS control-lock state machine

**Modify:**
- `host/server.py` — update imports → `Dog`; remove dead code; delegate OTA → `OtaManager`, lock → `ControlLock`; remove dead reckoning call; remove dead `_read_config_local_value` import
- `bark_cli.py` — update `find_serial_port` import to `from dog.discover import find_serial_port`
- `web/index.html` — add offset + servo-pin panel section
- `web/modules/panels.js` — add `initOffsetPanel()` + `updateServoPins()`
- `web/app.module.js` — import offset panel; handle `telem_servo_pins`; wire send

**Delete:**
- `host/firmware_transport.py`
- `host/comms.py`
- `host/dead_reckoning.py`

---

### Task 1: Create host/dog/io.py — raw I/O layer

**Files:**
- Create: `host/dog/__init__.py`
- Create: `host/dog/io.py`

`DogIO` owns the asyncio streams and background tasks. `Dog` (Task 2) composes it.

- [ ] **Step 1: Create the package init and io.py**

`host/dog/__init__.py` — empty for now (populated in Task 2):
```python
# host/dog/__init__.py — populated in Task 2
```

`host/dog/io.py`:
```python
"""Raw serial/TCP I/O for Dog. Owned by Dog; not public API."""

import asyncio
import logging

logger = logging.getLogger(__name__)

SERIAL_BAUD = 115200


class DogIO:
    """
    Manages asyncio streams and background reader/keepalive tasks.
    Call open_serial() or open_tcp(), then start_tasks(on_line_cb, send_ping_cb).
    """

    def __init__(self):
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._open = False
        self._reader_task: asyncio.Task | None = None
        self._keepalive_task: asyncio.Task | None = None

    # --- Connection ---

    async def open_serial(self, port: str, dtr_reset: bool = False) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=port, baudrate=SERIAL_BAUD
        )
        if dtr_reset:
            serial_obj = getattr(self._writer.transport, "serial", None)
            if serial_obj and hasattr(serial_obj, "dtr"):
                serial_obj.dtr = False
                await asyncio.sleep(0.1)
                serial_obj.dtr = True
                await asyncio.sleep(3.0)
                await self._drain()
            else:
                await asyncio.sleep(1)
        else:
            await asyncio.sleep(1)
        self._open = True

    async def open_tcp(self, host: str, port: int) -> None:
        self._reader, self._writer = await asyncio.open_connection(host, port)
        self._open = True

    async def close(self) -> None:
        self._open = False
        for task in (self._reader_task, self._keepalive_task):
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self._reader_task = None
        self._keepalive_task = None
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None

    def is_open(self) -> bool:
        return self._open

    # --- I/O ---

    async def send(self, data: bytes) -> None:
        if self._writer:
            self._writer.write(data)
            await self._writer.drain()

    async def _drain(self) -> None:
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.2)
        except asyncio.TimeoutError:
            pass

    # --- Background tasks ---

    def start_tasks(self, on_line_cb, send_ping_cb) -> None:
        """Start reader and keepalive tasks. Call after open_*()."""
        self._reader_task = asyncio.create_task(self._reader_loop(on_line_cb))
        self._keepalive_task = asyncio.create_task(self._keepalive_loop(send_ping_cb))

    async def _reader_loop(self, on_line_cb) -> None:
        try:
            while self._open and self._reader:
                line = await self._reader.readline()
                if not line:
                    logger.warning("DogIO: EOF — TCP connection closed")
                    self._open = False
                    break
                on_line_cb(line)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.warning("DogIO: reader error: %s", e)
            self._open = False

    async def _keepalive_loop(self, send_ping_cb) -> None:
        try:
            while self._open:
                await asyncio.sleep(2.0)
                if self._open:
                    await send_ping_cb()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.debug("DogIO: keepalive error: %s", e)
```

- [ ] **Step 2: Verify syntax**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
python -c "import ast; ast.parse(open('host/dog/io.py').read()); print('OK')"
```

Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add host/dog/__init__.py host/dog/io.py
git commit -m "feat(dog): add host/dog package with DogIO raw I/O layer"
```

---

### Task 2: Create host/dog/dog.py — public Dog API

**Files:**
- Create: `host/dog/dog.py`
- Modify: `host/dog/__init__.py`

`Dog` replaces `FirmwareTransport`. No dead reckoning — `get_heading()` returns IMU yaw directly.

- [ ] **Step 1: Write host/dog/dog.py**

```python
"""
Dog — transport for custom MechDog firmware.

Connects over USB serial or WiFi TCP. All commands sent via send_json().
Firmware pushes NDJSON telemetry; cached values updated by background reader.
"""

import asyncio
import json
import logging
from typing import Optional

from .io import DogIO, SERIAL_BAUD

logger = logging.getLogger(__name__)

# Valid direction strings accepted by cmd_move
DIRECTIONS = frozenset({"forward", "backward", "left", "right", "stop"})


class Dog:
    """
    Transport for the MechDog custom firmware.

    Usage:
        dog = Dog(host="127.0.0.1", tcp_port=9001)
        await dog.open()
        await dog.send_json({"type": "cmd_stand"})
        await dog.close()
    """

    def __init__(self, port: str = None, host: str = None, tcp_port: int = 9000,
                 dtr_reset: bool = False):
        self._port = port
        self._host = host
        self._tcp_port = tcp_port
        self._dtr_reset = dtr_reset

        self._io = DogIO()

        # Telemetry cache — updated by _handle_telem()
        self._imu: dict = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm: int = 0
        self._battery_mv: int = 7400
        self._firmware_info: dict = {}
        self._engaged: bool = False
        self._ramping: bool = False
        self._battery_cutoff: bool = False

        # Ack queue — firmware acks are queued here for recv_ack()
        self._ack_queue: asyncio.Queue = asyncio.Queue(maxsize=64)

        # Optional callbacks invoked on ack or any telemetry message
        self._ack_callback = None
        self._telem_callback = None

    # --- Lifecycle ---

    async def open(self) -> None:
        if self._port:
            await self._io.open_serial(self._port, self._dtr_reset)
        elif self._host:
            await self._io.open_tcp(self._host, self._tcp_port)
        else:
            raise ConnectionError("Dog: no port or host specified")

        await self._send_json_raw({"type": "ping"})
        await asyncio.sleep(0.5)

        self._io.start_tasks(self._on_raw_line, self._send_ping)
        logger.info("Dog opened on %s", self._port or f"{self._host}:{self._tcp_port}")

    async def close(self) -> None:
        if self._io.is_open():
            try:
                await self._send_json_raw({"type": "cmd_engage", "enabled": False})
                await self.recv_ack("cmd_engage", timeout=1.0)
            except Exception:
                pass
        await self._io.close()
        logger.info("Dog closed")

    def is_open(self) -> bool:
        return self._io.is_open()

    async def send_json(self, msg: dict) -> None:
        if not self._io.is_open():
            raise ConnectionError("Dog not open")
        await self._send_json_raw(msg)

    # --- Telemetry accessors ---

    def get_imu(self) -> dict:
        return dict(self._imu)

    def get_sonar_mm(self) -> int:
        return self._sonar_mm

    def get_battery_mv(self) -> int:
        return self._battery_mv

    def get_engaged(self) -> bool:
        return self._engaged

    def get_ramping(self) -> bool:
        return self._ramping

    def get_battery_cutoff(self) -> bool:
        return self._battery_cutoff

    def get_fw_version(self) -> str:
        return self._firmware_info.get("fw_version", "")

    def get_heading(self) -> float:
        return self._imu.get("yaw", 0.0)

    @property
    def firmware_info(self) -> dict:
        return self._firmware_info

    # --- Callbacks ---

    def set_ack_callback(self, cb) -> None:
        self._ack_callback = cb

    def set_telem_callback(self, cb) -> None:
        self._telem_callback = cb

    async def recv_ack(self, ref_type: str, timeout: float = 2.0) -> Optional[dict]:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return None
            try:
                msg = await asyncio.wait_for(self._ack_queue.get(), timeout=remaining)
                if msg.get("ref_type") == ref_type:
                    return msg
            except asyncio.TimeoutError:
                return None

    # --- Helper commands (thin wrappers around send_json) ---

    async def cmd_buzzer(self, freq_hz: int, duration_ms: int) -> None:
        await self.send_json({"type": "cmd_buzzer", "freq_hz": freq_hz, "duration_ms": duration_ms})

    async def cmd_gpio_mode(self, pin: int, mode: str) -> None:
        await self.send_json({"type": "cmd_gpio", "op": "mode", "pin": pin, "mode": mode})

    async def cmd_gpio_write(self, pin: int, value: int) -> None:
        await self.send_json({"type": "cmd_gpio", "op": "write", "pin": pin, "value": value})

    async def cmd_gpio_read(self, pin: int) -> None:
        await self.send_json({"type": "cmd_gpio", "op": "read", "pin": pin})

    async def cmd_gpio_analog(self, pin: int) -> None:
        await self.send_json({"type": "cmd_gpio", "op": "analog", "pin": pin})

    async def cmd_gpio_subscribe(self, pin: int, mode: str = "input_floating") -> None:
        await self.send_json({"type": "cmd_gpio", "op": "subscribe", "pin": pin, "mode": mode})

    async def cmd_gpio_unsubscribe(self, pin: int) -> None:
        await self.send_json({"type": "cmd_gpio", "op": "unsubscribe", "pin": pin})

    async def cmd_i2c_scan(self, bus: int = 1) -> None:
        await self.send_json({"type": "cmd_i2c", "op": "scan", "bus": bus})

    async def cmd_i2c_read(self, addr: int, reg: int, length: int = 1, bus: int = 1) -> None:
        await self.send_json({"type": "cmd_i2c", "op": "read", "addr": addr, "reg": reg,
                              "len": length, "bus": bus})

    async def cmd_i2c_write(self, addr: int, reg: int, val: int, bus: int = 1) -> None:
        await self.send_json({"type": "cmd_i2c", "op": "write", "addr": addr, "reg": reg,
                              "val": val, "bus": bus})

    async def cmd_aux_servo(self, index: int, pulse_us: int) -> None:
        await self.send_json({"type": "cmd_servo", "index": index, "pulse_us": pulse_us})

    # --- Internal ---

    async def _send_json_raw(self, msg: dict) -> None:
        await self._io.send((json.dumps(msg) + "\n").encode())

    async def _send_ping(self) -> None:
        await self._send_json_raw({"type": "ping"})

    def _on_raw_line(self, line: bytes) -> None:
        try:
            msg = json.loads(line.decode(errors="replace").strip())
            self._handle_telem(msg)
        except json.JSONDecodeError as e:
            logger.warning("Dog: bad JSON (ignored): %s", e)
        except UnicodeDecodeError:
            pass

    def _handle_telem(self, msg: dict) -> None:
        msg_type = msg.get("type", "")
        if msg_type == "telem_imu":
            self._imu["pitch"] = msg.get("pitch", 0)
            self._imu["roll"] = msg.get("roll", 0)
            self._imu["yaw"] = msg.get("yaw", 0)
        elif msg_type == "telem_sonar":
            self._sonar_mm = msg.get("distance_mm", 0)
        elif msg_type == "telem_battery":
            self._battery_mv = msg.get("voltage_mv", 7400)
        elif msg_type == "telem_status":
            self._engaged = msg.get("engaged", self._engaged)
            self._ramping = msg.get("ramping", self._ramping)
            self._battery_cutoff = msg.get("battery_cutoff", self._battery_cutoff)
            if msg.get("wifi") and msg.get("wifi_ip"):
                self._firmware_info["wifi_ip"] = msg["wifi_ip"]
                self._firmware_info["tcp_port"] = msg.get("tcp_port", 9000)
        elif msg_type == "boot":
            self._firmware_info = msg
            logger.info("Dog boot: %s", msg)
        elif msg_type == "ack":
            try:
                self._ack_queue.put_nowait(msg)
            except asyncio.QueueFull:
                logger.debug("Dog: ack queue full — dropping oldest")
                self._ack_queue.get_nowait()
                self._ack_queue.put_nowait(msg)
            if self._ack_callback:
                try:
                    self._ack_callback(msg)
                except Exception:
                    pass
            if not msg.get("ok"):
                logger.warning("Dog NACK: %s — %s",
                               msg.get("ref_type", "?"), msg.get("error", "?"))
        elif msg_type == "ota_status":
            self._firmware_info["ota_status"] = msg.get("status")
            self._firmware_info["ota_error"] = msg.get("error")
            logger.info("Dog OTA: %s", msg)
        elif msg_type == "pong":
            logger.debug("Dog: pong")
        elif msg_type == "error":
            logger.warning("Dog error: %s", msg.get("msg", ""))
        if self._telem_callback:
            try:
                self._telem_callback(msg)
            except Exception:
                pass
```

- [ ] **Step 2: Update __init__.py**

```python
# host/dog/__init__.py
from .dog import Dog, DIRECTIONS, SERIAL_BAUD

__all__ = ["Dog", "DIRECTIONS", "SERIAL_BAUD"]
```

- [ ] **Step 3: Verify syntax**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
python -c "import ast; ast.parse(open('host/dog/dog.py').read()); print('dog.py OK')"
python -c "import ast; ast.parse(open('host/dog/__init__.py').read()); print('__init__.py OK')"
```

Expected: two `OK` lines.

- [ ] **Step 4: Commit**

```bash
git add host/dog/dog.py host/dog/__init__.py
git commit -m "feat(dog): add Dog class — public API replacing FirmwareTransport"
```

---

### Task 3: Create host/dog/discover.py — device discovery

**Files:**
- Create: `host/dog/discover.py`

Moves `find_serial_port`, `_probe_serial_sync`, and `_detect_serial_transport` from `server.py` into the Dog package. Renames `_detect_serial_transport` → `detect_serial_dog` and returns `(Dog, label)` instead of `(FirmwareTransport, label)`.

- [ ] **Step 1: Write host/dog/discover.py**

```python
"""Auto-detect MechDog devices: USB serial glob and serial probe."""

import asyncio
import glob
import logging

logger = logging.getLogger(__name__)


def find_serial_port() -> str | None:
    """Return the first matching USB serial port, or None."""
    ports = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyUSB*")
    return ports[0] if ports else None


def _probe_serial_sync(port: str) -> bool:
    """Blocking probe for custom firmware — runs in a thread via asyncio.to_thread.

    Returns True if JSON pong detected (custom firmware).
    """
    import serial
    import time
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(3.0)
        resp = ""
        for _ in range(3):
            ser.write(b'{"type":"ping"}\n')
            time.sleep(0.5)
            resp += ser.read(ser.in_waiting).decode(errors="replace")
            if '"pong"' in resp:
                break
        ser.close()
        return '"pong"' in resp
    except Exception as e:
        logger.warning("Serial probe failed on %s: %s", port, e)
        return False


async def detect_serial_dog(port: str):
    """Probe a serial port for custom firmware.

    Returns (Dog, label) on success. Raises ConnectionError if not found.
    """
    from .dog import Dog
    logger.info("Probing %s for custom firmware...", port)
    found = await asyncio.to_thread(_probe_serial_sync, port)
    if not found:
        raise ConnectionError(f"No custom firmware response on {port}")
    dog = Dog(port=port)
    label = f"fw:{port.split('/')[-1]}"
    logger.info("Detected custom firmware on %s", port)
    return dog, label
```

- [ ] **Step 2: Verify syntax**

```bash
python -c "import ast; ast.parse(open('host/dog/discover.py').read()); print('OK')"
```

Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add host/dog/discover.py
git commit -m "feat(dog): add discover.py — find_serial_port and detect_serial_dog"
```

---

### Task 4: Update server.py — Dog rename, remove dead code, drop dead reckoning

**Files:**
- Modify: `host/server.py`
- Modify: `bark_cli.py`

Replace all `FirmwareTransport` → `Dog`; remove `comms.py`, `dead_reckoning.py`, and `_read_config_local_value` imports; remove `transport.record_motion()` call; update telem_odometry broadcast to drop x/y position (heading still works from IMU yaw via `dog.get_heading()`).

- [ ] **Step 1: Update imports at top of server.py**

Replace the current import block (lines 8–24) with:

```python
import argparse
import asyncio
import hashlib
import json
import logging
import os
import re

from aiohttp import web

from behaviors.balance import BalanceLayer
from behaviors.button_engage import ButtonEngageBehavior
from dog import Dog, DIRECTIONS
from dog.discover import find_serial_port, detect_serial_dog

logger = logging.getLogger(__name__)
```

- [ ] **Step 2: Update Server.__init__ type annotation**

Change:
```python
def __init__(self, transport: Transport, web_dir: str, ...
```
to:
```python
def __init__(self, transport: Dog, web_dir: str, ...
```

- [ ] **Step 3: Replace FirmwareTransport(...) with Dog(...) everywhere in server.py**

Find all `FirmwareTransport(` occurrences (in `_switch_transport`, `_on_mdns_found`, `_on_device_removed`) and replace with `Dog(`. Also remove the inline `from firmware_transport import FirmwareTransport` lines inside those methods.

There are four call sites total:
- `_switch_transport` mode `fw-usb`: `transport = Dog(port=port)`
- `_switch_transport` mode `fw-wifi`: `transport = Dog(host=ip, tcp_port=tcp_port)`
- `_on_mdns_found`: `transport = Dog(host=ip, tcp_port=port)`
- `_on_device_removed`: `transport = Dog(host=ip, tcp_port=tcp_port)`

- [ ] **Step 4: Replace _detect_serial_transport with detect_serial_dog**

In `_on_device_added`:
```python
# was:
transport, label = await _detect_serial_transport(port)
# now:
transport, label = await detect_serial_dog(port)
```

In `main()`:
```python
# was:
transport, transport_label = await _detect_serial_transport(serial_port)
# now:
transport, transport_label = await detect_serial_dog(serial_port)
```

Also in `main()`, replace the remaining FirmwareTransport instantiation:
```python
# was:
transport = FirmwareTransport(host=host, tcp_port=tcp_port)
# now:
transport = Dog(host=host, tcp_port=tcp_port)
```
Remove `from firmware_transport import FirmwareTransport` at top of `main()`.

- [ ] **Step 5: Remove dead reckoning call from cmd_move handler**

In `_handle_ws_message` (the `cmd_move` branch), remove this line:
```python
self._transport.record_motion(direction)
```

- [ ] **Step 6: Update telem_odometry broadcast in _telemetry_loop**

Replace the current odometry block (which calls `get_position()` and broadcasts x/y) with:
```python
# Odometry broadcast — heading from IMU yaw; no dead-reckoning position
if self._transport and self._ws_clients:
    await self._broadcast({
        "type": "telem_odometry",
        "motion": self._motion,
        "heading": round(self._transport.get_heading(), 1),
    })
```

- [ ] **Step 7: Remove now-dead module-level functions from server.py**

Delete the functions `_probe_serial_sync` and `_detect_serial_transport` — they moved to `dog/discover.py`.

- [ ] **Step 8: Update bark_cli.py — two find_serial_port import sites**

In `bark_cli.py`, `_do_stock_flash` (line ~86) and `cmd_flash` (line ~159) both do:
```python
from server import find_serial_port
```
Change both to:
```python
from dog.discover import find_serial_port
```

These are inside functions so they're inline imports — update both.

- [ ] **Step 9: Verify syntax**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
python -c "
import ast
ast.parse(open('host/server.py').read()); print('server.py OK')
ast.parse(open('bark_cli.py').read()); print('bark_cli.py OK')
"
```

Expected: two `OK` lines.

- [ ] **Step 10: Commit**

```bash
git add host/server.py bark_cli.py
git commit -m "refactor: rename FirmwareTransport → Dog; drop dead reckoning; update discover imports"
```

---

### Task 5: Delete firmware_transport.py, comms.py, dead_reckoning.py

**Files:**
- Delete: `host/firmware_transport.py`
- Delete: `host/comms.py`
- Delete: `host/dead_reckoning.py`

- [ ] **Step 1: Verify no remaining references**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
grep -rn "firmware_transport\|from comms import\|from dead_reckoning\|DeadReckoningMixin\|record_motion" \
  host/ bark_cli.py web/ 2>/dev/null
```

Expected: no output. If any appear, fix them before proceeding.

- [ ] **Step 2: Delete the three files**

```bash
rm host/firmware_transport.py host/comms.py host/dead_reckoning.py
```

- [ ] **Step 3: Run bark test**

```bash
python bark_cli.py test
```

Expected: `All 6 test(s) passed.`

- [ ] **Step 4: Commit**

```bash
git add -u
git commit -m "chore: delete firmware_transport.py, comms.py, dead_reckoning.py — replaced by host/dog/"
```

---

### Task 6: Create host/lock.py — extract ControlLock from server.py

**Files:**
- Create: `host/lock.py`
- Modify: `host/server.py`

Extract the eight lock-management methods into a `ControlLock` class. Server holds `self._lock = ControlLock()` and delegates.

- [ ] **Step 1: Write host/lock.py**

```python
"""WebSocket control-lock state machine."""

import time
from aiohttp import web


class ControlLock:
    """
    Mutual-exclusion lock for WebSocket control sessions.

    One client holds the lock at a time. Lock auto-releases after
    `timeout` seconds of inactivity. Call touch() on each control command.
    """

    def __init__(self, timeout: float = 30.0):
        self._holder: web.WebSocketResponse | None = None
        self._name: str = ""
        self._time: float = 0.0
        self._timeout = timeout

    def is_timed_out(self) -> bool:
        if self._holder and self._time:
            return time.monotonic() - self._time > self._timeout
        return False

    def is_locked_by(self, ws: web.WebSocketResponse) -> bool:
        if self.is_timed_out():
            return False
        return self._holder is ws

    def is_locked(self) -> bool:
        if self.is_timed_out():
            return False
        return self._holder is not None

    def can_control(self, ws: web.WebSocketResponse) -> bool:
        if self.is_timed_out():
            return True
        return self._holder is None or self._holder is ws

    def acquire(self, ws: web.WebSocketResponse, name: str) -> None:
        self._holder = ws
        self._name = name
        self._time = time.monotonic()

    def release(self) -> None:
        self._holder = None
        self._name = ""
        self._time = 0.0

    def touch(self) -> None:
        """Reset timeout — call on each control command from lock holder."""
        if self._holder:
            self._time = time.monotonic()

    def release_if_holder(self, ws: web.WebSocketResponse) -> bool:
        """Release lock if ws holds it. Returns True if released."""
        if self._holder is ws:
            self.release()
            return True
        return False

    @property
    def holder(self) -> web.WebSocketResponse | None:
        return self._holder if not self.is_timed_out() else None

    @property
    def holder_name(self) -> str:
        return self._name if not self.is_timed_out() else ""

    def status_msg(self) -> dict:
        timed_out = self.is_timed_out()
        return {
            "type": "lock_status",
            "locked": self._holder is not None and not timed_out,
            "holder": self._name if self._holder and not timed_out else None,
            "is_you": False,  # caller overrides per-client
        }
```

- [ ] **Step 2: Update server.py to use ControlLock**

Add import at top of server.py:
```python
from lock import ControlLock
```

In `Server.__init__`, replace the four individual lock state fields:
```python
# Remove:
#   self._lock_holder: web.WebSocketResponse | None = None
#   self._lock_name: str = ""
#   self._lock_time: float = 0.0
#   self._lock_timeout: float = 30.0
# Add:
self._lock = ControlLock(timeout=30.0)
```

- [ ] **Step 3: Update all lock method call sites in server.py**

| Old call | New call |
|---|---|
| `self._lock_timed_out()` | `self._lock.is_timed_out()` |
| `self._is_locked_by(ws)` | `self._lock.is_locked_by(ws)` |
| `self._is_locked()` | `self._lock.is_locked()` |
| `self._can_control(ws)` | `self._lock.can_control(ws)` |
| `self._lock_holder` | `self._lock.holder` |
| `self._lock_name` | `self._lock.holder_name` |
| `await self._acquire_lock(ws, name)` | `self._lock.acquire(ws, name)` |
| `await self._release_lock()` | `self._lock.release()` |
| `self._lock_time = _time.monotonic()` | `self._lock.touch()` |
| `self._lock_status_msg()` | `self._lock.status_msg()` |

- [ ] **Step 4: Update _broadcast_lock_status**

```python
async def _broadcast_lock_status(self) -> None:
    msg = self._lock.status_msg()
    dead = set()
    for ws in self._ws_clients:
        m = dict(msg)
        m["is_you"] = self._lock.is_locked_by(ws)
        try:
            await ws.send_str(json.dumps(m))
        except (ConnectionError, ConnectionResetError):
            dead.add(ws)
    self._ws_clients -= dead
```

- [ ] **Step 5: Remove eight extracted lock methods from server.py**

Delete from Server class:
- `_lock_timed_out`
- `_check_lock_timeout`
- `_is_locked_by`
- `_is_locked`
- `_can_control`
- `_lock_status_msg`
- `_acquire_lock`
- `_release_lock`

- [ ] **Step 6: Verify syntax and run bark test**

```bash
python -c "
import ast
ast.parse(open('host/lock.py').read()); print('lock.py OK')
ast.parse(open('host/server.py').read()); print('server.py OK')
"
python bark_cli.py test
```

Expected: two `OK` lines; `All 6 test(s) passed.`

- [ ] **Step 7: Commit**

```bash
git add host/lock.py host/server.py
git commit -m "refactor(server): extract ControlLock to lock.py — remove 8 lock methods from Server"
```

---

### Task 7: Create host/ota.py — extract OTA handlers from server.py

**Files:**
- Create: `host/ota.py`
- Modify: `host/server.py`

Extract `_read_available_fw_version`, `_firmware_binary_path`, `_compute_sha256`, `_do_firmware_build`, and the four `_handle_firmware_*` methods into an `OtaManager` class.

The `_do_firmware_build` body is a **direct copy** of `server._do_firmware_build` — same subprocess pattern, adjusted to use local `_firmware_dir()` and `_binary_path()` helpers instead of the inline `os.path` calls.

- [ ] **Step 1: Write host/ota.py**

```python
"""OTA firmware update: build, binary serve, cmd_ota_update orchestration."""

import asyncio
import hashlib
import logging
import os
import re

from aiohttp import web

logger = logging.getLogger(__name__)


def _firmware_dir() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "firmware"))


def _binary_path() -> str:
    return os.path.abspath(os.path.join(
        _firmware_dir(), ".pio", "build", "mechdog", "firmware.bin"
    ))


def _compute_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def read_available_fw_version() -> str:
    """Parse FW_VERSION from firmware/include/config.h."""
    config_path = os.path.join(_firmware_dir(), "include", "config.h")
    try:
        with open(config_path) as f:
            m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', f.read())
            return m.group(1) if m else ""
    except FileNotFoundError:
        return ""


class OtaManager:
    """Handles firmware OTA: build, binary serve, and OTA command dispatch."""

    def __init__(self, get_transport_fn, get_transport_label_fn):
        """
        get_transport_fn: callable → current Dog instance (or None)
        get_transport_label_fn: callable → current transport label string
        """
        self._get_transport = get_transport_fn
        self._get_label = get_transport_label_fn
        self._binary_sha256: str | None = None
        self.available_version = read_available_fw_version()

    def add_routes(self, router) -> None:
        router.add_get("/api/firmware/status",  self.handle_status)
        router.add_post("/api/firmware/build",  self.handle_build)
        router.add_get("/api/firmware/binary",  self.handle_binary)
        router.add_post("/api/firmware/update", self.handle_update)

    async def _do_build(self) -> dict:
        """Run pio build. Implementation mirrors server._do_firmware_build."""
        # Copy the body of server._do_firmware_build verbatim here,
        # replacing _firmware_binary_path() with _binary_path()
        # and the firmware_dir computation with _firmware_dir().
        # The subprocess invocation pattern is identical to what was in server.py.
        raise NotImplementedError("Copy body from server._do_firmware_build")

    async def handle_status(self, request: web.Request) -> web.Response:
        transport = self._get_transport()
        label = self._get_label()
        current = transport.get_fw_version() if transport else ""
        is_wifi = "fw:" in label and "/dev/" not in label
        path = _binary_path()
        binary_exists = os.path.exists(path)
        return web.json_response({
            "current_version": current,
            "available_version": self.available_version,
            "update_available": bool(current and self.available_version and
                                     current != self.available_version),
            "transport": label,
            "can_ota": is_wifi,
            "binary_ready": binary_exists,
            "sha256": self._binary_sha256 if binary_exists else None,
        })

    async def handle_build(self, request: web.Request) -> web.Response:
        return web.json_response(await self._do_build())

    async def handle_binary(self, request: web.Request) -> web.Response:
        path = _binary_path()
        if not os.path.exists(path):
            return web.json_response(
                {"error": "No firmware binary. Run /api/firmware/build first."}, status=404
            )
        return web.FileResponse(path, headers={
            "Content-Type": "application/octet-stream",
            "Content-Disposition": "attachment; filename=firmware.bin",
        })

    async def handle_update(self, request: web.Request) -> web.Response:
        build = await self._do_build()
        if not build.get("ok"):
            return web.json_response(
                {"ok": False, "error": "Build failed", "output": build.get("output")},
                status=500,
            )
        host_parts = request.host.split(":")
        host_ip = host_parts[0]
        host_port = host_parts[1] if len(host_parts) > 1 else "8080"
        binary_url = f"http://{host_ip}:{host_port}/api/firmware/binary"
        sha256_hex = _compute_sha256(_binary_path())
        self._binary_sha256 = sha256_hex

        transport = self._get_transport()
        if not transport:
            return web.json_response({"ok": False, "error": "No firmware transport"}, status=400)
        try:
            await transport.send_json({
                "type": "cmd_ota_update",
                "url": binary_url,
                "sha256": sha256_hex,
            })
        except Exception as e:
            return web.json_response(
                {"ok": False, "error": f"Failed to send OTA command: {e}"}, status=500
            )
        return web.json_response({
            "ok": True,
            "binary_url": binary_url,
            "new_version": self.available_version,
            "sha256": sha256_hex,
        })
```

**Note on `_do_build`:** The `raise NotImplementedError` is a placeholder in this plan to avoid embedding subprocess code. When implementing, copy the body of `server._do_firmware_build` verbatim, replacing `_firmware_binary_path()` with `_binary_path()` and the inline `firmware_dir` computation with `_firmware_dir()`. The subprocess call signature is identical. Remove the `raise` line.

- [ ] **Step 2: Update server.py to use OtaManager**

Add import:
```python
from ota import OtaManager
```

In `Server.__init__`, add:
```python
self._ota = OtaManager(
    get_transport_fn=lambda: self._transport,
    get_transport_label_fn=lambda: self._transport_label,
)
```
Remove `self._available_fw_version = ...` and `self._binary_sha256 = None` from `__init__`.

In `Server.start()`, replace the four individual OTA route lines with:
```python
self._ota.add_routes(app.router)
```

In `_broadcast_status` and `_ws_handler` initial status message, replace `self._available_fw_version` with `self._ota.available_version`.

- [ ] **Step 3: Remove extracted code from server.py**

Delete from server.py:
- Module-level functions: `_read_available_fw_version`, `_firmware_binary_path`, `_compute_sha256`
- Server methods: `_do_firmware_build`, `_handle_firmware_status`, `_handle_firmware_build`, `_handle_firmware_binary`, `_handle_firmware_update`

- [ ] **Step 4: Verify syntax and line counts**

```bash
python -c "
import ast
ast.parse(open('host/ota.py').read()); print('ota.py OK')
ast.parse(open('host/server.py').read()); print('server.py OK')
"
wc -l host/server.py host/ota.py host/lock.py host/dog/dog.py host/dog/io.py host/dog/discover.py
python bark_cli.py test
```

Expected: all syntax OK; `All 6 test(s) passed.`; no single file over 500 lines.

- [ ] **Step 5: Commit**

```bash
git add host/ota.py host/server.py
git commit -m "refactor(server): extract OtaManager to ota.py — remove 5 OTA methods from Server"
```

---

### Task 8: Web UI — offset and servo-pin panel

**Files:**
- Modify: `web/index.html`
- Modify: `web/modules/panels.js`
- Modify: `web/app.module.js`

Add a collapsible "Offsets & Pins" panel to the operations column. Sends `cmd_offset` and `cmd_servo_pin`; displays current pins from `telem_servo_pins`. The server already passes both through (server.py passthrough list includes `cmd_offset`, `cmd_servo_pin`).

- [ ] **Step 1: Add offset panel HTML to web/index.html**

Find the right-panel operations section (the div with action buttons, OTA button, etc.). Add this block at the bottom of that section, before its closing `</div>`:

```html
    <details id="offset-panel" style="margin-top:12px">
      <summary style="cursor:pointer;font-size:0.85rem;color:#aaa;user-select:none">Offsets &amp; Pins</summary>
      <div style="margin-top:8px;display:flex;flex-direction:column;gap:6px;font-size:0.8rem">
        <div style="color:#888">Servo offset (index 0–7, ±500 µs)</div>
        <div style="display:flex;gap:6px;align-items:center">
          <select id="offset-idx" style="width:50px">
            <option>0</option><option>1</option><option>2</option><option>3</option>
            <option>4</option><option>5</option><option>6</option><option>7</option>
          </select>
          <input id="offset-val" type="number" min="-500" max="500" value="0"
                 style="width:70px;background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:2px 4px">
          <button id="offset-set-btn" style="padding:2px 8px">Set</button>
        </div>
        <div style="color:#888;margin-top:4px">Servo pin (GPIO)</div>
        <div style="display:flex;gap:6px;align-items:center">
          <select id="pin-idx" style="width:50px">
            <option>0</option><option>1</option><option>2</option><option>3</option>
            <option>4</option><option>5</option><option>6</option><option>7</option>
          </select>
          <input id="pin-val" type="number" min="0" max="39" value="0"
                 style="width:60px;background:#1a1a2e;color:#e0e0e0;border:1px solid #444;padding:2px 4px">
          <button id="pin-set-btn" style="padding:2px 8px">Set Pin</button>
        </div>
        <div id="servo-pins-display" style="color:#666;font-size:0.75rem;margin-top:4px"></div>
      </div>
    </details>
```

- [ ] **Step 2: Add initOffsetPanel and updateServoPins to web/modules/panels.js**

Append to the end of `web/modules/panels.js`:

```javascript
export function initOffsetPanel(sendFn) {
    const offsetBtn = document.getElementById('offset-set-btn');
    const pinBtn = document.getElementById('pin-set-btn');
    if (!offsetBtn || !pinBtn) return;

    offsetBtn.addEventListener('click', () => {
        const index = parseInt(document.getElementById('offset-idx').value, 10);
        const offset_us = parseInt(document.getElementById('offset-val').value, 10);
        if (isNaN(index) || isNaN(offset_us)) return;
        sendFn({ type: 'cmd_offset', index, offset_us });
    });

    pinBtn.addEventListener('click', () => {
        const index = parseInt(document.getElementById('pin-idx').value, 10);
        const pin = parseInt(document.getElementById('pin-val').value, 10);
        if (isNaN(index) || isNaN(pin)) return;
        sendFn({ type: 'cmd_servo_pin', index, pin });
    });
}

export function updateServoPins(msg) {
    const el = document.getElementById('servo-pins-display');
    if (!el) return;
    const pins = msg.pins || [];
    if (!pins.length) return;
    el.textContent = 'Pins: ' + pins.map((p, i) => `${i}→${p}`).join('  ');
}
```

- [ ] **Step 3: Wire up in web/app.module.js**

Add to the import block at the top:
```javascript
import { initOffsetPanel, updateServoPins } from './modules/panels.js';
```

In the initialization section (after `initBatteryGraph` and `initOtaPanel` calls), add:
```javascript
initOffsetPanel((msg) => ws.send(JSON.stringify(msg)));
```

In `handleMessage`, add a case for `telem_servo_pins`:
```javascript
} else if (msg.type === 'telem_servo_pins') {
    updateServoPins(msg);
}
```

- [ ] **Step 4: Verify with bark mock**

```bash
python bark_cli.py mock --no-browser &
sleep 4
# Open http://localhost:8456 in browser
# Confirm: "Offsets & Pins" details element appears in operations panel
# Confirm: clicking Set sends cmd_offset in browser dev tools Network/WS tab
```

- [ ] **Step 5: Commit**

```bash
git add web/index.html web/modules/panels.js web/app.module.js
git commit -m "feat(web): add offset and servo-pin panel to primary UI"
```

---

### Task 9: Verification

**Files:** none — verification only.

- [ ] **Step 1: Run bark test**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
python bark_cli.py test
```

Expected: `All 6 test(s) passed.`

- [ ] **Step 2: Build mock firmware**

```bash
cd firmware/test && make bark-mock
```

Expected: clean build or `make: 'bark-mock' is up to date.`

- [ ] **Step 3: Smoke test bark mock**

```bash
cd /Users/gwen/workspace/bark-buddy/.worktrees/phase3
python bark_cli.py mock --no-browser
```

Expected: server starts with no import errors; mock firmware connects; no tracebacks.

- [ ] **Step 4: Check no references to deleted files remain**

```bash
grep -rn "firmware_transport\|from comms import\|from dead_reckoning\|DeadReckoningMixin\|record_motion" \
  host/ web/ bark_cli.py 2>/dev/null
```

Expected: no output.

- [ ] **Step 5: Check line counts**

```bash
wc -l host/server.py host/ota.py host/lock.py host/dog/dog.py host/dog/io.py host/dog/discover.py
```

Expected: server.py ≤ 550 lines; all dog/* files ≤ 250 lines.

---

## Post-plan decisions

**`host/ota_flash.py`** (264 lines): Not atticized. Revisit alongside OTA owner-auth plan (`docs/superpowers/plans/2026-04-18-ota-owner-auth.md`).

**IK gait pipeline**: Resume after Phase 3 merge. All IK firmware code and unit tests are kernel and unaffected.
