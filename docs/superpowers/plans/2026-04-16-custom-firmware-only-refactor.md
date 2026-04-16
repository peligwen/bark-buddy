# Custom-Firmware-Only Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate all stock/hybrid firmware paths and the CMD text protocol; host talks JSON directly to a single FirmwareTransport; `bark mock` replaces `bark sim` with real C++ compiled for the host.

**Architecture:** One wire protocol (NDJSON over serial or TCP), one Python transport class, one C++ firmware source that runs on ESP32 or as a native host binary with platform shims. DogComms and CMD encoding are deleted entirely.

**Tech Stack:** Python 3.11+ asyncio, C++17 (clang), ESP32 ArduinoJson, BSD sockets for mock TCP.

---

## Phase A — Delete stock/hybrid paths

### Task 1: Delete stock/hybrid files

**Files:**
- Delete: `host/hybrid_transport.py`
- Delete: `host/repl_transport.py`
- Delete: `host/webrepl_transport.py`
- Delete: `host/hw_transport.py`
- Delete: `host/setup_wifi.py`
- Delete: `host/capture_profile.py`
- Delete: `host/monitor_pins.py`
- Delete: `host/test_hybrid.py`
- Delete: `host/test_sim.py`
- Delete: `host/sim/sim_transport.py`
- Delete: `host/sim/physics.py`
- Delete: `firmware/hybrid/handler.py`
- Rename: `host/capture_stock_pose.py` → `host/capture_pose.py`

- [ ] **Step 1: Delete the files**

```bash
cd /Users/gwen/workspace/bark-buddy
rm host/hybrid_transport.py host/repl_transport.py host/webrepl_transport.py host/hw_transport.py
rm host/setup_wifi.py host/capture_profile.py host/monitor_pins.py host/test_hybrid.py host/test_sim.py
rm host/sim/sim_transport.py host/sim/physics.py
rmdir host/sim/
rm firmware/hybrid/handler.py
rmdir firmware/hybrid/
mv host/capture_stock_pose.py host/capture_pose.py
```

- [ ] **Step 2: Verify deletions**

```bash
ls host/*.py | grep -E "hybrid|repl|webrepl|hw_transport|setup_wifi|capture_profile|monitor_pins|test_hybrid|test_sim"
ls host/sim/ 2>/dev/null || echo "sim/ gone"
ls firmware/hybrid/ 2>/dev/null || echo "hybrid/ gone"
ls host/capture_pose.py
```
Expected: no output for the first command, "sim/ gone", "hybrid/ gone", and the capture_pose.py path.

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "refactor: delete all stock/hybrid firmware paths and sim transport"
```

---

### Task 2: Remove WEBREPL_PASS from config files

**Files:**
- Modify: `firmware/include/config.h`
- Modify: `firmware/include/config_local.h.example`

- [ ] **Step 1: Remove WEBREPL_PASS from config.h**

Open `firmware/include/config.h` and delete the two lines:
```cpp
// WebREPL password (stock firmware only)
#define WEBREPL_PASS "..."
```

- [ ] **Step 2: Remove from example**

Open `firmware/include/config_local.h.example` and delete any `WEBREPL_PASS` line.

- [ ] **Step 3: Verify**

```bash
grep -r "WEBREPL" firmware/include/ && echo "FOUND — remove it" || echo "clean"
```
Expected: `clean`

- [ ] **Step 4: Commit**

```bash
git add firmware/include/config.h firmware/include/config_local.h.example
git commit -m "refactor: remove WEBREPL_PASS from config"
```

---

## Phase B — Collapse CMD protocol, unify transports

### Task 3: Rewrite comms.py — slim Transport ABC

**Files:**
- Modify: `host/comms.py` (trim from ~275 lines to ~55 lines)

- [ ] **Step 1: Rewrite comms.py**

Replace the entire file:

```python
"""
Transport ABC for Bark-Buddy.

Subclasses implement the connection (USB serial, WiFi TCP, mock).
All communication uses JSON directly — no CMD text protocol.
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Optional

SERIAL_BAUD = 115200
READ_TIMEOUT = 0.1

# Valid direction strings accepted by cmd_move
DIRECTIONS = frozenset({"forward", "backward", "left", "right", "stop"})


class Transport(ABC):

    @abstractmethod
    async def open(self) -> None: ...

    @abstractmethod
    async def close(self) -> None: ...

    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    async def send_json(self, msg: dict) -> None: ...

    # Telemetry accessors — return cached values from the streaming reader

    def get_imu(self) -> dict:
        return {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}

    def get_sonar_mm(self) -> int:
        return 0

    def get_battery_mv(self) -> int:
        return 7400

    def get_lifecycle(self) -> str:
        return "unknown"

    def get_fw_version(self) -> str:
        return ""

    @property
    def firmware_info(self) -> dict:
        return {}

    def get_position(self) -> tuple[float, float]:
        return (0.0, 0.0)

    def get_heading(self) -> float:
        return 0.0

    def get_joint_states(self) -> list[float]:
        return [1500.0] * 8

    async def recv_ack(self, ref_type: str, timeout: float = 2.0) -> Optional[dict]:
        return None

    def set_ack_callback(self, cb) -> None:
        pass

    def set_telem_callback(self, cb) -> None:
        pass
```

- [ ] **Step 2: Verify syntax**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -c "import sys; sys.path.insert(0,'host'); import comms; print('ok')"
```
Expected: `ok`

- [ ] **Step 3: Commit**

```bash
git add host/comms.py
git commit -m "refactor: slim comms.py to Transport ABC + DIRECTIONS, delete DogComms"
```

---

### Task 4: Merge json_transport_base into firmware_transport; delete json_transport_base.py

**Files:**
- Modify: `host/firmware_transport.py` (absorb base, remove CMD translator)
- Delete: `host/json_transport_base.py`

- [ ] **Step 1: Write the merged firmware_transport.py**

Replace `host/firmware_transport.py` with:

```python
"""
NDJSON transport for custom MechDog firmware.

Connects over USB serial or WiFi TCP.
Firmware pushes telemetry (IMU 50 Hz, sonar 20 Hz) — no polling needed.
All commands are sent as JSON; responses arrive as ack messages.
"""

import asyncio
import json
import logging
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT
from dead_reckoning import DeadReckoningMixin

logger = logging.getLogger(__name__)


class FirmwareTransport(DeadReckoningMixin, Transport):
    """
    Transport for custom firmware speaking NDJSON over serial or TCP.

    Sensor reads return cached values updated by the background reader task.
    All commands are sent via send_json(); no CMD text protocol.
    """

    def __init__(self, port: str = None, host: str = None, tcp_port: int = 9000,
                 dtr_reset: bool = False):
        self._port = port
        self._host = host
        self._tcp_port = tcp_port
        self._dtr_reset = dtr_reset

        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._open = False
        self._reader_task: Optional[asyncio.Task] = None
        self._keepalive_task: Optional[asyncio.Task] = None

        # Telemetry cache
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm = 0
        self._battery_mv = 7400
        self._firmware_info = {}
        self._lifecycle = "unknown"

        # Ack queue for tools that need to wait on specific ack messages
        self._ack_queue: asyncio.Queue = asyncio.Queue()

        # Optional callbacks invoked on ack or any telemetry message
        self._ack_callback = None
        self._telem_callback = None

        self._init_dead_reckoning()

    # --- Transport ABC ---

    async def open(self) -> None:
        if self._port:
            import serial_asyncio
            self._reader, self._writer = await serial_asyncio.open_serial_connection(
                url=self._port, baudrate=115200
            )
            if self._dtr_reset:
                serial_obj = getattr(self._writer.transport, 'serial', None)
                if serial_obj and hasattr(serial_obj, 'dtr'):
                    serial_obj.dtr = False
                    await asyncio.sleep(0.1)
                    serial_obj.dtr = True
                    await asyncio.sleep(3.0)
                    await self._drain()
                else:
                    await asyncio.sleep(1)
            else:
                await asyncio.sleep(1)
        elif self._host:
            self._reader, self._writer = await asyncio.open_connection(
                self._host, self._tcp_port
            )
        else:
            raise ConnectionError("No port or host specified")

        self._open = True

        await self._send_json({"type": "ping"})
        await asyncio.sleep(0.5)

        self._reader_task = asyncio.create_task(self._reader_loop())
        self._keepalive_task = asyncio.create_task(self._keepalive_loop())

        logger.info("FirmwareTransport opened on %s",
                    self._port or f"{self._host}:{self._tcp_port}")

    async def close(self) -> None:
        if self._open and self._writer:
            try:
                await self._send_json({"type": "cmd_shutdown"})
                await self.recv_ack("cmd_shutdown", timeout=5.0)
            except Exception:
                pass
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
        self._open = False
        logger.info("FirmwareTransport closed")

    def is_open(self) -> bool:
        return self._open

    async def send_json(self, msg: dict) -> None:
        if not self._open:
            raise ConnectionError("FirmwareTransport not open")
        await self._send_json(msg)

    # --- Telemetry accessors ---

    def get_imu(self) -> dict:
        return dict(self._imu)

    def get_sonar_mm(self) -> int:
        return self._sonar_mm

    def get_battery_mv(self) -> int:
        return self._battery_mv

    def get_lifecycle(self) -> str:
        return self._lifecycle

    def get_fw_version(self) -> str:
        return self._firmware_info.get("fw_version", "")

    @property
    def firmware_info(self) -> dict:
        return self._firmware_info

    def get_heading(self) -> float:
        if self._imu.get("yaw", 0) != 0:
            return self._imu["yaw"]
        self._step_dead_reckoning()
        return self._heading

    def get_joint_states(self) -> list:
        return list(self._joint_states) if hasattr(self, '_joint_states') else [1500.0] * 8

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

    # --- Background tasks ---

    async def _keepalive_loop(self):
        try:
            while self._open and self._writer:
                await asyncio.sleep(2.0)
                if self._open and self._writer:
                    await self._send_json({"type": "ping"})
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.debug("FirmwareTransport keepalive error: %s", e)

    async def _reader_loop(self):
        try:
            while self._open and self._reader:
                line = await self._reader.readline()
                if not line:
                    break
                try:
                    msg = json.loads(line.decode(errors="replace").strip())
                    self._handle_telem(msg)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.warning("FirmwareTransport reader error: %s", e)

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
            self._lifecycle = msg.get("lifecycle", self._lifecycle)
            if msg.get("wifi") and msg.get("wifi_ip"):
                self._firmware_info["wifi_ip"] = msg["wifi_ip"]
                self._firmware_info["tcp_port"] = msg.get("tcp_port", 9000)
        elif msg_type == "boot":
            self._firmware_info = msg
            logger.info("FirmwareTransport boot: %s", msg)
        elif msg_type == "ack":
            self._ack_queue.put_nowait(msg)
            if self._ack_callback:
                try:
                    self._ack_callback(msg)
                except Exception:
                    pass
            if not msg.get("ok"):
                logger.warning("FirmwareTransport NACK: %s — %s",
                               msg.get("ref_type", "?"), msg.get("error", "?"))
        elif msg_type == "ota_status":
            self._firmware_info["ota_status"] = msg.get("status")
            self._firmware_info["ota_error"] = msg.get("error")
            logger.info("FirmwareTransport OTA: %s", msg)
        elif msg_type == "pong":
            logger.debug("FirmwareTransport pong")
        elif msg_type == "error":
            logger.warning("FirmwareTransport error: %s", msg.get("msg", ""))
        if self._telem_callback:
            try:
                self._telem_callback(msg)
            except Exception:
                pass

    async def _send_json(self, msg: dict) -> None:
        if not self._writer:
            return
        self._writer.write((json.dumps(msg) + "\n").encode())
        await self._writer.drain()

    async def _drain(self) -> None:
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.2)
        except asyncio.TimeoutError:
            pass
```

- [ ] **Step 2: Delete json_transport_base.py**

```bash
rm /Users/gwen/workspace/bark-buddy/host/json_transport_base.py
```

- [ ] **Step 3: Verify imports resolve**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -c "import sys; sys.path.insert(0,'host'); from firmware_transport import FirmwareTransport; print('ok')"
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add host/firmware_transport.py
git rm host/json_transport_base.py
git commit -m "refactor: merge json_transport_base into firmware_transport, delete CMD translator"
```

---

### Task 5: Add record_motion() to DeadReckoningMixin

**Files:**
- Modify: `host/dead_reckoning.py`

- [ ] **Step 1: Read current dead_reckoning.py**

Check what `_motion_cmd` and `_last_motion_time` are doing and where `_step_dead_reckoning` uses them.

- [ ] **Step 2: Add record_motion() method**

Find the method that sets `_motion_cmd` and `_last_motion_time` and add a public wrapper. Add this method to `DeadReckoningMixin`:

```python
def record_motion(self, direction: str) -> None:
    """Record a motion command for dead-reckoning odometry. Call before send_json cmd_move."""
    self._step_dead_reckoning()
    dir_to_motion = {
        "forward": 3, "backward": 4, "left": 5, "right": 6, "stop": 1
    }
    self._motion_cmd = dir_to_motion.get(direction, 1)
    self._last_motion_time = time.monotonic()
```

- [ ] **Step 3: Verify**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -c "import sys; sys.path.insert(0,'host'); from dead_reckoning import DeadReckoningMixin; print(hasattr(DeadReckoningMixin,'record_motion'))"
```
Expected: `True`

- [ ] **Step 4: Commit**

```bash
git add host/dead_reckoning.py
git commit -m "feat: add record_motion() to DeadReckoningMixin for odometry after CMD collapse"
```

---

### Task 6: Rewrite BalanceLayer — takes Transport directly

**Files:**
- Modify: `host/behaviors/balance.py`

- [ ] **Step 1: Read current balance.py to understand structure**

Note the fall-detection thresholds, `_balance_enabled` flag, and any event emission.

- [ ] **Step 2: Rewrite balance.py**

Replace with (~55 lines):

```python
"""
Balance monitoring layer.

Reads IMU from transport cache; detects falls; emits events.
Balance enable/disable is forwarded to firmware via cmd_balance — firmware owns the servo loop.
"""

import logging
import time

logger = logging.getLogger(__name__)

FALL_PITCH_DEG = 35.0
FALL_ROLL_DEG = 35.0
RECOVER_PITCH_DEG = 20.0
RECOVER_ROLL_DEG = 20.0


class BalanceLayer:

    def __init__(self, transport):
        self._transport = transport
        self._balance_enabled = False
        self._fallen = False
        self._fall_time: float = 0.0

    @property
    def enabled(self) -> bool:
        return self._balance_enabled

    async def set_enabled(self, enabled: bool) -> None:
        self._balance_enabled = enabled
        await self._transport.send_json({"type": "cmd_balance", "enabled": enabled})

    def check(self) -> dict:
        """
        Read cached IMU, detect fall state changes.
        Returns event dict with keys: fallen (bool), recovered (bool), imu (dict).
        Call from server telemetry loop.
        """
        imu = self._transport.get_imu()
        pitch = abs(imu.get("pitch", 0))
        roll = abs(imu.get("roll", 0))
        event = {"fallen": False, "recovered": False, "imu": imu}

        if not self._fallen:
            if pitch > FALL_PITCH_DEG or roll > FALL_ROLL_DEG:
                self._fallen = True
                self._fall_time = time.monotonic()
                event["fallen"] = True
                logger.warning("BalanceLayer: fall detected pitch=%.1f roll=%.1f", pitch, roll)
        else:
            if pitch < RECOVER_PITCH_DEG and roll < RECOVER_ROLL_DEG:
                self._fallen = False
                event["recovered"] = True
                logger.info("BalanceLayer: recovered after %.1fs", time.monotonic() - self._fall_time)

        return event

    @property
    def is_fallen(self) -> bool:
        return self._fallen
```

- [ ] **Step 3: Verify**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -c "import sys; sys.path.insert(0,'host'); from behaviors.balance import BalanceLayer; print('ok')"
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add host/behaviors/balance.py
git commit -m "refactor: rewrite BalanceLayer to take Transport directly, drop DogComms"
```

---

### Task 7: Rewrite ScanBehavior — takes Transport directly

**Files:**
- Modify: `host/behaviors/scan.py`

- [ ] **Step 1: Read current scan.py**

Note the step angle, settle time, scan geometry constants, and the turn/stop call pattern.

- [ ] **Step 2: Replace DogComms calls with send_json**

Find every `self._dog.turn_right()`, `self._dog.stop()`, `self._dog.move(...)` call and replace:

```python
# turn right one step:
await self._transport.send_json({"type": "cmd_move", "direction": "right"})
# stop:
await self._transport.send_json({"type": "cmd_stand"})
```

Change constructor signature:
```python
def __init__(self, transport):
    self._transport = transport
    # ... rest of init unchanged
```

Remove any `from comms import DogComms` import; add `from comms import Transport` if type hints are used.

- [ ] **Step 3: Verify**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -c "import sys; sys.path.insert(0,'host'); from behaviors.scan import ScanBehavior; print('ok')"
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add host/behaviors/scan.py
git commit -m "refactor: rewrite ScanBehavior to take Transport directly, use send_json"
```

---

### Task 8: server.py — remove DogComms, wire Transport directly (Part 1: structure)

**Files:**
- Modify: `host/server.py` — imports, `__init__`, `_replace_transport`, `_switch_transport`

- [ ] **Step 1: Remove DogComms import and references**

Remove:
```python
from comms import DogComms
```

Anywhere `self._dog = DogComms(transport)` is called, replace with `self._transport = transport`.

- [ ] **Step 2: Update Server.__init__**

Change constructor to accept `transport: Transport` (or `None`):
- Remove `dog: DogComms` parameter
- Assign `self._transport: Optional[Transport] = transport`
- Change `self._balance = BalanceLayer(self._dog)` → `self._balance = BalanceLayer(self._transport)`
- Change `self._scan = ScanBehavior(self._dog)` → `self._scan = ScanBehavior(self._transport)`

- [ ] **Step 3: Update _replace_transport**

Old pattern:
```python
new_dog = DogComms(new_transport)
self._dog = new_dog
self._balance = BalanceLayer(new_dog)
self._scan = ScanBehavior(new_dog)
```

New pattern:
```python
self._transport = new_transport
self._balance = BalanceLayer(new_transport)
self._scan = ScanBehavior(new_transport)
```

- [ ] **Step 4: Update _switch_transport — drop sim fallback**

Remove the `elif mode == "sim":` branch and any `SimTransport` instantiation.
Remove the exception-handler fallback that creates `SimTransport()`.
On unknown mode or failed connection → raise `ConnectionError` with a descriptive message.

Remove `from sim.sim_transport import SimTransport` import.

- [ ] **Step 5: Remove _TRANSPORT_PRIORITY "sim" entry**

Change dict to:
```python
_TRANSPORT_PRIORITY = {
    "mock": 0,
    "fw-wifi": 1,
    "fw-usb": 2,
}
```

- [ ] **Step 6: Verify syntax**

```bash
python3 -m py_compile host/server.py && echo ok
```
Expected: `ok`

---

### Task 9: server.py — cmd handlers, telem_sonar, no-hardware exit (Part 2)

**Files:**
- Modify: `host/server.py` — `_handle_ws_message`, `_telemetry_loop`, `main`

- [ ] **Step 1: Update _handle_ws_message**

Replace all `self._dog.move(direction)` with:
```python
if direction in DIRECTIONS:
    self._transport.record_motion(direction)
    await self._transport.send_json({"type": "cmd_move", "direction": direction})
```

Replace `self._dog.stand()` with:
```python
await self._transport.send_json({"type": "cmd_stand"})
```

Replace `self._dog.enable_balance()` / `disable_balance()` with:
```python
await self._balance.set_enabled(enabled)
```

Delete `cmd_sim_noise` handler entirely.

All passthrough commands (`cmd_servo`, `cmd_transform`, `cmd_gait_params`, `cmd_action`, `cmd_pose`, `cmd_balance`, `cmd_move`, `cmd_stand`) should call `await self._transport.send_json(msg)` directly.

- [ ] **Step 2: Update _telemetry_loop**

Replace `self._balance.update()` (which polled `dog.read_imu()` via CMD|5) with:
```python
balance_event = self._balance.check()
if balance_event["fallen"]:
    await self._broadcast({"type": "event_fall", "imu": balance_event["imu"]})
elif balance_event["recovered"]:
    await self._broadcast({"type": "event_recovered"})
```

Remove the ultrasonic poll block (was `cmd = "CMD|4|...|$"`, `await self._dog.read_ultrasonic()`).

Register a telem callback instead — firmware pushes `telem_sonar`; forward it directly:
```python
def _on_telem(msg: dict) -> None:
    asyncio.create_task(self._broadcast(msg))
self._transport.set_telem_callback(_on_telem)
```

Remove any server-side renaming of `telem_sonar` → `telem_ultrasonic`.

- [ ] **Step 3: Update main() — add --fw-tcp, remove --wifi/--wifi-password/--sim**

Remove `args.wifi`, `args.wifi_password`, `args.sim` parsing.

Add:
```python
parser.add_argument("--fw-tcp", default=None, metavar="HOST[:PORT]",
                    help="Connect to firmware over TCP (e.g. 127.0.0.1:9000 for bark mock)")
```

No hardware + no `--fw-tcp` → exit:
```python
print("No MechDog detected. Plug in a device or run 'bark mock'.")
sys.exit(1)
```

`--fw-tcp` → parse host/port, create `FirmwareTransport(host=..., tcp_port=...)`.
If host is `127.0.0.1` or `localhost` → label `"mock"`, else `f"fw-wifi:{host}"`.

- [ ] **Step 4: Update _on_mdns_lost and _on_device_removed — no sim fallback**

Remove any `SimTransport()` fallback. Set transport to `None` / go to disconnected state; broadcast a status message to the UI.

- [ ] **Step 5: Update _probe_serial_sync — drop MicroPython Ctrl-C path**

Remove the Ctrl-C/MicroPython fallback. JSON ping only: no pong → raise `ConnectionError("No firmware response to ping")`.

- [ ] **Step 6: Update _detect_serial_transport — single branch**

Remove hybrid branch. `fw` JSON ping succeeds → return `FirmwareTransport(port=..., dtr_reset=...)` labeled `"fw-usb:{port}"`. Failure → raise.

- [ ] **Step 7: Verify syntax**

```bash
python3 -m py_compile host/server.py && echo ok
```
Expected: `ok`

- [ ] **Step 8: Commit**

```bash
git add host/server.py
git commit -m "refactor: server.py — remove DogComms, direct send_json, telem_sonar push, no sim fallback"
```

---

### Task 10: server.py — broadcast_status cleanup

**Files:**
- Modify: `host/server.py` — `_broadcast_status`

- [ ] **Step 1: Remove wifi_* fields from WebREPL**

In `_broadcast_status` (and any companion helper), remove fields that came from WebREPL probing:
- `wifi_ssid`, `wifi_password`, any `wifi_status` field sourced from stock firmware

Keep fields from `transport.firmware_info` — those carry `wifi_ip` and `tcp_port` from the firmware's own `telem_status` message.

- [ ] **Step 2: Remove connected property bridging DogComms**

Replace any `self._dog.connected` or `dog.transport.is_open()` with `self._transport.is_open()`.

- [ ] **Step 3: Verify syntax**

```bash
python3 -m py_compile host/server.py && echo ok
```

- [ ] **Step 4: Commit**

```bash
git add host/server.py
git commit -m "refactor: remove WebREPL wifi fields from broadcast_status"
```

---

### Task 11: bark_cli.py — drop wifi-setup and sim; add mock stub

**Files:**
- Modify: `bark_cli.py`

- [ ] **Step 1: Remove wifi-setup and sim subcommands**

Delete `cmd_wifi_setup` function.
Delete `bark wifi-setup` subparser.
Delete `bark sim` subparser (sets `args.sim=True`).
Delete `--wifi` and `--wifi-password` flags from `_add_server_flags`.

- [ ] **Step 2: Add cmd_mock function**

```python
def cmd_mock(args):
    import socket
    import time
    mock_bin = FIRMWARE_DIR / "test" / "bark-mock"
    if not mock_bin.exists():
        result = subprocess.run(
            ["make", "-C", str(FIRMWARE_DIR / "test"), "bark-mock"],
            check=True
        )
    proc = subprocess.Popen([str(mock_bin), "--tcp-port", "9001"])
    try:
        # Wait for port to open (up to 5 s)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection(("127.0.0.1", 9001), timeout=0.2)
                s.close()
                break
            except OSError:
                time.sleep(0.1)
        else:
            proc.terminate()
            raise RuntimeError("bark-mock did not open port 9001 within 5 s")
        args.fw_tcp = "127.0.0.1:9001"
        cmd_serve(args)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
```

- [ ] **Step 3: Add mock subparser**

```python
p_mock = sub.add_parser("mock", help="Start server with mock firmware (compiled C++)")
_add_server_flags(p_mock)
```

Add to dispatch:
```python
elif args.command == "mock":
    cmd_mock(args)
```

- [ ] **Step 4: Verify syntax**

```bash
python3 -m py_compile bark_cli.py && echo ok
```
Expected: `ok`

- [ ] **Step 5: Commit**

```bash
git add bark_cli.py
git commit -m "refactor: replace bark sim/wifi-setup with bark mock; drop --wifi/--wifi-password"
```

---

### Task 12: Web UI — telem_sonar, mock badge, remove sim-panel

**Files:**
- Modify: `web/app.module.js`
- Modify: `web/modules/panels.js`
- Modify: `web/index.html`
- Modify: `web/style.css`

- [ ] **Step 1: app.module.js — rename telem_ultrasonic to telem_sonar**

Find the handler for `telem_ultrasonic` (line ~147) and change the case key to `telem_sonar`. Update any internal variable names for clarity.

- [ ] **Step 2: app.module.js — update transport badge**

Find the badge logic (lines ~72-80):
```js
badge.className = "transport-badge " + (isSim ? "sim" : "live");
badge.textContent = isSim ? "SIM" : label;
```

Replace with:
```js
const isMock = label === "mock";
badge.className = "transport-badge " + (isMock ? "mock" : "live");
badge.textContent = isMock ? "MOCK" : label;
```

Remove `syncNoiseSliders` call and any `#sim-panel` show/hide logic.

- [ ] **Step 3: panels.js — delete setupNoisePanel**

Remove `setupNoisePanel` function and `syncNoiseSliders` function entirely.

- [ ] **Step 4: index.html — update badge default and remove sim-panel**

Line ~19: change `class="transport-badge sim"` and text `SIM` to:
```html
<span id="transport-badge" class="transport-badge"></span>
```

Find `#sim-panel` div and delete it entirely.

- [ ] **Step 5: style.css — replace .sim badge with .mock/.live**

Delete `.transport-badge.sim` rule (line ~109).
Delete `#sim-panel.hidden` rule (line ~449).

Add:
```css
.transport-badge.live { background: #2d6a2d; color: #90ee90; }
.transport-badge.mock { background: #6a5a1a; color: #ffd700; }
```

- [ ] **Step 6: Verify JS syntax**

```bash
node --check web/app.module.js && node --check web/modules/panels.js && echo ok
```
Expected: `ok`

- [ ] **Step 7: Commit**

```bash
git add web/app.module.js web/modules/panels.js web/index.html web/style.css
git commit -m "refactor: web UI — telem_sonar, mock badge, remove sim-panel"
```

---

### Task 13: Fix test_server.py and test_scan.py for new Transport API

**Files:**
- Modify: `host/test_server.py`
- Modify: `host/test_scan.py`
- Modify: `host/test_mapping.py` (if it uses DogComms)
- Modify: `host/test_wall_mesh.py` (if it uses DogComms)

- [ ] **Step 1: Add an inline mock transport for test_server.py**

`test_server.py` previously launched `server.py` without `--fw-tcp`; after the refactor that exits with code 1. Fix by launching with a minimal TCP echo that satisfies the firmware handshake.

Add a helper at the top of `test_server.py`:

```python
import asyncio
import json
import threading
import socket

def _start_mock_firmware_tcp(port: int) -> None:
    """Minimal TCP server that answers ping with pong, cmd_shutdown with ack."""
    def _serve():
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(1)
        srv.settimeout(10)
        try:
            conn, _ = srv.accept()
            conn.settimeout(0.5)
            # send boot message
            conn.sendall((json.dumps({"type":"boot","fw_version":"mock-test","imu":True,"sonar":True,"servos":True,"pins_verified":True}) + "\n").encode())
            while True:
                try:
                    data = b""
                    while not data.endswith(b"\n"):
                        chunk = conn.recv(256)
                        if not chunk:
                            return
                        data += chunk
                    msg = json.loads(data.strip())
                    if msg.get("type") == "ping":
                        conn.sendall((json.dumps({"type":"pong"}) + "\n").encode())
                    elif msg.get("type") == "cmd_shutdown":
                        conn.sendall((json.dumps({"type":"ack","ref_type":"cmd_shutdown","ok":True}) + "\n").encode())
                        return
                except socket.timeout:
                    pass
        except Exception:
            pass
        finally:
            srv.close()
    t = threading.Thread(target=_serve, daemon=True)
    t.start()
    return t
```

In the test setup, call:
```python
_start_mock_firmware_tcp(19999)
# pass --fw-tcp 127.0.0.1:19999 to server subprocess
```

- [ ] **Step 2: Update test_scan.py**

Replace `SimTransport()` + `DogComms(transport)` with a simple mock:

```python
class MockTransport:
    def __init__(self):
        self._open = True
        self.sent = []
    def is_open(self): return self._open
    async def send_json(self, msg): self.sent.append(msg)
    def get_imu(self): return {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    def get_sonar_mm(self): return 300
    def record_motion(self, direction): pass

transport = MockTransport()
scan = ScanBehavior(transport)
```

- [ ] **Step 3: Update test_mapping.py and test_wall_mesh.py**

Grep for `DogComms`:
```bash
grep -l "DogComms\|SimTransport" host/test_*.py
```

For each file that uses `DogComms`, replace with the same `MockTransport` pattern above.

- [ ] **Step 4: Run tests**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 -m pytest host/test_scan.py host/test_mapping.py host/test_wall_mesh.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add host/test_server.py host/test_scan.py host/test_mapping.py host/test_wall_mesh.py
git commit -m "test: fix host tests after DogComms removal — inline MockTransport"
```

---

### Task 14: Phase B smoke test

- [ ] **Step 1: Syntax-check all Python**

```bash
python3 -m compileall host/ bark_cli.py -q
```
Expected: no errors

- [ ] **Step 2: Run host test suite**

```bash
python3 -m pytest host/ -v --tb=short
```
Expected: all pass (skip tests that require hardware)

- [ ] **Step 3: Grep for DogComms and SimTransport — confirm zero references**

```bash
grep -r "DogComms\|SimTransport\|cmd_sim_noise\|telem_ultrasonic\|hybrid_transport\|webrepl\|WebREPL" host/ bark_cli.py web/ --include="*.py" --include="*.js" --include="*.html" | grep -v ".pyc"
```
Expected: zero matches

- [ ] **Step 4: Commit**

```bash
git commit --allow-empty -m "chore: Phase B complete — single transport, no CMD protocol, no sim"
```

---

## Phase C — Mock Firmware (native C++ binary)

### Task 15: mock_arduino.h extensions — physics hooks

**Files:**
- Modify: `firmware/test/mock_arduino.h`

- [ ] **Step 1: Add physics hook to ledcWrite**

The existing `ledcWrite` stub captures to `_servo_duty[]`. Extend it to forward to a physics hook:

```cpp
// At top of mock_arduino.h, after existing includes:
namespace physics {
    void on_servo_duty(uint8_t pin, uint32_t duty);
}

// In ledcWrite stub — replace the existing stub with:
inline void ledcWrite(uint8_t pin, uint32_t duty) {
    if (pin < 8) {
        _servo_duty[pin] = duty;
        _servo_log[pin] = duty;
    }
#ifdef MOCK_FIRMWARE
    physics::on_servo_duty(pin, duty);
#endif
}
```

- [ ] **Step 2: Add real-time millis/delay under MOCK_REAL_TIME**

```cpp
#ifdef MOCK_REAL_TIME
#include <chrono>
#include <thread>
inline uint32_t millis() {
    static auto _start = std::chrono::steady_clock::now();
    return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - _start).count();
}
inline void delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#endif
```

- [ ] **Step 3: Verify existing unit tests still compile**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test_ik test_servos
```
Expected: build succeeds

- [ ] **Step 4: Commit**

```bash
git add firmware/test/mock_arduino.h
git commit -m "feat(mock): add physics hooks to mock_arduino.h ledcWrite"
```

---

### Task 16: physics.cpp/.h — body-pose physics model

**Files:**
- Create: `firmware/mock/physics.h`
- Create: `firmware/mock/physics.cpp`

- [ ] **Step 1: Create firmware/mock/ directory and physics.h**

```bash
mkdir -p /Users/gwen/workspace/bark-buddy/firmware/mock
```

Create `firmware/mock/physics.h`:

```cpp
#pragma once
#include <cstdint>

namespace physics {

struct IMUSample {
    float pitch_deg, roll_deg, yaw_deg;
    float ax, ay, az;   // body-frame accelerometer (g)
    float gx, gy, gz;   // body-frame gyro (deg/s)
};

void init();
void on_servo_duty(uint8_t pin, uint32_t duty);
void tick(float dt_s);

void imu_sample(IMUSample& out);
uint16_t sonar_mm();
int battery_raw();

}  // namespace physics
```

- [ ] **Step 2: Create firmware/mock/physics.cpp**

```cpp
#include "physics.h"
#include "../include/config.h"
#include "../include/ik.h"
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <array>

// MOCK_SONAR_MM default 500 mm; override at build time with -DMOCK_SONAR_MM=N
#ifndef MOCK_SONAR_MM
#define MOCK_SONAR_MM 500
#endif

static std::mutex s_mtx;

// Servo pulses (us), indexed by LEDC channel (== servo index)
static uint16_t s_pulse_us[8];

// Smoothed body angles
static float s_pitch_deg = 0.0f;
static float s_roll_deg  = 0.0f;
static float s_yaw_deg   = 0.0f;

// Previous values for gyro derivative
static float s_prev_pitch = 0.0f;
static float s_prev_roll  = 0.0f;
static float s_prev_yaw   = 0.0f;

// Gyro estimates
static float s_gx = 0.0f, s_gy = 0.0f, s_gz = 0.0f;

// Low-pass time constant (seconds)
static constexpr float LP_TAU = 0.2f;

// Convert LEDC duty back to pulse microseconds
static uint16_t duty_to_us(uint32_t duty) {
    // Inverse of us_to_duty: duty * period_us / LEDC_MAX_DUTY
    return (uint16_t)((uint32_t)duty * (1000000UL / SERVO_FREQ_HZ) / LEDC_MAX_DUTY);
}

namespace physics {

void init() {
    for (int i = 0; i < 8; i++)
        s_pulse_us[i] = 1500;
}

void on_servo_duty(uint8_t pin, uint32_t duty) {
    // Find servo index from pin
    for (int i = 0; i < 8; i++) {
        if (SERVO_PINS[i] == pin) {
            std::lock_guard<std::mutex> lk(s_mtx);
            s_pulse_us[i] = duty_to_us(duty);
            return;
        }
    }
}

void tick(float dt_s) {
    // Read servo pulses under lock
    uint16_t pulses[8];
    {
        std::lock_guard<std::mutex> lk(s_mtx);
        for (int i = 0; i < 8; i++) pulses[i] = s_pulse_us[i];
    }

    // Forward kinematics: get foot Z positions for all 4 legs
    // Leg order: 0=FL, 1=FR, 2=RL, 3=RR
    // Each leg: hip = pulses[2*leg], knee = pulses[2*leg+1]
    float foot_z[4];
    for (int leg = 0; leg < 4; leg++) {
        FootPos fp = pulses_to_foot((uint8_t)leg, pulses[2*leg], pulses[2*leg+1]);
        foot_z[leg] = fp.z;
    }

    // Body orientation from foot height differences
    // pitch = atan2(front_avg_z - rear_avg_z, wheelbase)  (deg)
    // roll  = atan2(left_avg_z  - right_avg_z, track)     (deg)
    float front_z = (foot_z[0] + foot_z[1]) * 0.5f;  // FL + FR
    float rear_z  = (foot_z[2] + foot_z[3]) * 0.5f;  // RL + RR
    float left_z  = (foot_z[0] + foot_z[2]) * 0.5f;  // FL + RL
    float right_z = (foot_z[1] + foot_z[3]) * 0.5f;  // FR + RR

    constexpr float wheelbase_mm = 120.0f;
    constexpr float track_mm     = 80.0f;
    constexpr float rad2deg      = 57.29577951f;

    float target_pitch = std::atan2f(front_z - rear_z, wheelbase_mm) * rad2deg;
    float target_roll  = std::atan2f(left_z  - right_z, track_mm)    * rad2deg;

    // Low-pass filter
    float alpha = dt_s / (LP_TAU + dt_s);
    s_pitch_deg += alpha * (target_pitch - s_pitch_deg);
    s_roll_deg  += alpha * (target_roll  - s_roll_deg);
    // yaw stays 0 — no rotation model

    // Gyro from numerical derivative (deg/s)
    s_gx = (s_roll_deg  - s_prev_roll)  / dt_s;
    s_gy = (s_pitch_deg - s_prev_pitch) / dt_s;
    s_gz = (s_yaw_deg   - s_prev_yaw)   / dt_s;

    s_prev_pitch = s_pitch_deg;
    s_prev_roll  = s_roll_deg;
    s_prev_yaw   = s_yaw_deg;
}

void imu_sample(IMUSample& out) {
    out.pitch_deg = s_pitch_deg;
    out.roll_deg  = s_roll_deg;
    out.yaw_deg   = s_yaw_deg;

    // Gravity projection into body frame (1 g down = [0, 0, -1] in world)
    float p = s_pitch_deg * 0.01745329f;
    float r = s_roll_deg  * 0.01745329f;
    out.ax = -std::sinf(p);
    out.ay =  std::sinf(r) * std::cosf(p);
    out.az = -std::cosf(p) * std::cosf(r);

    out.gx = s_gx;
    out.gy = s_gy;
    out.gz = s_gz;
}

uint16_t sonar_mm() {
    return MOCK_SONAR_MM;
}

int battery_raw() {
    // Simulate ~7.4V on a 12-bit ADC with voltage divider (same as firmware)
    return 3200;
}

}  // namespace physics
```

- [ ] **Step 3: Verify it compiles standalone**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
clang++ -std=c++17 -DHOST_BUILD=1 -DMOCK_FIRMWARE=1 -DPINS_VERIFIED=1 \
  -I. -I../include -I../mock \
  -c ../mock/physics.cpp -o /tmp/physics.o && echo ok
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add firmware/mock/physics.h firmware/mock/physics.cpp
git commit -m "feat(mock): add physics.cpp — servo duty to body-pose IMU model"
```

---

### Task 17: net_tcp.cpp/.h — single-client BSD socket server

**Files:**
- Create: `firmware/mock/net_tcp.h`
- Create: `firmware/mock/net_tcp.cpp`

- [ ] **Step 1: Create net_tcp.h**

```cpp
#pragma once
#include <cstddef>

namespace net_tcp {

// Initialize TCP server on given port. Call once at startup.
bool init(int port);

// Non-blocking poll: accept a new client if none; returns true if a client is connected.
bool poll();

// Send bytes to connected client. Returns false if disconnected.
bool send(const char* data, size_t len);

// Read a newline-terminated line into buf (max len bytes). Returns bytes read, 0 if none, -1 on disconnect.
int readline(char* buf, int max_len);

// Close all connections and server socket.
void shutdown();

}  // namespace net_tcp
```

- [ ] **Step 2: Create net_tcp.cpp**

```cpp
#include "net_tcp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>

static int s_srv  = -1;
static int s_conn = -1;

static void set_nonblocking(int fd) {
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

namespace net_tcp {

bool init(int port) {
    s_srv = socket(AF_INET, SOCK_STREAM, 0);
    if (s_srv < 0) return false;
    int yes = 1;
    setsockopt(s_srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    set_nonblocking(s_srv);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (bind(s_srv, (sockaddr*)&addr, sizeof(addr)) < 0) return false;
    if (listen(s_srv, 1) < 0) return false;
    return true;
}

bool poll() {
    if (s_conn >= 0) return true;
    s_conn = accept(s_srv, nullptr, nullptr);
    if (s_conn < 0) { s_conn = -1; return false; }
    set_nonblocking(s_conn);
    return true;
}

bool send(const char* data, size_t len) {
    if (s_conn < 0) return false;
    ssize_t n = ::send(s_conn, data, len, MSG_NOSIGNAL);
    if (n < 0) { close(s_conn); s_conn = -1; return false; }
    return true;
}

int readline(char* buf, int max_len) {
    if (s_conn < 0) return -1;
    int i = 0;
    while (i < max_len - 1) {
        char c;
        ssize_t n = recv(s_conn, &c, 1, 0);
        if (n == 0) { close(s_conn); s_conn = -1; return -1; }
        if (n < 0) break;  // EAGAIN
        buf[i++] = c;
        if (c == '\n') break;
    }
    buf[i] = '\0';
    return i;
}

void shutdown() {
    if (s_conn >= 0) { close(s_conn); s_conn = -1; }
    if (s_srv  >= 0) { close(s_srv);  s_srv  = -1; }
}

}  // namespace net_tcp
```

- [ ] **Step 3: Verify compilation**

```bash
clang++ -std=c++17 -c firmware/mock/net_tcp.cpp -o /tmp/net_tcp.o && echo ok
```
Expected: `ok`

- [ ] **Step 4: Commit**

```bash
git add firmware/mock/net_tcp.h firmware/mock/net_tcp.cpp
git commit -m "feat(mock): add net_tcp — BSD socket single-client TCP server"
```

---

### Task 18: Platform shim headers

**Files:**
- Create: `firmware/mock/WiFi.h`
- Create: `firmware/mock/ESPmDNS.h`
- Create: `firmware/mock/Wire.h`
- Create: `firmware/mock/esp_compat.h`
- Create: `firmware/mock/freertos_shim.h`
- Create: `firmware/mock/preferences_file.h`

- [ ] **Step 1: Create WiFi.h**

```cpp
#pragma once
// Drop-in <WiFi.h> shim for mock build. Delegates to net_tcp.
#include "net_tcp.h"
#include <string>

#define WL_CONNECTED 3

struct WiFiClass {
    int status() { return WL_CONNECTED; }
    std::string localIP() { return "127.0.0.1"; }
    bool softAP(const char*, const char* = nullptr) { return true; }
    void begin(const char*, const char*) {}
    void disconnect() {}
};
extern WiFiClass WiFi;

struct WiFiServer {
    int _port;
    explicit WiFiServer(int port = 9000) : _port(port) {}
    void begin() { net_tcp::init(_port); }
};

struct WiFiClient {
    bool connected()                          { return net_tcp::poll(); }
    bool available()                          { return net_tcp::poll(); }
    size_t print(const char* s)               { return net_tcp::send(s, strlen(s)) ? strlen(s) : 0; }
    size_t println(const char* s)             {
        size_t n = net_tcp::send(s, strlen(s));
        net_tcp::send("\n", 1);
        return n + 1;
    }
    int readBytesUntil(char, char* buf, int len) { return net_tcp::readline(buf, len); }
    void stop()                               {}
};
```

- [ ] **Step 2: Create ESPmDNS.h**

```cpp
#pragma once
struct MDNSClass {
    bool begin(const char*) { return true; }
    void addService(const char*, const char*, int) {}
};
extern MDNSClass MDNS;
```

- [ ] **Step 3: Create Wire.h**

```cpp
#pragma once
#include <cstdint>
// TwoWire no-op — IMU and sonar are intercepted at the driver level.
struct TwoWire {
    void begin(int = -1, int = -1) {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    int available() { return 0; }
    int read() { return 0; }
    size_t write(uint8_t) { return 1; }
};
extern TwoWire Wire;
```

- [ ] **Step 4: Create esp_compat.h**

```cpp
#pragma once
#include <cstdlib>
#include <cstdint>
struct ESPClass {
    uint32_t getChipId() { return 0xDEADBEEF; }
    void restart() { exit(0); }
};
extern ESPClass ESP;
```

- [ ] **Step 5: Create freertos_shim.h**

```cpp
#pragma once
#include <mutex>
#include <thread>
#include <functional>
#include <cstdint>

using SemaphoreHandle_t = std::mutex*;
using TaskHandle_t      = std::thread*;
using BaseType_t        = int;
using UBaseType_t       = unsigned int;
using TickType_t        = uint32_t;

#define pdTRUE  1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu

inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new std::mutex; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t) { s->lock(); return pdTRUE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { s->unlock(); return pdTRUE; }

inline BaseType_t xTaskCreate(void(*fn)(void*), const char*, uint32_t, void* arg, UBaseType_t, TaskHandle_t*) {
    std::thread* t = new std::thread(fn, arg);
    t->detach();
    return pdTRUE;
}

inline void vTaskDelay(TickType_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
```

- [ ] **Step 6: Create preferences_file.h**

```cpp
#pragma once
// File-backed Preferences shim.
// Stores key-value pairs in ~/.bark-buddy/mock/offsets.json.
#include <string>
#include <unordered_map>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <nlohmann/json.hpp>

class Preferences {
    std::string _ns;
    std::unordered_map<std::string, int32_t> _cache;
    std::filesystem::path _path;

    std::filesystem::path _state_dir() {
        const char* home = getenv("HOME");
        auto p = std::filesystem::path(home ? home : "/tmp") / ".bark-buddy" / "mock";
        std::filesystem::create_directories(p);
        return p;
    }

    void _load() {
        _path = _state_dir() / (_ns + ".json");
        if (!std::filesystem::exists(_path)) return;
        std::ifstream f(_path);
        try {
            nlohmann::json j; f >> j;
            for (auto& [k, v] : j.items())
                _cache[k] = v.get<int32_t>();
        } catch (...) {}
    }

    void _save() {
        nlohmann::json j(_cache);
        std::ofstream f(_path);
        f << j.dump(2) << "\n";
    }

public:
    void begin(const char* ns, bool = false) { _ns = ns; _load(); }
    void end() { _save(); }
    int32_t getInt(const char* key, int32_t def = 0) {
        auto it = _cache.find(key); return it != _cache.end() ? it->second : def;
    }
    void putInt(const char* key, int32_t val) { _cache[key] = val; _save(); }
    bool isKey(const char* key) { return _cache.count(key) > 0; }
};
```

Note: `preferences_file.h` depends on nlohmann/json (header-only). Add to Makefile: `$(shell pkg-config --cflags nlohmann_json 2>/dev/null || echo -I/usr/local/include)`.

- [ ] **Step 7: Commit**

```bash
git add firmware/mock/WiFi.h firmware/mock/ESPmDNS.h firmware/mock/Wire.h \
        firmware/mock/esp_compat.h firmware/mock/freertos_shim.h firmware/mock/preferences_file.h
git commit -m "feat(mock): add platform shim headers (WiFi, mDNS, Wire, ESP, FreeRTOS, Preferences)"
```

---

### Task 19: imu_mock.cpp and sonar_mock.cpp

**Files:**
- Create: `firmware/mock/imu_mock.cpp`
- Create: `firmware/mock/sonar_mock.cpp`

- [ ] **Step 1: Create imu_mock.cpp**

This replaces `firmware/src/imu.cpp` at link time.

```cpp
// Mock IMU driver — reads from physics model instead of real I2C hardware.
#include "../include/imu.h"
#include "physics.h"

bool imu_init() { return true; }

bool imu_read(IMUData& out) {
    physics::IMUSample s;
    physics::imu_sample(s);
    out.pitch = s.pitch_deg;
    out.roll  = s.roll_deg;
    out.yaw   = s.yaw_deg;
    out.ax    = s.ax;
    out.ay    = s.ay;
    out.az    = s.az;
    out.gx    = s.gx;
    out.gy    = s.gy;
    out.gz    = s.gz;
    return true;
}
```

- [ ] **Step 2: Create sonar_mock.cpp**

This replaces `firmware/src/sonar.cpp` at link time.

```cpp
// Mock sonar driver — returns physics model value.
#include "../include/sonar.h"
#include "physics.h"

bool sonar_init() { return true; }

uint16_t sonar_read_mm() {
    return physics::sonar_mm();
}
```

- [ ] **Step 3: Verify imu.h and sonar.h have the right signatures**

```bash
grep -n "imu_read\|IMUData\|sonar_read_mm" \
  firmware/include/imu.h firmware/include/sonar.h
```

Confirm `imu_read(IMUData&)` and `sonar_read_mm()` match the mock implementations. If field names differ, update `imu_mock.cpp` to match.

- [ ] **Step 4: Commit**

```bash
git add firmware/mock/imu_mock.cpp firmware/mock/sonar_mock.cpp
git commit -m "feat(mock): add imu_mock.cpp and sonar_mock.cpp — physics-backed sensor drivers"
```

---

### Task 20: mock_sensor_task.cpp — std::thread at 50/10 Hz

**Files:**
- Create: `firmware/mock/mock_sensor_task.cpp`

This replaces `firmware/src/sensor_task.cpp` at link time to avoid pulling in FreeRTOS headers.

- [ ] **Step 1: Check sensor_task.h public API**

```bash
cat firmware/include/sensor_task.h
```

Note the function signatures (likely `sensor_task_start()`, and a struct for the snapshot).

- [ ] **Step 2: Create mock_sensor_task.cpp**

```cpp
// Mock sensor task — replaces firmware/src/sensor_task.cpp.
// Uses std::thread instead of xTaskCreate to avoid FreeRTOS.
#include "../include/sensor_task.h"
#include "physics.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

static std::mutex s_mtx;
static SensorSnapshot s_snapshot{};
static std::atomic<bool> s_running{false};

static void imu_thread() {
    while (s_running) {
        physics::IMUSample s;
        physics::imu_sample(s);
        {
            std::lock_guard<std::mutex> lk(s_mtx);
            s_snapshot.pitch = s.pitch_deg;
            s_snapshot.roll  = s.roll_deg;
            s_snapshot.yaw   = s.yaw_deg;
            s_snapshot.ax    = s.ax;
            s_snapshot.ay    = s.ay;
            s_snapshot.az    = s.az;
            s_snapshot.gx    = s.gx;
            s_snapshot.gy    = s.gy;
            s_snapshot.gz    = s.gz;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
    }
}

static void sonar_thread() {
    while (s_running) {
        {
            std::lock_guard<std::mutex> lk(s_mtx);
            s_snapshot.sonar_mm = physics::sonar_mm();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz
    }
}

void sensor_task_start() {
    s_running = true;
    std::thread(imu_thread).detach();
    std::thread(sonar_thread).detach();
}

SensorSnapshot sensor_task_snapshot() {
    std::lock_guard<std::mutex> lk(s_mtx);
    return s_snapshot;
}
```

If `SensorSnapshot` field names differ from those above, adjust to match `firmware/include/sensor_task.h`.

- [ ] **Step 3: Verify sensor_task.h fields match**

```bash
grep -n "struct SensorSnapshot\|pitch\|roll\|sonar" firmware/include/sensor_task.h | head -20
```

Update `mock_sensor_task.cpp` field assignments to match exactly.

- [ ] **Step 4: Commit**

```bash
git add firmware/mock/mock_sensor_task.cpp
git commit -m "feat(mock): add mock_sensor_task.cpp — std::thread replaces FreeRTOS task"
```

---

### Task 21: mock_main.cpp — host entry point

**Files:**
- Create: `firmware/mock/mock_main.cpp`

- [ ] **Step 1: Check what setup() and loop() look like in main.cpp**

```bash
grep -n "void setup\|void loop\|WiFiServer\|WiFiClient\|client.readBytesUntil\|send_json\|process_rx" \
  firmware/src/main.cpp | head -30
```

Note: the firmware `setup()` initializes WiFi server, sensors, servos. `loop()` accepts clients, reads NDJSON lines, calls `process_rx()`. The mock replaces WiFi TCP with `net_tcp` via the `WiFi.h` shim.

- [ ] **Step 2: Create mock_main.cpp**

```cpp
// Mock firmware host entry point.
// Parses --tcp-port, runs setup() + loop() from firmware/src/main.cpp,
// and ticks the physics model at ~200 Hz from a separate thread.
#include "physics.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Defined in firmware/src/main.cpp
extern void setup();
extern void loop();

// Physics tick thread
static std::atomic<bool> s_tick_running{true};

static void physics_thread() {
    using clock = std::chrono::steady_clock;
    auto last = clock::now();
    while (s_tick_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;
        if (dt > 0.05f) dt = 0.05f;  // clamp for startup
        physics::tick(dt);
    }
}

// Singletons required by shim headers
WiFiClass  WiFi;
MDNSClass  MDNS;
TwoWire    Wire;
ESPClass   ESP;

static int parse_port(int argc, char** argv) {
    for (int i = 1; i + 1 < argc; i++) {
        if (strcmp(argv[i], "--tcp-port") == 0)
            return atoi(argv[i + 1]);
    }
    return 9001;
}

int main(int argc, char** argv) {
    int port = parse_port(argc, argv);
    printf("[mock] starting on port %d\n", port);

    // Override the port that WiFiServer uses
    // The WiFi.h shim's WiFiServer::begin() calls net_tcp::init(_port).
    // We set the port via environment variable read by WiFiServer constructor,
    // OR simply re-init after setup() calls WiFiServer::begin().
    // Simplest: set an env var the WiFiServer ctor reads.
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", port);
    setenv("BARK_MOCK_TCP_PORT", port_str, 1);

    physics::init();
    std::thread pthr(physics_thread);
    pthr.detach();

    setup();

    while (true) {
        loop();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return 0;
}
```

Note: the WiFiServer shim needs to read `BARK_MOCK_TCP_PORT` env var if set. Update `WiFi.h`:
```cpp
struct WiFiServer {
    int _port;
    explicit WiFiServer(int port = 9000) {
        const char* env = getenv("BARK_MOCK_TCP_PORT");
        _port = (env && *env) ? atoi(env) : port;
    }
    void begin() { net_tcp::init(_port); }
};
```

- [ ] **Step 3: Commit**

```bash
git add firmware/mock/mock_main.cpp firmware/mock/WiFi.h
git commit -m "feat(mock): add mock_main.cpp — host entry point, physics tick thread, port override"
```

---

### Task 22: Makefile — add bark-mock target

**Files:**
- Modify: `firmware/test/Makefile`

- [ ] **Step 1: Read existing Makefile**

Note the pattern for existing test targets — CXX flags, include paths, source lists.

- [ ] **Step 2: Add bark-mock target**

Add to `firmware/test/Makefile`:

```makefile
# bark-mock: real firmware source compiled as host binary with mock platform shims
MOCK_INC = -I../mock -I. -I../include
MOCK_DEFS = -DHOST_BUILD=1 -DMOCK_FIRMWARE=1 -DWIFI_ENABLED=0 -DPINS_VERIFIED=1 \
            -DMOCK_REAL_TIME=1
MOCK_CXXFLAGS = $(CXXFLAGS) $(MOCK_INC) $(MOCK_DEFS) -pthread

# Firmware sources (exclude files replaced by mock)
FW_SRC = ../src/gait.cpp ../src/balance.cpp ../src/servos.cpp \
         ../src/calibrate.cpp ../src/command_handlers.cpp ../src/offsets.cpp \
         ../src/body_transform.cpp

# Mock replacements and new files
MOCK_SRC = ../mock/mock_main.cpp ../mock/physics.cpp ../mock/net_tcp.cpp \
           ../mock/mock_sensor_task.cpp ../mock/imu_mock.cpp ../mock/sonar_mock.cpp

bark-mock: $(FW_SRC) $(MOCK_SRC)
	$(CXX) $(MOCK_CXXFLAGS) -o $@ $^ -pthread
	@echo "bark-mock built"
```

Note: adjust `FW_SRC` to match every `.cpp` in `firmware/src/` except `main.cpp`, `sensor_task.cpp`, `imu.cpp`, `sonar.cpp`. Run `ls firmware/src/*.cpp` to get the full list.

- [ ] **Step 3: Verify the full source list**

```bash
ls /Users/gwen/workspace/bark-buddy/firmware/src/*.cpp
```

Add any missing files to `FW_SRC` (excluding the four replaced by mock).

- [ ] **Step 4: Build**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make bark-mock
```
Expected: compiles and links without errors.

- [ ] **Step 5: Smoke test — start mock, check port opens**

```bash
./bark-mock --tcp-port 19001 &
MOCK_PID=$!
sleep 1
nc -z 127.0.0.1 19001 && echo "port open" || echo "port closed"
kill $MOCK_PID
```
Expected: `port open`

- [ ] **Step 6: Commit**

```bash
git add firmware/test/Makefile
git commit -m "feat(mock): add bark-mock build target to firmware/test/Makefile"
```

---

### Task 23: Documentation sweep

**Files:**
- Modify: `CLAUDE.md`
- Modify: `docs/architecture.md`
- Modify: `README.md`

- [ ] **Step 1: Update CLAUDE.md**

- Remove "Stock firmware (fallback)" section from Architecture.
- Update "Two firmware paths, same Python host and web UI" → "One firmware path (custom C++).".
- Remove "Stock firmware caveat" section entirely.
- Remove hybrid/ from Project Layout.
- Remove stock-firmware-only files from host/ layout (hybrid_transport, repl_transport, webrepl_transport, hw_transport, setup_wifi, capture_profile, monitor_pins).
- Remove sim/ from host/ layout; add "behaviors/mock not applicable" note.
- Update Conventions: remove `--sim`, `--wifi` overrides; add `--fw-tcp HOST[:PORT]` for explicit TCP connection.
- Update CLI subcommands: surviving = `bark` (serve + auto-detect), `bark mock`, `bark flash`, `bark test`, `bark kill`. Removed: `bark sim`, `bark wifi-setup`.

- [ ] **Step 2: Update docs/architecture.md if it exists**

```bash
ls /Users/gwen/workspace/bark-buddy/docs/architecture.md 2>/dev/null && echo exists
```

If it exists: remove stock/hybrid section, update transport diagram to show FirmwareTransport only, add mock firmware branch.

- [ ] **Step 3: Update README.md if it mentions sim or stock firmware**

```bash
grep -n "sim\|stock\|wifi-setup\|WebREPL" /Users/gwen/workspace/bark-buddy/README.md 2>/dev/null | head -20
```

Update any references.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md docs/ README.md
git commit -m "docs: update CLAUDE.md, architecture, README — single firmware path + bark mock"
```

---

### Task 24: Final verification

- [ ] **Step 1: Full Python compile check**

```bash
python3 -m compileall host/ bark_cli.py -q
```
Expected: no errors

- [ ] **Step 2: Grep for dead code — zero references**

```bash
grep -rn "DogComms\|SimTransport\|sim_transport\|hybrid_transport\|repl_transport\|webrepl\|WebREPL\|cmd_sim_noise\|telem_ultrasonic\|WEBREPL_PASS" \
  host/ bark_cli.py web/ firmware/include/ firmware/src/ \
  --include="*.py" --include="*.js" --include="*.html" --include="*.h" --include="*.cpp" \
  | grep -v "\.pyc\|#.*historical\|docs/superpowers" \
  | grep -v "^Binary"
```
Expected: zero matches (or only in historical spec/plan docs — those are fine)

- [ ] **Step 3: Run host test suite**

```bash
python3 -m pytest host/ -v --tb=short
```

- [ ] **Step 4: Build firmware/test targets**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make
make bark-mock
```
Expected: all pass

- [ ] **Step 5: End-to-end mock test**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
./bark-mock --tcp-port 19002 &
MOCK_PID=$!
sleep 1
python3 - <<'PYEOF'
import asyncio, sys
sys.path.insert(0, 'host')
from firmware_transport import FirmwareTransport

async def test():
    t = FirmwareTransport(host="127.0.0.1", tcp_port=19002)
    await t.open()
    print("opened")
    info = t.firmware_info
    print("boot:", info)
    await t.send_json({"type": "cmd_stand"})
    ack = await t.recv_ack("cmd_stand", timeout=2.0)
    print("ack:", ack)
    await t.close()
    print("closed")

asyncio.run(test())
PYEOF
kill $MOCK_PID
```
Expected: `opened`, boot dict with `fw_version`, `ack: {...ok: True}`, `closed`

- [ ] **Step 6: CLI smoke test**

```bash
python3 bark_cli.py --help
python3 bark_cli.py mock --help
```
Expected: help text shows `mock` subcommand, no `sim`, no `wifi-setup`.

- [ ] **Step 7: Final commit**

```bash
git commit --allow-empty -m "chore: Phase C complete — mock firmware binary, bark mock subcommand"
```
