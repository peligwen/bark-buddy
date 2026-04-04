"""
Hybrid transport for MechDog.

Bootstraps the MicroPython NDJSON handler via REPL, then switches to
streaming NDJSON mode. Combines stock firmware's proven servo control
with the structured JSON protocol of the custom firmware.

Bootstrap sequence:
  1. Open serial, get clean REPL prompt (Ctrl+C)
  2. Upload handler.py to the ESP32 filesystem
  3. exec(open('handler.py').read()) — starts the handler
  4. Wait for {"type": "boot"} message
  5. From here on, serial is pure NDJSON (no more REPL)
"""

import asyncio
import json
import logging
import math
import os
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)

# Path to the MicroPython handler script
HANDLER_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "firmware", "hybrid", "handler.py"
)


class HybridTransport(Transport):
    """
    Transport that bootstraps MicroPython NDJSON handler then speaks JSON.

    After bootstrap, behaves identically to FirmwareTransport — streams
    telemetry, caches sensor values, sends JSON commands.
    """

    FORWARD_SPEED = 0.10
    TURN_SPEED = 45.0

    def __init__(self, port: str, baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._open = False
        self._reader_task: Optional[asyncio.Task] = None
        self._last_response: Optional[str] = None

        # Telemetry cache
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm = 0
        self._battery_mv = 7400
        self._firmware_info = {}

        # Dead reckoning
        self._heading = 0.0
        self._x = 0.0
        self._y = 0.0
        self._motion_cmd = 1
        self._last_motion_time = 0.0

    async def open(self) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        self._open = True

        # Bootstrap the handler
        await self._bootstrap()

        # Start background reader for streaming telemetry
        self._reader_task = asyncio.create_task(self._reader_loop())
        logger.info("HybridTransport ready on %s", self._port)

    async def close(self) -> None:
        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None
        self._open = False
        logger.info("HybridTransport closed")

    async def send(self, data: str) -> None:
        """Translate CMD protocol to JSON and send."""
        if not self._open:
            raise ConnectionError("Not open")

        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return

        inner = data[4:-2]
        parts = inner.split("|")
        if not parts:
            return

        response = await self._handle_cmd(parts[0], parts[1:])
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError("Not open")
        resp = self._last_response
        self._last_response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._motion_cmd = 1
        self._last_motion_time = time.monotonic()

    def get_position(self) -> tuple[float, float, float]:
        self._step_dead_reckoning()
        return (self._x, self._y, 0.0)

    def get_heading(self) -> float:
        if self._imu.get("yaw", 0) != 0:
            return self._imu["yaw"]
        self._step_dead_reckoning()
        return self._heading

    # --- Bootstrap ---

    async def _bootstrap(self) -> None:
        """Upload handler.py to MicroPython and exec it."""
        logger.info("Bootstrapping hybrid handler...")

        # Get clean REPL prompt
        await asyncio.sleep(1)
        self._writer.write(b'\x03\x03\r\n')
        await self._writer.drain()
        await asyncio.sleep(0.5)
        await self._drain()

        # Read handler source
        with open(HANDLER_PATH, "r") as f:
            handler_src = f.read()

        # Upload handler as a file on the ESP32 filesystem
        # Split into chunks to avoid REPL buffer overflow
        await self._repl_exec("f=open('_hybrid.py','w')")
        chunk_size = 128
        for i in range(0, len(handler_src), chunk_size):
            chunk = handler_src[i:i + chunk_size]
            # Escape for Python string literal
            escaped = chunk.replace("\\", "\\\\").replace("'", "\\'").replace("\n", "\\n").replace("\r", "")
            await self._repl_exec(f"f.write('{escaped}')")
        await self._repl_exec("f.close()")
        logger.info("Handler uploaded (%d bytes)", len(handler_src))

        # Execute the handler — this starts the NDJSON loop
        # After this, REPL is gone and serial is pure NDJSON
        self._writer.write(b"exec(open('_hybrid.py').read())\r\n")
        await self._writer.drain()

        # Wait for boot message
        boot_ok = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                line = await asyncio.wait_for(self._reader.readline(), timeout=2.0)
                text = line.decode(errors="replace").strip()
                if text.startswith("{"):
                    try:
                        msg = json.loads(text)
                        if msg.get("type") == "boot":
                            self._firmware_info = msg
                            logger.info("Hybrid handler booted: %s", msg)
                            boot_ok = True
                            break
                    except json.JSONDecodeError:
                        pass
            except asyncio.TimeoutError:
                continue

        if not boot_ok:
            raise ConnectionError("Hybrid handler did not send boot message")

    async def _repl_exec(self, cmd: str) -> None:
        """Send a single-line command to REPL and wait for prompt."""
        self._writer.write((cmd + "\r\n").encode())
        await self._writer.drain()
        await asyncio.sleep(0.05)
        await self._drain()

    async def _drain(self) -> None:
        """Drain pending input from the reader."""
        if not self._reader:
            return
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.1)
        except asyncio.TimeoutError:
            pass

    # --- CMD translation (same as FirmwareTransport) ---

    async def _handle_cmd(self, func: str, args: list[str]) -> Optional[str]:
        if func == "1":
            sub = args[0] if args else "3"
            val = args[1] if len(args) > 1 else "0"
            if sub == "3":
                await self._send_json({"type": "cmd_balance", "enabled": val == "1"})
            return "CMD|1|OK|$"

        elif func == "2":
            code = int(args[1]) if len(args) > 1 else 1
            await self._send_json({"type": "cmd_stand"})
            return "CMD|2|OK|$"

        elif func == "3":
            sub = int(args[0]) if args else 1
            self._step_dead_reckoning()
            self._motion_cmd = sub
            self._last_motion_time = time.monotonic()
            dir_map = {
                1: "stop", 2: "stop", 3: "forward",
                4: "backward", 5: "left", 6: "right"
            }
            direction = dir_map.get(sub, "stop")
            if sub == 2:
                await self._send_json({"type": "cmd_stand"})
            else:
                await self._send_json({"type": "cmd_move", "direction": direction})
            return "CMD|3|OK|$"

        elif func == "4":
            return f"CMD|4|{self._sonar_mm}|$"

        elif func == "5":
            return f"CMD|5|{self._imu['pitch']}|{self._imu['roll']}|$"

        elif func == "6":
            return f"CMD|6|{self._battery_mv}|$"

        return None

    # --- Background reader ---

    async def _reader_loop(self):
        """Read streaming NDJSON, update telemetry cache."""
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
            logger.warning("Hybrid reader error: %s", e)

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
        elif msg_type == "boot":
            self._firmware_info = msg
            logger.info("Handler boot: %s", msg)
        elif msg_type == "pong":
            logger.debug("Handler pong")
        elif msg_type == "error":
            logger.warning("Handler error: %s", msg.get("msg", ""))

    # --- JSON I/O ---

    async def _send_json(self, msg: dict) -> None:
        if not self._writer:
            return
        data = json.dumps(msg) + "\n"
        self._writer.write(data.encode())
        await self._writer.drain()

    # --- Dead reckoning ---

    def _step_dead_reckoning(self) -> None:
        now = time.monotonic()
        dt = now - self._last_motion_time if self._last_motion_time > 0 else 0
        if dt <= 0 or dt > 1.0:
            self._last_motion_time = now
            return
        self._last_motion_time = now
        cmd = self._motion_cmd
        if cmd == 3:
            rad = math.radians(self._heading)
            self._x += self.FORWARD_SPEED * dt * math.cos(rad)
            self._y += self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 4:
            rad = math.radians(self._heading)
            self._x -= self.FORWARD_SPEED * dt * math.cos(rad)
            self._y -= self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 5:
            self._heading -= self.TURN_SPEED * dt
        elif cmd == 6:
            self._heading += self.TURN_SPEED * dt
