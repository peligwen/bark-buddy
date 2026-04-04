"""
Shared base for JSON-streaming transports (FirmwareTransport, HybridTransport).

Both transports speak the same NDJSON protocol post-connection: they push
telemetry, accept JSON commands, and translate the CMD protocol to JSON.
This base class holds all that shared logic so neither subclass duplicates it.

Subclasses implement open() with their own connection setup (serial vs TCP,
bootstrap vs ping). close() is provided here and is identical for both.
"""

import asyncio
import json
import logging
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT
from dead_reckoning import DeadReckoningMixin

logger = logging.getLogger(__name__)


class JsonStreamTransport(DeadReckoningMixin, Transport):
    """
    Abstract base for transports that speak streaming NDJSON.

    Provides: CMD→JSON translation, background telemetry reader,
    telemetry cache, JSON I/O, dead reckoning.
    Subclasses must implement open() and provide a log_prefix class attribute.
    """

    log_prefix = "JsonStream"  # Override in subclass for cleaner log messages

    def __init__(self):
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._open = False
        self._reader_task: Optional[asyncio.Task] = None
        self._last_response: Optional[str] = None

        # Telemetry cache (updated by reader task at 50Hz IMU / 20Hz sonar)
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm = 0
        self._battery_mv = 7400
        self._firmware_info = {}

        # Ack queue for tools that need to wait on specific ack messages
        self._ack_queue: asyncio.Queue = asyncio.Queue()

        self._init_dead_reckoning()

    @property
    def firmware_info(self) -> dict:
        """Boot/status info from firmware (WiFi IP, TCP port, etc.)."""
        return self._firmware_info

    # --- Transport ABC implementation ---

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
        logger.info("%s closed", self.log_prefix)

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError(f"{self.log_prefix} not open")
        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return
        parts = data[4:-2].split("|")
        if not parts:
            return
        response = await self._handle_cmd(parts[0], parts[1:])
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError(f"{self.log_prefix} not open")
        resp = self._last_response
        self._last_response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    def get_heading(self) -> float:
        """Return IMU yaw if available, otherwise dead-reckoned heading."""
        if self._imu.get("yaw", 0) != 0:
            return self._imu["yaw"]
        self._step_dead_reckoning()
        return self._heading

    # --- Public telemetry accessors (for tools that bypass CMD protocol) ---

    def get_imu(self) -> dict:
        """Current IMU reading from telemetry cache."""
        return dict(self._imu)

    def get_sonar_mm(self) -> int:
        """Current sonar reading from telemetry cache."""
        return self._sonar_mm

    def get_battery_mv(self) -> int:
        """Current battery voltage from telemetry cache."""
        return self._battery_mv

    async def send_json(self, msg: dict) -> None:
        """Send a raw JSON message to firmware. For tools that speak JSON directly."""
        if not self._open:
            raise ConnectionError(f"{self.log_prefix} not open")
        await self._send_json(msg)

    async def recv_ack(self, ref_type: str, timeout: float = 2.0) -> Optional[dict]:
        """Wait for a specific ack message from the firmware telemetry stream."""
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
                # Wrong ref_type — keep draining until we find a match or time out
            except asyncio.TimeoutError:
                return None

    # --- CMD → JSON translation ---

    async def _handle_cmd(self, func: str, args: list[str]) -> Optional[str]:
        if func == "1":
            sub = args[0] if args else "3"
            val = args[1] if len(args) > 1 else "0"
            if sub == "3":
                await self._send_json({"type": "cmd_balance", "enabled": val == "1"})
            return "CMD|1|OK|$"

        elif func == "2":
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

    # --- Background telemetry reader ---

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
            logger.warning("%s reader error: %s", self.log_prefix, e)

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
            if msg.get("wifi") and msg.get("wifi_ip"):
                self._firmware_info["wifi_ip"] = msg["wifi_ip"]
                self._firmware_info["tcp_port"] = msg.get("tcp_port", 9000)
        elif msg_type == "boot":
            self._firmware_info = msg
            logger.info("%s boot: %s", self.log_prefix, msg)
        elif msg_type == "ack":
            self._ack_queue.put_nowait(msg)
            if not msg.get("ok"):
                logger.warning("%s NACK: %s — %s", self.log_prefix,
                               msg.get("ref_type", "?"), msg.get("error", "?"))
        elif msg_type == "pong":
            logger.debug("%s pong", self.log_prefix)
        elif msg_type == "error":
            logger.warning("%s error: %s", self.log_prefix, msg.get("msg", ""))

    # --- JSON I/O ---

    async def _send_json(self, msg: dict) -> None:
        if not self._writer:
            return
        self._writer.write((json.dumps(msg) + "\n").encode())
        await self._writer.drain()

