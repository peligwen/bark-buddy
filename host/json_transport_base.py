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
import math
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)


class JsonStreamTransport(Transport):
    """
    Abstract base for transports that speak streaming NDJSON.

    Provides: CMD→JSON translation, background telemetry reader,
    telemetry cache, JSON I/O, dead reckoning.
    Subclasses must implement open() and provide a log_prefix class attribute.
    """

    FORWARD_SPEED = 0.10
    TURN_SPEED = 45.0
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

        # Dead reckoning
        self._heading = 0.0
        self._x = 0.0
        self._y = 0.0
        self._motion_cmd = 1
        self._last_motion_time = 0.0

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
