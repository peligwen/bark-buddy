"""
NDJSON transport for custom MechDog firmware.

Connects to the custom C++ firmware over serial or WiFi TCP.
The firmware pushes telemetry (IMU 50Hz, sonar 20Hz) — no polling needed.
Commands are sent as JSON, responses come as ack messages.
"""

import asyncio
import json
import logging
import math
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)


class FirmwareTransport(Transport):
    """
    Transport for custom firmware speaking NDJSON over serial or TCP.

    Unlike REPL transports, this receives streaming telemetry from the
    firmware. Sensor reads return cached values from the stream.
    """

    FORWARD_SPEED = 0.10
    TURN_SPEED = 45.0

    def __init__(self, port: str = None, host: str = None, tcp_port: int = 9000):
        self._port = port
        self._host = host
        self._tcp_port = tcp_port
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._open = False
        self._reader_task: Optional[asyncio.Task] = None
        self._last_response: Optional[str] = None

        # Telemetry cache (updated by reader task)
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
        if self._port:
            import serial_asyncio
            self._reader, self._writer = await serial_asyncio.open_serial_connection(
                url=self._port, baudrate=115200
            )
            await asyncio.sleep(1)
        elif self._host:
            self._reader, self._writer = await asyncio.open_connection(
                self._host, self._tcp_port
            )
        else:
            raise ConnectionError("No port or host specified")

        self._open = True

        # Send ping to verify firmware
        await self._send_json({"type": "ping"})
        await asyncio.sleep(0.5)

        # Start background reader for streaming telemetry
        self._reader_task = asyncio.create_task(self._reader_loop())

        logger.info("FirmwareTransport opened on %s",
                     self._port or f"{self._host}:{self._tcp_port}")

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
        logger.info("FirmwareTransport closed")

    async def send(self, data: str) -> None:
        """Translate CMD protocol to JSON and send. Cache telemetry responses."""
        if not self._open:
            raise ConnectionError("Firmware not open")

        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return

        inner = data[4:-2]
        parts = inner.split("|")
        if not parts:
            return

        func = parts[0]
        response = await self._handle_cmd(func, parts[1:])
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError("Firmware not open")
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
        # Use IMU yaw if available, otherwise dead reckoning
        if self._imu.get("yaw", 0) != 0:
            return self._imu["yaw"]
        self._step_dead_reckoning()
        return self._heading

    # --- CMD translation ---

    async def _handle_cmd(self, func: str, args: list[str]) -> Optional[str]:
        if func == "1":
            # Balance
            sub = args[0] if args else "3"
            val = args[1] if len(args) > 1 else "0"
            if sub == "3":
                await self._send_json({"type": "cmd_balance", "enabled": val == "1"})
            return "CMD|1|OK|$"

        elif func == "2":
            # Action
            code = int(args[1]) if len(args) > 1 else 1
            await self._send_json({"type": "cmd_stand"})
            return "CMD|2|OK|$"

        elif func == "3":
            # Motion
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
            # Sonar — return cached value
            return f"CMD|4|{self._sonar_mm}|$"

        elif func == "5":
            # IMU — return cached value
            return f"CMD|5|{self._imu['pitch']}|{self._imu['roll']}|$"

        elif func == "6":
            # Battery — return cached value
            return f"CMD|6|{self._battery_mv}|$"

        return None

    # --- Background reader ---

    async def _reader_loop(self):
        """Read streaming NDJSON from firmware, update telemetry cache."""
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
            logger.warning("Firmware reader error: %s", e)

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
            # Cache WiFi info from status messages
            if msg.get("wifi") and msg.get("wifi_ip"):
                self._firmware_info["wifi_ip"] = msg["wifi_ip"]
                self._firmware_info["tcp_port"] = msg.get("tcp_port", 9000)
        elif msg_type == "boot":
            self._firmware_info = msg
            logger.info("Firmware boot: %s", msg)
        elif msg_type == "pong":
            logger.debug("Firmware pong")

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
