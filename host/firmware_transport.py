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
        self._engaged = False
        self._ramping = False
        self._battery_cutoff = False

        # Ack queue for tools that need to wait on specific ack messages
        self._ack_queue: asyncio.Queue = asyncio.Queue(maxsize=64)

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
                await self._send_json({"type": "cmd_engage", "enabled": False})
                await self.recv_ack("cmd_engage", timeout=1.0)
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

    def get_engaged(self) -> bool:
        return self._engaged

    def get_ramping(self) -> bool:
        return self._ramping

    def get_battery_cutoff(self) -> bool:
        return self._battery_cutoff

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
                    logger.warning("FirmwareTransport EOF — TCP connection closed")
                    self._open = False
                    break
                try:
                    msg = json.loads(line.decode(errors="replace").strip())
                    self._handle_telem(msg)
                except json.JSONDecodeError as e:
                    logger.warning("FirmwareTransport bad JSON (ignored): %s", e)
                except UnicodeDecodeError:
                    pass
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.warning("FirmwareTransport reader error: %s", e)
            self._open = False

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
            logger.info("FirmwareTransport boot: %s", msg)
        elif msg_type == "ack":
            try:
                self._ack_queue.put_nowait(msg)
            except asyncio.QueueFull:
                logger.debug("FirmwareTransport ack queue full — dropping oldest entry")
                self._ack_queue.get_nowait()
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

    # --- Helper methods for new firmware commands ---

    async def cmd_buzzer(self, freq_hz: int, duration_ms: int) -> None:
        await self.send_json({"type": "cmd_buzzer", "freq_hz": freq_hz, "duration_ms": duration_ms})

    async def cmd_gpio_mode(self, pin: int, mode: str) -> None:
        """mode: 'input_floating' | 'input_pullup' | 'input_pulldown' | 'output'"""
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
        await self.send_json({"type": "cmd_i2c", "op": "read", "addr": addr, "reg": reg, "len": length, "bus": bus})

    async def cmd_i2c_write(self, addr: int, reg: int, val: int, bus: int = 1) -> None:
        await self.send_json({"type": "cmd_i2c", "op": "write", "addr": addr, "reg": reg, "val": val, "bus": bus})

    async def cmd_aux_servo(self, index: int, pulse_us: int) -> None:
        """index: 8-10 for aux servo ports"""
        await self.send_json({"type": "cmd_servo", "index": index, "pulse_us": pulse_us})

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
