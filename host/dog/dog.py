"""
Dog — transport for custom MechDog firmware.

Connects over USB serial or WiFi TCP. All commands sent via send_json().
Firmware pushes NDJSON telemetry; cached values updated by background reader.
"""

import asyncio
import json
import logging

from .io import DogIO

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

    def __init__(self, port: str | None = None, host: str | None = None, tcp_port: int = 9000,
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
        self._battery_present: bool = True
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
                pass  # transport may already be dead; shutdown should proceed
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

    def get_battery_present(self) -> bool:
        return self._battery_present

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

    async def recv_ack(self, ref_type: str, timeout: float = 2.0) -> dict | None:
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
        text = line.decode(errors="replace").strip()
        if not text:
            return
        decoder = json.JSONDecoder()
        pos = 0
        while pos < len(text):
            brace = text.find('{', pos)
            if brace == -1:
                if pos == 0:
                    logger.debug("Dog: non-JSON line (ignored): %r", text[:80])
                break
            pos = brace
            try:
                msg, end = decoder.raw_decode(text, pos)
                self._handle_telem(msg)
                pos = end
            except json.JSONDecodeError as e:
                logger.warning("Dog: bad JSON (ignored): %s | raw=%r", e, text[pos:pos+60])
                break

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
            self._battery_present = msg.get("present", True)
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
