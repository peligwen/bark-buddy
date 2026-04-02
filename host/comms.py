"""
Abstract transport layer for communicating with MechDog firmware.

Supports serial (USB) now, WiFi (TCP) later. Both use NDJSON framing.
"""

import abc
import asyncio
import json
import logging
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# Protocol constants (must match firmware/include/protocol.h)
MSG_PING = "ping"
MSG_PONG = "pong"
MSG_ACK = "ack"
MSG_CMD_MOVE = "cmd_move"
MSG_CMD_STAND = "cmd_stand"
MSG_CMD_SET_GAIT = "cmd_set_gait"
MSG_CMD_PATROL = "cmd_patrol"
MSG_CMD_BALANCE = "cmd_balance"
MSG_TELEM_IMU = "telem_imu"
MSG_TELEM_STATUS = "telem_status"
MSG_EVENT_FALL = "event_fall"
MSG_EVENT_RECOVERED = "event_recovered"

HEARTBEAT_INTERVAL = 2.0  # seconds
HEARTBEAT_TIMEOUT = 5.0   # seconds


class Transport(abc.ABC):
    """Abstract transport — serial or WiFi."""

    @abc.abstractmethod
    async def open(self) -> None: ...

    @abc.abstractmethod
    async def close(self) -> None: ...

    @abc.abstractmethod
    async def readline(self) -> str: ...

    @abc.abstractmethod
    async def writeline(self, line: str) -> None: ...

    @abc.abstractmethod
    def is_open(self) -> bool: ...


class SerialTransport(Transport):
    """USB serial transport using pyserial-asyncio."""

    def __init__(self, port: str, baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None

    async def open(self) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        logger.info("Serial connected: %s @ %d", self._port, self._baudrate)

    async def close(self) -> None:
        if self._writer:
            self._writer.close()
            self._reader = None
            self._writer = None

    async def readline(self) -> str:
        if not self._reader:
            raise ConnectionError("Serial not open")
        line = await self._reader.readline()
        return line.decode("utf-8").strip()

    async def writeline(self, line: str) -> None:
        if not self._writer:
            raise ConnectionError("Serial not open")
        self._writer.write((line + "\n").encode("utf-8"))
        await self._writer.drain()

    def is_open(self) -> bool:
        return self._writer is not None


class DogComms:
    """
    High-level comms with the MechDog firmware.

    Handles message serialization, heartbeat, reconnection, and dispatching
    incoming messages to registered handlers.
    """

    def __init__(self, transport: Transport):
        self._transport = transport
        self._handlers: dict[str, list[Callable]] = {}
        self._connected = False
        self._recv_task: Optional[asyncio.Task] = None
        self._heartbeat_task: Optional[asyncio.Task] = None

    # --- Public API ---

    async def connect(self) -> None:
        """Open transport and start background tasks."""
        await self._transport.open()
        self._connected = True
        self._recv_task = asyncio.create_task(self._recv_loop())
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
        logger.info("DogComms connected")

    async def disconnect(self) -> None:
        """Stop background tasks and close transport."""
        self._connected = False
        for task in (self._recv_task, self._heartbeat_task):
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        await self._transport.close()
        logger.info("DogComms disconnected")

    async def send(self, msg: dict[str, Any]) -> None:
        """Send a JSON message to firmware."""
        line = json.dumps(msg, separators=(",", ":"))
        logger.debug("TX: %s", line)
        await self._transport.writeline(line)

    async def send_move(self, direction: str, speed: float = 0.5) -> None:
        await self.send({"type": MSG_CMD_MOVE, "direction": direction, "speed": speed})

    async def send_stand(self) -> None:
        await self.send({"type": MSG_CMD_STAND})

    async def send_balance(self, enabled: bool) -> None:
        await self.send({"type": MSG_CMD_BALANCE, "enabled": enabled})

    async def send_ping(self) -> None:
        await self.send({"type": MSG_PING})

    def on(self, msg_type: str, handler: Callable) -> None:
        """Register a handler for a message type."""
        self._handlers.setdefault(msg_type, []).append(handler)

    @property
    def connected(self) -> bool:
        return self._connected

    # --- Internal ---

    async def _recv_loop(self) -> None:
        """Read messages from transport and dispatch to handlers."""
        while self._connected:
            try:
                line = await self._transport.readline()
                if not line:
                    continue
                msg = json.loads(line)
                logger.debug("RX: %s", msg)
                msg_type = msg.get("type")
                if msg_type and msg_type in self._handlers:
                    for handler in self._handlers[msg_type]:
                        try:
                            result = handler(msg)
                            if asyncio.iscoroutine(result):
                                await result
                        except Exception:
                            logger.exception("Handler error for %s", msg_type)
            except json.JSONDecodeError:
                logger.warning("Invalid JSON received")
            except ConnectionError:
                logger.warning("Connection lost")
                self._connected = False
                break
            except asyncio.CancelledError:
                break

    async def _heartbeat_loop(self) -> None:
        """Send periodic pings."""
        while self._connected:
            try:
                await self.send_ping()
                await asyncio.sleep(HEARTBEAT_INTERVAL)
            except ConnectionError:
                logger.warning("Heartbeat failed — connection lost")
                self._connected = False
                break
            except asyncio.CancelledError:
                break
