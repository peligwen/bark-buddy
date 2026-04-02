"""
Communication layer for the MechDog stock firmware.

Sends CMD protocol commands over serial and parses responses.
Abstract Transport base allows mock transport for development.
"""

import abc
import asyncio
import logging
from typing import Optional

logger = logging.getLogger(__name__)

# Serial constants
SERIAL_BAUD = 115200
READ_TIMEOUT = 0.5  # seconds

# CMD protocol motion sub-codes
MOTION_STOP = 1
MOTION_STAND = 2
MOTION_FORWARD = 3
MOTION_BACKWARD = 4
MOTION_TURN_LEFT = 5
MOTION_TURN_RIGHT = 6
MOTION_SHIFT_LEFT = 7
MOTION_SHIFT_RIGHT = 8

# Direction string to motion sub-code mapping
DIRECTION_MAP = {
    "stop": MOTION_STOP,
    "stand": MOTION_STAND,
    "forward": MOTION_FORWARD,
    "backward": MOTION_BACKWARD,
    "left": MOTION_TURN_LEFT,
    "right": MOTION_TURN_RIGHT,
    "shift_left": MOTION_SHIFT_LEFT,
    "shift_right": MOTION_SHIFT_RIGHT,
}

# Action group codes
ACTION_WAVE = 1
ACTION_STRETCH = 2
ACTION_TURN_AROUND = 3
ACTION_SIT = 4
ACTION_LIE_DOWN = 5


class Transport(abc.ABC):
    """Abstract transport -- serial or mock."""

    @abc.abstractmethod
    async def open(self) -> None: ...

    @abc.abstractmethod
    async def close(self) -> None: ...

    @abc.abstractmethod
    async def send(self, data: str) -> None: ...

    @abc.abstractmethod
    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]: ...

    @abc.abstractmethod
    def is_open(self) -> bool: ...


class SerialTransport(Transport):
    """USB serial transport using pyserial-asyncio."""

    def __init__(self, port: str, baudrate: int = SERIAL_BAUD):
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

    async def send(self, data: str) -> None:
        if not self._writer:
            raise ConnectionError("Serial not open")
        self._writer.write(data.encode("ascii"))
        await self._writer.drain()

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._reader:
            raise ConnectionError("Serial not open")
        try:
            data = await asyncio.wait_for(
                self._reader.readuntil(b"$"), timeout=timeout
            )
            return data.decode("ascii").strip()
        except asyncio.TimeoutError:
            return None
        except asyncio.IncompleteReadError:
            return None

    def is_open(self) -> bool:
        return self._writer is not None


class DogComms:
    """
    High-level communication with the MechDog via CMD protocol.

    Translates Python method calls into CMD|...|$ strings,
    sends them over the transport, and parses responses.
    """

    def __init__(self, transport: Transport):
        self._transport = transport
        self._connected = False
        self._lock = asyncio.Lock()

    async def connect(self) -> None:
        await self._transport.open()
        self._connected = True
        logger.info("DogComms connected")

    async def disconnect(self) -> None:
        self._connected = False
        await self._transport.close()
        logger.info("DogComms disconnected")

    @property
    def connected(self) -> bool:
        return self._connected

    # --- Movement ---

    async def move(self, direction: str) -> bool:
        """Send a motion command. direction: forward/backward/left/right/stop/stand/shift_left/shift_right"""
        code = DIRECTION_MAP.get(direction, MOTION_STOP)
        return await self._send_cmd(3, code)

    async def move_forward(self) -> bool:
        return await self.move("forward")

    async def move_backward(self) -> bool:
        return await self.move("backward")

    async def turn_left(self) -> bool:
        return await self.move("left")

    async def turn_right(self) -> bool:
        return await self.move("right")

    async def stop(self) -> bool:
        return await self.move("stop")

    async def stand(self) -> bool:
        return await self.move("stand")

    # --- Balance ---

    async def set_balance(self, enabled: bool) -> bool:
        """Toggle the stock self-balance mode."""
        return await self._send_cmd(1, 3, 1 if enabled else 0)

    async def enable_balance(self) -> bool:
        return await self.set_balance(True)

    async def disable_balance(self) -> bool:
        return await self.set_balance(False)

    # --- Posture ---

    async def set_pitch(self, angle: int) -> bool:
        return await self._send_cmd(1, 1, angle)

    async def set_roll(self, angle: int) -> bool:
        return await self._send_cmd(1, 2, angle)

    async def set_height(self, height: int) -> bool:
        return await self._send_cmd(1, 4, height)

    # --- Actions ---

    async def action(self, action_code: int) -> bool:
        """Trigger an action group (1=wave, 2=stretch, 3=turn, 4=sit, 5=lie)."""
        return await self._send_cmd(2, 1, action_code)

    # --- Telemetry ---

    async def read_imu(self) -> Optional[dict]:
        """Read IMU tilt angles. Returns {"pitch": float, "roll": float} or None."""
        resp = await self._send_and_recv("CMD|5|$")
        if resp:
            parts = self._parse_response(resp)
            if parts and len(parts) >= 3 and parts[0] == "5":
                try:
                    return {"pitch": float(parts[1]), "roll": float(parts[2])}
                except (ValueError, IndexError):
                    logger.warning("Bad IMU response: %s", resp)
        return None

    async def read_battery(self) -> Optional[int]:
        """Read battery voltage in millivolts. Returns int or None."""
        resp = await self._send_and_recv("CMD|6|$")
        if resp:
            parts = self._parse_response(resp)
            if parts and len(parts) >= 2 and parts[0] == "6":
                try:
                    return int(parts[1])
                except (ValueError, IndexError):
                    logger.warning("Bad battery response: %s", resp)
        return None

    # --- Internal ---

    async def _send_cmd(self, func: int, *args: int) -> bool:
        """Build and send a CMD string, check for OK ack."""
        cmd_parts = [str(func)] + [str(a) for a in args]
        cmd = "CMD|" + "|".join(cmd_parts) + "|$"
        resp = await self._send_and_recv(cmd)
        if resp:
            parts = self._parse_response(resp)
            return parts is not None and "OK" in parts
        return False

    async def _send_and_recv(self, cmd: str) -> Optional[str]:
        """Send a CMD string and wait for a response."""
        async with self._lock:
            logger.debug("TX: %s", cmd)
            try:
                await self._transport.send(cmd)
                resp = await self._transport.recv()
                if resp:
                    logger.debug("RX: %s", resp)
                return resp
            except ConnectionError:
                logger.warning("Connection lost during send/recv")
                self._connected = False
                return None

    @staticmethod
    def _parse_response(resp: str) -> Optional[list[str]]:
        """Parse a CMD|...|$ response into its pipe-separated parts."""
        resp = resp.strip()
        if resp.startswith("CMD|") and resp.endswith("|$"):
            inner = resp[4:-2]  # strip CMD| and |$
            return inner.split("|")
        return None
