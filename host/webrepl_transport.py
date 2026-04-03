"""
WiFi WebREPL transport for MechDog.

Connects to the MicroPython WebREPL WebSocket on the MechDog's WiFi IP.
Command translation is handled by the HardwareTransport base class.
"""

import asyncio
import logging
import time

from hw_transport import HardwareTransport

logger = logging.getLogger(__name__)


class WebReplTransport(HardwareTransport):
    """MechDog transport over WiFi WebREPL (WebSocket)."""

    def __init__(self, host: str, port: int = 8266, password: str = "bark"):
        super().__init__()
        self._host = host
        self._port = port
        self._password = password
        self._ws = None
        self._ws_lock = asyncio.Lock()

    async def open(self) -> None:
        import websockets

        uri = f"ws://{self._host}:{self._port}/"
        logger.info("Connecting to WebREPL at %s", uri)

        self._ws = await websockets.connect(uri, ping_interval=None)

        # WebREPL handshake
        greeting = await self._ws_recv_until("Password: ", timeout=5)
        logger.debug("WebREPL greeting: %r", greeting)

        await self._ws.send(self._password + "\r\n")
        auth_resp = await self._ws_recv_until(">>>", timeout=5)
        if "Access denied" in auth_resp:
            raise ConnectionError("WebREPL authentication failed")

        self._open = True
        await self._init_repl()
        logger.info("WebReplTransport opened on %s:%d", self._host, self._port)

    async def close(self) -> None:
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
        self._ws = None
        self._open = False
        logger.info("WebReplTransport closed")

    async def _exec(self, cmd: str) -> None:
        async with self._ws_lock:
            if not self._ws:
                return
            await self._ws.send(cmd + "\r\n")
            await self._ws_recv_until(">>>", timeout=2)

    async def _exec_read(self, cmd: str) -> str:
        async with self._ws_lock:
            if not self._ws:
                return ""
            await self._ws.send(cmd + "\r\n")
            raw = await self._ws_recv_until(">>>", timeout=3)

            # Extract output: skip echo and prompt lines
            lines = raw.split("\r\n")
            output = []
            for line in lines:
                line = line.strip()
                if line and not line.startswith(">>>") and line != cmd:
                    output.append(line)
            return "\n".join(output)

    async def _ws_recv_until(self, marker: str, timeout: float = 3) -> str:
        """Read WebSocket messages until marker appears or timeout."""
        buf = ""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                msg = await asyncio.wait_for(
                    self._ws.recv(), timeout=min(remaining, 0.5)
                )
                buf += msg.decode(errors="replace") if isinstance(msg, bytes) else msg
                if marker in buf:
                    break
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.warning("WebREPL recv error: %s", e)
                break
        return buf
