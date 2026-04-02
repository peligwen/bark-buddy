"""
USB serial REPL transport for MechDog.

Sends Python commands to the MicroPython REPL over USB serial.
Command translation is handled by the HardwareTransport base class.
"""

import asyncio
import logging
from typing import Optional

from hw_transport import HardwareTransport

logger = logging.getLogger(__name__)


class ReplTransport(HardwareTransport):
    """MechDog transport over USB serial REPL."""

    def __init__(self, port: str, baudrate: int = 115200):
        super().__init__()
        self._port = port
        self._baudrate = baudrate
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None

    async def open(self) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        self._open = True

        # Get a clean REPL prompt
        await asyncio.sleep(1)
        self._writer.write(b'\x03\x03\r\n')
        await self._writer.drain()
        await asyncio.sleep(0.5)
        await self._drain_input()

        await self._init_repl()
        logger.info("ReplTransport opened on %s", self._port)

    async def close(self) -> None:
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._reader = None
        self._writer = None
        self._open = False
        logger.info("ReplTransport closed")

    async def _exec(self, cmd: str) -> None:
        if not self._writer:
            return
        self._writer.write((cmd + "\r\n").encode())
        await self._writer.drain()
        await asyncio.sleep(0.15)
        await self._drain_input()

    async def _exec_read(self, cmd: str) -> str:
        if not self._writer or not self._reader:
            return ""
        await self._drain_input()
        self._writer.write((cmd + "\r\n").encode())
        await self._writer.drain()
        await asyncio.sleep(0.5)

        data = b""
        try:
            while self._reader and not self._reader.at_eof():
                chunk = await asyncio.wait_for(
                    self._reader.read(1024), timeout=0.3
                )
                data += chunk
                if b">>>" in data:
                    break
        except asyncio.TimeoutError:
            pass

        return self._parse_repl_output(data.decode(errors="replace"), cmd)

    async def _drain_input(self) -> None:
        if not self._reader:
            return
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.1)
        except asyncio.TimeoutError:
            pass

    @staticmethod
    def _parse_repl_output(text: str, cmd: str) -> str:
        """Extract printed output from REPL response, stripping echo and prompt."""
        lines = text.split("\r\n")
        output = []
        for line in lines:
            line = line.strip()
            if line and not line.startswith(">>>") and line != cmd:
                output.append(line)
        return "\n".join(output)
