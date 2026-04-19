"""Raw serial/TCP I/O for Dog. Owned by Dog; not public API."""

import asyncio
import logging
from typing import Awaitable, Callable

logger = logging.getLogger(__name__)

SERIAL_BAUD = 115200


class DogIO:
    """
    Manages asyncio streams and background reader/keepalive tasks.
    Call open_serial() or open_tcp(), then start_tasks(on_line_cb, send_ping_cb).
    """

    def __init__(self):
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._open = False
        self._reader_task: asyncio.Task | None = None
        self._keepalive_task: asyncio.Task | None = None

    # --- Connection ---

    async def open_serial(self, port: str, dtr_reset: bool = False) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=port, baudrate=SERIAL_BAUD
        )
        if dtr_reset:
            serial_obj = getattr(self._writer.transport, "serial", None)
            if serial_obj and hasattr(serial_obj, "dtr"):
                serial_obj.dtr = False
                await asyncio.sleep(0.1)
                serial_obj.dtr = True
                await asyncio.sleep(3.0)
                await self._drain()
            else:
                await asyncio.sleep(1)
        else:
            await asyncio.sleep(1)
        self._open = True

    async def open_tcp(self, host: str, port: int) -> None:
        self._reader, self._writer = await asyncio.open_connection(host, port)
        self._open = True

    async def close(self) -> None:
        self._open = False
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

    def is_open(self) -> bool:
        return self._open

    # --- I/O ---

    async def send(self, data: bytes) -> None:
        if self._writer:
            self._writer.write(data)
            await self._writer.drain()

    async def _drain(self) -> None:
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.2)
        except asyncio.TimeoutError:
            pass

    # --- Background tasks ---

    def start_tasks(self, on_line_cb: Callable[[bytes], None], send_ping_cb: Callable[[], Awaitable[None]]) -> None:
        """Start reader and keepalive tasks. Call after open_*()."""
        self._reader_task = asyncio.create_task(self._reader_loop(on_line_cb))
        self._keepalive_task = asyncio.create_task(self._keepalive_loop(send_ping_cb))

    async def _reader_loop(self, on_line_cb) -> None:
        try:
            while self._open and self._reader:
                line = await self._reader.readline()
                if not line:
                    logger.warning("DogIO: EOF — connection closed")
                    self._open = False
                    break
                on_line_cb(line)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.warning("DogIO: reader error: %s", e)
            self._open = False

    async def _keepalive_loop(self, send_ping_cb) -> None:
        try:
            while self._open:
                await asyncio.sleep(2.0)
                if self._open:
                    await send_ping_cb()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.debug("DogIO: keepalive error: %s", e)
