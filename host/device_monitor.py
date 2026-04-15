"""USB serial device hot-plug monitor."""
import asyncio
import logging
from collections.abc import Awaitable
from typing import Callable, Optional
import serial.tools.list_ports

logger = logging.getLogger(__name__)

PORT_PATTERNS = ("/dev/cu.usbserial", "/dev/ttyUSB", "COM")


class DeviceMonitor:
    """Polls serial ports every interval_s seconds, fires callbacks on change."""

    def __init__(self,
                 on_added: Callable[[str], Awaitable[None]],
                 on_removed: Callable[[str], Awaitable[None]],
                 interval_s: float = 2.0):
        self._on_added = on_added
        self._on_removed = on_removed
        self._interval_s = interval_s
        self._known_ports: set[str] = set()
        self._task: Optional[asyncio.Task] = None

    def _matching_ports(self) -> set[str]:
        return {
            info.device
            for info in serial.tools.list_ports.comports()
            if any(info.device.startswith(p) for p in PORT_PATTERNS)
        }

    async def start(self):
        """Start monitoring. Captures the current port set as baseline."""
        self._known_ports = await asyncio.get_running_loop().run_in_executor(None, self._matching_ports)
        self._task = asyncio.create_task(self._poll_loop())

    async def stop(self):
        """Stop monitoring."""
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _poll_loop(self):
        while True:
            await asyncio.sleep(self._interval_s)
            current = await asyncio.get_running_loop().run_in_executor(None, self._matching_ports)
            added = current - self._known_ports
            removed = self._known_ports - current
            self._known_ports = current
            for port in sorted(added):
                logger.info("USB device added: %s", port)
                try:
                    await self._on_added(port)
                except Exception:
                    logger.exception("on_added callback failed for %s", port)
            for port in sorted(removed):
                logger.info("USB device removed: %s", port)
                try:
                    await self._on_removed(port)
                except Exception:
                    logger.exception("on_removed callback failed for %s", port)
