"""
Hybrid transport for MechDog.

Bootstraps the MicroPython NDJSON handler via REPL, then switches to
streaming NDJSON mode. Combines stock firmware's proven servo control
with the structured JSON protocol of the custom firmware.

Bootstrap sequence:
  1. Open serial, get clean REPL prompt (Ctrl+C)
  2. Upload handler.py to the ESP32 filesystem
  3. exec(open('handler.py').read()) — starts the handler
  4. Wait for {"type": "boot"} message
  5. From here on, serial is pure NDJSON (no more REPL)
"""

import asyncio
import json
import logging
import os
import time
from typing import Optional

from json_transport_base import JsonStreamTransport

logger = logging.getLogger(__name__)

# Path to the MicroPython handler script
HANDLER_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "firmware", "hybrid", "handler.py"
)


class HybridTransport(JsonStreamTransport):
    """
    Transport that bootstraps MicroPython NDJSON handler then speaks JSON.

    After bootstrap, behaves identically to FirmwareTransport — streams
    telemetry, caches sensor values, sends JSON commands.
    """

    log_prefix = "HybridTransport"

    def __init__(self, port: str, baudrate: int = 115200):
        super().__init__()
        self._port = port
        self._baudrate = baudrate

    async def open(self) -> None:
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        self._open = True

        # Bootstrap the handler
        await self._bootstrap()

        # Start background reader for streaming telemetry
        self._reader_task = asyncio.create_task(self._reader_loop())
        logger.info("HybridTransport ready on %s", self._port)

    # --- Bootstrap ---

    async def _bootstrap(self) -> None:
        """Start the hybrid handler on the ESP32.

        First tries a DTR reset — if the handler was previously uploaded
        to the filesystem, it auto-runs on boot and sends a boot message.
        Falls back to uploading the handler via REPL if DTR reset doesn't
        produce a boot message.
        """
        # Try DTR reset first (fast path — handler already on filesystem)
        logger.info("Trying DTR reset for existing handler...")
        boot_msg = await self._try_dtr_reset()
        if boot_msg:
            self._firmware_info = boot_msg
            logger.info("Handler already installed, booted via DTR reset: %s", boot_msg)
            return

        # Fall back to REPL upload
        logger.info("No existing handler, uploading via REPL...")
        await self._upload_handler()

        # Wait for boot message after upload
        boot_msg = await self._wait_for_boot(timeout=10.0)
        if not boot_msg:
            raise ConnectionError("Hybrid handler did not send boot message")
        self._firmware_info = boot_msg
        logger.info("Hybrid handler booted after upload: %s", boot_msg)

    async def _try_dtr_reset(self) -> dict | None:
        """Toggle DTR to reset ESP32 and look for boot message."""
        transport = self._writer.transport
        serial_obj = transport.serial if hasattr(transport, 'serial') else None

        if serial_obj and hasattr(serial_obj, 'dtr'):
            serial_obj.dtr = False
            await asyncio.sleep(0.1)
            serial_obj.dtr = True
            await asyncio.sleep(2.5)  # ESP32 boot takes ~2s
        else:
            # No DTR control — just try Ctrl+C and wait
            self._writer.write(b'\x03\x03\r\n')
            await self._writer.drain()
            await asyncio.sleep(1.0)

        return await self._wait_for_boot(timeout=5.0)

    async def _wait_for_boot(self, timeout: float = 10.0) -> dict | None:
        """Wait for a boot JSON message on serial."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = await asyncio.wait_for(self._reader.readline(), timeout=2.0)
                text = line.decode(errors="replace").strip()
                if text.startswith("{"):
                    try:
                        msg = json.loads(text)
                        if msg.get("type") == "boot":
                            return msg
                    except json.JSONDecodeError:
                        pass
            except asyncio.TimeoutError:
                continue
        return None

    async def _upload_handler(self) -> None:
        """Upload handler.py to MicroPython filesystem and exec it."""
        # Get clean REPL prompt
        await asyncio.sleep(1)
        self._writer.write(b'\x03\x03\r\n')
        await self._writer.drain()
        await asyncio.sleep(0.5)
        await self._drain()

        # Read handler source
        with open(HANDLER_PATH, "r") as f:
            handler_src = f.read()

        # Upload handler as a file on the ESP32 filesystem
        # Split into chunks to avoid REPL buffer overflow
        await self._repl_exec("f=open('_hybrid.py','w')")
        chunk_size = 128
        for i in range(0, len(handler_src), chunk_size):
            chunk = handler_src[i:i + chunk_size]
            escaped = chunk.replace("\\", "\\\\").replace("'", "\\'").replace("\n", "\\n").replace("\r", "")
            await self._repl_exec(f"f.write('{escaped}')")
        await self._repl_exec("f.close()")
        logger.info("Handler uploaded (%d bytes)", len(handler_src))

        # Execute the handler — starts the NDJSON loop
        self._writer.write(b"exec(open('_hybrid.py').read())\r\n")
        await self._writer.drain()

    async def _repl_exec(self, cmd: str) -> None:
        """Send a single-line command to REPL and wait for prompt."""
        self._writer.write((cmd + "\r\n").encode())
        await self._writer.drain()
        await asyncio.sleep(0.05)
        await self._drain()

    async def _drain(self) -> None:
        """Drain pending input from the reader."""
        if not self._reader:
            return
        try:
            while True:
                await asyncio.wait_for(self._reader.read(1024), timeout=0.1)
        except asyncio.TimeoutError:
            pass
