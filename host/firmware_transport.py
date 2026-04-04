"""
NDJSON transport for custom MechDog firmware.

Connects to the custom C++ firmware over serial or WiFi TCP.
The firmware pushes telemetry (IMU 50Hz, sonar 20Hz) — no polling needed.
Commands are sent as JSON, responses come as ack messages.
"""

import asyncio
import logging
from typing import Optional

from json_transport_base import JsonStreamTransport

logger = logging.getLogger(__name__)


class FirmwareTransport(JsonStreamTransport):
    """
    Transport for custom firmware speaking NDJSON over serial or TCP.

    Unlike REPL transports, this receives streaming telemetry from the
    firmware. Sensor reads return cached values from the stream.
    """

    log_prefix = "FirmwareTransport"

    def __init__(self, port: str = None, host: str = None, tcp_port: int = 9000):
        super().__init__()
        self._port = port
        self._host = host
        self._tcp_port = tcp_port

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
