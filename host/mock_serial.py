"""
Mock serial transport for development without MechDog hardware.

Echoes commands with acks and generates fake IMU telemetry so the host
and web UI can be developed and tested independently.
"""

import asyncio
import json
import logging
import math
import time

from comms import Transport

logger = logging.getLogger(__name__)


class MockTransport(Transport):
    """
    Simulated serial connection to a fake MechDog.

    - Responds to pings with pongs
    - Acks all known commands
    - Streams fake IMU telemetry at ~20 Hz
    - Streams fake status at ~1 Hz
    """

    def __init__(self):
        self._open = False
        self._inbox: asyncio.Queue[str] = asyncio.Queue()
        self._outbox: asyncio.Queue[str] = asyncio.Queue()
        self._task: asyncio.Task | None = None

    async def open(self) -> None:
        self._open = True
        self._task = asyncio.create_task(self._simulate())
        logger.info("MockTransport opened")

    async def close(self) -> None:
        self._open = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        logger.info("MockTransport closed")

    async def readline(self) -> str:
        return await self._inbox.get()

    async def writeline(self, line: str) -> None:
        await self._outbox.put(line)

    def is_open(self) -> bool:
        return self._open

    async def _simulate(self) -> None:
        """Run the fake firmware loop."""
        imu_interval = 0.05   # 20 Hz
        status_interval = 1.0  # 1 Hz
        last_imu = 0.0
        last_status = 0.0
        start = time.monotonic()

        while self._open:
            now = time.monotonic()
            elapsed = now - start

            # Process any incoming commands
            try:
                line = self._outbox.get_nowait()
                await self._handle_command(line)
            except asyncio.QueueEmpty:
                pass

            # Stream fake IMU telemetry
            if now - last_imu >= imu_interval:
                await self._send_imu(elapsed)
                last_imu = now

            # Stream fake status
            if now - last_status >= status_interval:
                await self._send_status()
                last_status = now

            await asyncio.sleep(0.01)  # 100 Hz tick

    async def _handle_command(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return

        msg_type = msg.get("type")
        if not msg_type:
            return

        if msg_type == "ping":
            await self._respond({"type": "pong"})
        elif msg_type.startswith("cmd_"):
            await self._respond({
                "type": "ack",
                "ref_type": msg_type,
                "ok": True,
            })
            logger.debug("Mock acked: %s", msg_type)
        else:
            await self._respond({
                "type": "ack",
                "ref_type": msg_type,
                "ok": False,
                "error": "unknown_type",
            })

    async def _send_imu(self, elapsed: float) -> None:
        # Gentle sinusoidal sway to simulate a standing dog
        await self._respond({
            "type": "telem_imu",
            "pitch": round(1.5 * math.sin(elapsed * 0.5), 2),
            "roll": round(0.8 * math.cos(elapsed * 0.3), 2),
            "yaw": round((elapsed * 2.0) % 360, 1),
            "ax": round(0.01 * math.sin(elapsed), 4),
            "ay": round(-0.02 * math.cos(elapsed), 4),
            "az": 9.81,
            "gx": 0.0,
            "gy": 0.0,
            "gz": 0.0,
        })

    async def _send_status(self) -> None:
        await self._respond({
            "type": "telem_status",
            "mode": "idle",
            "battery_pct": 85,
            "servo_errors": [],
        })

    async def _respond(self, msg: dict) -> None:
        await self._inbox.put(json.dumps(msg, separators=(",", ":")))
