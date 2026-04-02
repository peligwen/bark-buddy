"""
Mock transport for development without MechDog hardware.

Simulates the stock CMD protocol responses so the host and web UI
can be developed and tested independently.
"""

import asyncio
import logging
import math
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)


class MockTransport(Transport):
    """
    Simulated serial connection to a fake MechDog.

    - Acks motion/posture/action commands with CMD|<func>|OK|$
    - Returns fake IMU tilt data (gentle sinusoidal sway)
    - Returns fake battery voltage
    """

    def __init__(self):
        self._open = False
        self._inbox: asyncio.Queue[str] = asyncio.Queue()
        self._start_time = 0.0

    async def open(self) -> None:
        self._open = True
        self._start_time = time.monotonic()
        logger.info("MockTransport opened")

    async def close(self) -> None:
        self._open = False
        logger.info("MockTransport closed")

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("Mock not open")
        response = self._process_cmd(data)
        if response:
            await self._inbox.put(response)

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        try:
            return await asyncio.wait_for(self._inbox.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None

    def is_open(self) -> bool:
        return self._open

    def _process_cmd(self, cmd: str) -> Optional[str]:
        """Parse incoming CMD and generate appropriate response."""
        cmd = cmd.strip()
        if not (cmd.startswith("CMD|") and cmd.endswith("|$")):
            return None

        inner = cmd[4:-2]  # strip CMD| and |$
        parts = inner.split("|")
        if not parts:
            return None

        func = parts[0]

        # Function 1: Posture adjustment
        if func == "1":
            logger.debug("Mock: posture command %s", parts)
            return "CMD|1|OK|$"

        # Function 2: Action groups
        if func == "2":
            logger.debug("Mock: action command %s", parts)
            return "CMD|2|OK|$"

        # Function 3: Motion control
        if func == "3":
            motion_names = {
                "1": "stop", "2": "stand", "3": "forward", "4": "backward",
                "5": "turn_left", "6": "turn_right", "7": "shift_left", "8": "shift_right",
            }
            sub = parts[1] if len(parts) > 1 else "?"
            name = motion_names.get(sub, f"unknown({sub})")
            logger.debug("Mock: motion %s", name)
            return "CMD|3|OK|$"

        # Function 4: Ultrasonic
        if func == "4":
            elapsed = time.monotonic() - self._start_time
            # Simulate distance varying between 100-500mm
            dist = int(300 + 200 * math.sin(elapsed * 0.2))
            return f"CMD|4|{dist}|$"

        # Function 5: IMU data
        if func == "5":
            elapsed = time.monotonic() - self._start_time
            pitch = round(1.5 * math.sin(elapsed * 0.5), 1)
            roll = round(0.8 * math.cos(elapsed * 0.3), 1)
            return f"CMD|5|{pitch}|{roll}|$"

        # Function 6: Battery
        if func == "6":
            return "CMD|6|7400|$"  # 7.4V fake battery

        return None
