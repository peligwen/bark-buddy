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
    - Provides mock room walls for 3D visualization
    """

    # Dead reckoning speeds (matching patrol.py estimates)
    FORWARD_SPEED = 0.10   # m/s
    TURN_SPEED = 45.0      # deg/s

    def __init__(self):
        self._open = False
        self._inbox: asyncio.Queue[str] = asyncio.Queue()
        self._start_time = 0.0
        self._last_motion_time = 0.0
        self._motion_cmd = 1  # STOP
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0  # degrees
        # Mock room: triangle with interior partition
        # Vertices: A=(1.5, 0), B=(-0.75, 1.3), C=(-0.75, -1.3)
        # Dog starts at origin facing +X (toward vertex A)
        ax, ay = 1.5, 0.0
        bx, by = -0.75, 1.3
        cx, cy = -0.75, -1.3
        def _wall(x1, y1, x2, y2):
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            dx, dy = x2 - x1, y2 - y1
            length = math.sqrt(dx * dx + dy * dy)
            angle = math.atan2(dy, dx)
            return {"cx": mx, "cy": my, "length": length,
                    "thickness": 0.02, "height": 0.3, "angle": angle}
        self._wall_geom = [
            _wall(ax, ay, bx, by),   # A→B (front-left)
            _wall(bx, by, cx, cy),   # B→C (back wall)
            _wall(cx, cy, ax, ay),   # C→A (front-right)
            _wall(-0.3, 0.5, -0.3, -0.5),  # interior partition
        ]

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
        if not self._open:
            raise ConnectionError("Mock not open")
        try:
            return await asyncio.wait_for(self._inbox.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None

    def is_open(self) -> bool:
        return self._open

    def reset(self) -> None:
        """Reset dead reckoning to origin."""
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._motion_cmd = 1
        self._last_motion_time = time.monotonic()

    def get_position(self) -> tuple[float, float, float]:
        """Get dead-reckoned position (x, y, z)."""
        self._step_dead_reckoning()
        return (self._x, self._y, 0.0)

    def get_heading(self) -> float:
        """Get current heading in degrees."""
        self._step_dead_reckoning()
        return self._heading

    def _step_dead_reckoning(self) -> None:
        """Update position based on current motion and elapsed time."""
        now = time.monotonic()
        dt = now - self._last_motion_time if self._last_motion_time > 0 else 0
        if dt <= 0 or dt > 1.0:
            self._last_motion_time = now
            return
        self._last_motion_time = now

        cmd = self._motion_cmd
        if cmd == 3:  # forward
            rad = math.radians(self._heading)
            self._x += self.FORWARD_SPEED * dt * math.cos(rad)
            self._y += self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 4:  # backward
            rad = math.radians(self._heading)
            self._x -= self.FORWARD_SPEED * dt * math.cos(rad)
            self._y -= self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 5:  # turn left
            self._heading -= self.TURN_SPEED * dt
        elif cmd == 6:  # turn right
            self._heading += self.TURN_SPEED * dt

    def _raycast_ultrasonic(self) -> int:
        """Cast a ray from dog position along heading, return distance to nearest wall in mm."""
        self._step_dead_reckoning()
        ox, oy = self._x, self._y
        rad = math.radians(self._heading)
        dx, dy = math.cos(rad), math.sin(rad)
        max_range = 3.0  # meters

        min_dist = max_range
        for w in self._wall_geom:
            # Wall segment: center (cx, cy), length, angle, thickness
            # Compute wall endpoints
            wcos = math.cos(w["angle"])
            wsin = math.sin(w["angle"])
            half_l = w["length"] / 2
            ax = w["cx"] - wcos * half_l
            ay = w["cy"] - wsin * half_l
            bx = w["cx"] + wcos * half_l
            by = w["cy"] + wsin * half_l

            # Ray-segment intersection
            # Ray: P = O + t * D, Segment: Q = A + s * (B - A)
            sx, sy = bx - ax, by - ay
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-9:
                continue  # parallel
            t = ((ax - ox) * sy - (ay - oy) * sx) / denom
            s = ((ax - ox) * dy - (ay - oy) * dx) / denom
            if t > 0 and 0 <= s <= 1 and t < min_dist:
                min_dist = t

        return int(min_dist * 1000)

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
            sub = parts[1] if len(parts) > 1 else "1"
            self._step_dead_reckoning()
            self._motion_cmd = int(sub)
            self._last_motion_time = time.monotonic()
            return "CMD|3|OK|$"

        # Function 4: Ultrasonic — raycast against mock walls
        if func == "4":
            dist = self._raycast_ultrasonic()
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
