"""
Mock transport for development without MechDog hardware.

Simulates the stock CMD protocol responses so the host and web UI
can be developed and tested independently. Supports configurable
noise injection for realistic sensor simulation.
"""

import asyncio
import logging
import math
import random
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)

# Default noise configuration (all off)
DEFAULT_NOISE = {
    "packet_drop_pct": 0.0,
    "latency_base_ms": 0.0,
    "latency_jitter_ms": 0.0,
    "sonar_noise_mm": 0.0,
    "sonar_outlier_pct": 0.0,
    "imu_drift_dps": 0.0,
}


class MockTransport(Transport):
    """
    Simulated serial connection to a fake MechDog.

    - Acks motion/posture/action commands with CMD|<func>|OK|$
    - Returns fake IMU tilt data (gentle sinusoidal sway)
    - Returns fake battery voltage
    - Raycasts against mock room walls for ultrasonic
    - Supports configurable noise injection via set_noise_params()
    """

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
        self._heading = 0.0
        self._noise = dict(DEFAULT_NOISE)
        self._imu_drift_accum = 0.0
        self._last_imu_time = 0.0

        # Mock room: triangle with interior partition
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
            _wall(ax, ay, bx, by),
            _wall(bx, by, cx, cy),
            _wall(cx, cy, ax, ay),
            _wall(-0.3, 0.5, -0.3, -0.5),
        ]

    def set_noise_params(self, params: dict) -> None:
        """Update noise configuration. Only provided keys are changed."""
        for key in DEFAULT_NOISE:
            if key in params:
                self._noise[key] = float(params[key])

    def get_noise_params(self) -> dict:
        return dict(self._noise)

    async def open(self) -> None:
        self._open = True
        self._start_time = time.monotonic()
        self._last_imu_time = self._start_time
        logger.info("MockTransport opened")

    async def close(self) -> None:
        self._open = False
        logger.info("MockTransport closed")

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("Mock not open")

        # Packet drop
        if self._noise["packet_drop_pct"] > 0:
            if random.random() * 100 < self._noise["packet_drop_pct"]:
                return  # silently drop

        response = self._process_cmd(data)
        if response:
            # Latency injection
            latency_ms = self._noise["latency_base_ms"]
            jitter = self._noise["latency_jitter_ms"]
            if latency_ms > 0 or jitter > 0:
                delay = (latency_ms + random.uniform(0, jitter)) / 1000.0
                await asyncio.sleep(delay)
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
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._motion_cmd = 1
        self._last_motion_time = time.monotonic()
        self._imu_drift_accum = 0.0

    def get_position(self) -> tuple[float, float, float]:
        self._step_dead_reckoning()
        return (self._x, self._y, 0.0)

    def get_heading(self) -> float:
        self._step_dead_reckoning()
        return self._heading

    def _step_dead_reckoning(self) -> None:
        now = time.monotonic()
        dt = now - self._last_motion_time if self._last_motion_time > 0 else 0
        if dt <= 0 or dt > 1.0:
            self._last_motion_time = now
            return
        self._last_motion_time = now

        cmd = self._motion_cmd
        if cmd == 3:
            rad = math.radians(self._heading)
            self._x += self.FORWARD_SPEED * dt * math.cos(rad)
            self._y += self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 4:
            rad = math.radians(self._heading)
            self._x -= self.FORWARD_SPEED * dt * math.cos(rad)
            self._y -= self.FORWARD_SPEED * dt * math.sin(rad)
        elif cmd == 5:
            self._heading -= self.TURN_SPEED * dt
        elif cmd == 6:
            self._heading += self.TURN_SPEED * dt

    def _raycast_ultrasonic(self) -> int:
        """Cast a ray from dog position along heading, return distance in mm."""
        self._step_dead_reckoning()
        ox, oy = self._x, self._y
        rad = math.radians(self._heading)
        dx, dy = math.cos(rad), math.sin(rad)
        max_range = 3.0

        min_dist = max_range
        for w in self._wall_geom:
            wcos = math.cos(w["angle"])
            wsin = math.sin(w["angle"])
            half_l = w["length"] / 2
            ax = w["cx"] - wcos * half_l
            ay = w["cy"] - wsin * half_l
            bx = w["cx"] + wcos * half_l
            by = w["cy"] + wsin * half_l

            sx, sy = bx - ax, by - ay
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-9:
                continue
            t = ((ax - ox) * sy - (ay - oy) * sx) / denom
            s = ((ax - ox) * dy - (ay - oy) * dx) / denom
            if t > 0 and 0 <= s <= 1 and t < min_dist:
                min_dist = t

        dist_mm = min_dist * 1000

        # Sonar noise: gaussian
        if self._noise["sonar_noise_mm"] > 0:
            dist_mm += random.gauss(0, self._noise["sonar_noise_mm"])

        # Sonar outliers: replace with random garbage
        if self._noise["sonar_outlier_pct"] > 0:
            if random.random() * 100 < self._noise["sonar_outlier_pct"]:
                dist_mm = random.uniform(50, 3000)

        return max(0, int(dist_mm))

    def _process_cmd(self, cmd: str) -> Optional[str]:
        cmd = cmd.strip()
        if not (cmd.startswith("CMD|") and cmd.endswith("|$")):
            return None

        inner = cmd[4:-2]
        parts = inner.split("|")
        if not parts:
            return None

        func = parts[0]

        if func == "1":
            return "CMD|1|OK|$"

        if func == "2":
            return "CMD|2|OK|$"

        if func == "3":
            sub = parts[1] if len(parts) > 1 else "1"
            self._step_dead_reckoning()
            self._motion_cmd = int(sub)
            self._last_motion_time = time.monotonic()
            return "CMD|3|OK|$"

        if func == "4":
            dist = self._raycast_ultrasonic()
            return f"CMD|4|{dist}|$"

        if func == "5":
            now = time.monotonic()
            elapsed = now - self._start_time
            dt = now - self._last_imu_time
            self._last_imu_time = now

            pitch = 1.5 * math.sin(elapsed * 0.5)
            roll = 0.8 * math.cos(elapsed * 0.3)

            # IMU drift accumulation
            if self._noise["imu_drift_dps"] > 0 and dt > 0 and dt < 1.0:
                self._imu_drift_accum += self._noise["imu_drift_dps"] * dt
                pitch += self._imu_drift_accum

            return f"CMD|5|{round(pitch, 1)}|{round(roll, 1)}|$"

        if func == "6":
            return "CMD|6|7400|$"

        return None
