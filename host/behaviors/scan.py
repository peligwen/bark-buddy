"""
Ultrasonic scan behavior — rotate in place taking distance readings.

Performs a 360° sweep by turning in small increments, reading the
ultrasonic sensor at each step. Produces a list of (angle, distance)
pairs that can be converted to 2D points for mapping.
"""

import asyncio
import logging
import math
from dataclasses import dataclass
from typing import Callable, Optional

from comms import DogComms

logger = logging.getLogger(__name__)

# Scan parameters
SCAN_STEP_DEG = 15.0        # degrees per turn step
TURN_SPEED_DPS = 45.0       # estimated turn speed (match patrol)
SETTLE_TIME = 0.3           # seconds to wait after turning before reading
READINGS_PER_STEP = 3       # ultrasonic readings to average per step
READING_DELAY = 0.1         # seconds between readings at same angle
MAX_RANGE_MM = 3000         # ignore readings beyond this (no obstacle)


@dataclass
class ScanPoint:
    angle: float       # degrees from start heading (0 = initial forward)
    distance_mm: int   # ultrasonic distance in mm
    x: float           # meters, relative to scan origin
    y: float           # meters, relative to scan origin


class ScanResult:
    """Result of a completed scan."""

    def __init__(self, origin_x: float, origin_y: float, origin_heading: float):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_heading = origin_heading
        self.points: list[ScanPoint] = []

    def add_point(self, angle: float, distance_mm: int) -> ScanPoint:
        """Add a scan point and compute its x/y position."""
        # Convert to world-relative angle
        world_angle = self.origin_heading + angle
        rad = math.radians(world_angle)
        dist_m = distance_mm / 1000.0
        x = self.origin_x + dist_m * math.cos(rad)
        y = self.origin_y + dist_m * math.sin(rad)
        point = ScanPoint(angle=angle, distance_mm=distance_mm, x=x, y=y)
        self.points.append(point)
        return point

    def to_dict(self) -> dict:
        return {
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "origin_heading": self.origin_heading,
            "points": [
                {"angle": p.angle, "distance_mm": p.distance_mm,
                 "x": round(p.x, 3), "y": round(p.y, 3)}
                for p in self.points
            ],
        }


class ScanBehavior:
    """
    Rotate in place taking ultrasonic readings at each step.

    Usage:
        scan = ScanBehavior(dog)
        result = await scan.execute(origin_x=0, origin_y=0, origin_heading=0)
    """

    def __init__(self, dog: DogComms):
        self._dog = dog
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._on_point: Optional[Callable] = None
        self._on_complete: Optional[Callable] = None

    @property
    def running(self) -> bool:
        return self._running

    def on_point(self, callback: Callable) -> None:
        """Called with (ScanPoint, progress_pct) after each reading."""
        self._on_point = callback

    def on_complete(self, callback: Callable) -> None:
        """Called with (ScanResult) when scan finishes."""
        self._on_complete = callback

    async def execute(self, origin_x: float = 0, origin_y: float = 0,
                      origin_heading: float = 0) -> Optional[ScanResult]:
        """Perform a full 360° scan. Returns ScanResult or None if cancelled."""
        if self._running:
            logger.warning("Scan already running")
            return None

        self._running = True
        result = ScanResult(origin_x, origin_y, origin_heading)
        total_steps = int(360 / SCAN_STEP_DEG)
        current_angle = 0.0

        logger.info("Starting ultrasonic scan at (%.2f, %.2f) heading %.1f",
                     origin_x, origin_y, origin_heading)

        try:
            for step in range(total_steps):
                if not self._running:
                    logger.info("Scan cancelled at step %d/%d", step, total_steps)
                    return None

                # Take readings at current angle
                distance = await self._read_averaged()
                if distance is not None and distance < MAX_RANGE_MM:
                    point = result.add_point(current_angle, distance)
                    progress = round((step + 1) / total_steps * 100)
                    if self._on_point:
                        await self._fire(self._on_point, point, progress)

                # Turn to next step (except after last reading)
                if step < total_steps - 1:
                    await self._turn_step()
                    current_angle += SCAN_STEP_DEG

            # Return to original heading (turn back)
            return_angle = 360.0 - current_angle
            if return_angle > 0 and self._running:
                await self._turn_amount(return_angle)

            await self._dog.stop()

            logger.info("Scan complete: %d points", len(result.points))
            if self._on_complete:
                await self._fire(self._on_complete, result)
            return result

        except asyncio.CancelledError:
            await self._dog.stop()
            return None
        except Exception:
            logger.exception("Scan failed")
            await self._dog.stop()
            return None
        finally:
            self._running = False

    async def cancel(self) -> None:
        """Cancel a running scan and wait for it to finish."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        await self._dog.stop()

    async def _read_averaged(self) -> Optional[int]:
        """Take multiple readings and return the median."""
        readings = []
        for _ in range(READINGS_PER_STEP):
            dist = await self._dog.read_ultrasonic()
            if dist is not None:
                readings.append(dist)
            await asyncio.sleep(READING_DELAY)

        if not readings:
            return None
        readings.sort()
        return readings[len(readings) // 2]  # median

    async def _turn_step(self) -> None:
        """Turn one scan step (SCAN_STEP_DEG degrees to the right)."""
        await self._turn_amount(SCAN_STEP_DEG)

    async def _turn_amount(self, degrees: float) -> None:
        """Turn right by the given degrees (positive heading direction)."""
        await self._dog.turn_right()
        duration = degrees / TURN_SPEED_DPS
        await asyncio.sleep(duration)
        await self._dog.stop()
        await asyncio.sleep(SETTLE_TIME)

    @staticmethod
    async def _fire(callback: Callable, *args) -> None:
        result = callback(*args)
        if asyncio.iscoroutine(result):
            await result
