"""
Ultrasonic scan behavior — rotate in place while continuous mapping captures data.

Performs a 360° sweep by turning in small increments. Unlike the old approach,
this doesn't read the sensor directly — the telemetry loop's 1Hz ultrasonic
pings and continuous mapping handle point capture. The scan just orchestrates
the rotation and tracks progress.
"""

import asyncio
import logging
import math
from dataclasses import dataclass, field
from typing import Callable, Optional

from comms import DogComms

logger = logging.getLogger(__name__)

# Scan parameters
SCAN_STEP_DEG = 15.0        # degrees per turn step
TURN_DURATION = 0.35        # seconds to turn per step (approximate)
SETTLE_TIME = 0.8           # seconds to hold at each angle (let telemetry capture)
TOTAL_STEPS = int(360 / SCAN_STEP_DEG)


@dataclass
class ScanPoint:
    angle: float
    distance_mm: int
    x: float
    y: float
    z: float = 0.09


class ScanResult:
    """Result of a completed scan."""

    def __init__(self, origin_x: float, origin_y: float, origin_heading: float):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_heading = origin_heading
        self.points: list[ScanPoint] = []

    def add_point(self, angle: float, distance_mm: int) -> ScanPoint:
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
    Rotate in place while the telemetry loop captures ultrasonic readings.

    The scan doesn't read the sensor directly — it just turns the dog step
    by step. The server's continuous mapping (1Hz pings → _add_live_point)
    captures points during the settle time at each angle.
    """

    def __init__(self, dog: DogComms):
        self._dog = dog
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._on_point: Optional[Callable] = None
        self._on_complete: Optional[Callable] = None
        self._progress = 0

    @property
    def running(self) -> bool:
        return self._running

    @property
    def progress(self) -> int:
        return self._progress

    def on_point(self, callback: Callable) -> None:
        self._on_point = callback

    def on_complete(self, callback: Callable) -> None:
        self._on_complete = callback

    async def execute(self, origin_x: float = 0, origin_y: float = 0,
                      origin_heading: float = 0) -> Optional[ScanResult]:
        """Perform a full 360° scan. Returns ScanResult or None if cancelled."""
        if self._running:
            logger.warning("Scan already running")
            return None

        self._running = True
        self._progress = 0
        result = ScanResult(origin_x, origin_y, origin_heading)

        logger.info("Starting scan at (%.2f, %.2f) heading %.1f",
                     origin_x, origin_y, origin_heading)

        try:
            for step in range(TOTAL_STEPS):
                if not self._running:
                    logger.info("Scan cancelled at step %d/%d", step, TOTAL_STEPS)
                    return None

                self._progress = round((step + 1) / TOTAL_STEPS * 100)

                # Hold position — telemetry loop captures ultrasonic + adds points
                await asyncio.sleep(SETTLE_TIME)

                # Turn to next step (except after last)
                if step < TOTAL_STEPS - 1:
                    await self._dog.turn_right()
                    await asyncio.sleep(TURN_DURATION)
                    await self._dog.stop()

            # Return to original heading
            await self._dog.stop()

            logger.info("Scan complete")
            self._progress = 100
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
            self._progress = 0

    async def cancel(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        await self._dog.stop()

    @staticmethod
    async def _fire(callback: Callable, *args) -> None:
        result = callback(*args)
        if asyncio.iscoroutine(result):
            await result
