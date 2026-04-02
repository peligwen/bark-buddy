"""
Patrol behavior — navigate a sequence of waypoints via dead reckoning.

Uses IMU heading from the stock firmware to estimate orientation,
and timed movement commands for distance. This is rough but functional
on stock hardware without odometry sensors.
"""

import asyncio
import logging
import math
from dataclasses import dataclass
from typing import Callable, Optional

from comms import DogComms

logger = logging.getLogger(__name__)

# Tuning constants (calibrate on real hardware)
FORWARD_SPEED_MPS = 0.10    # estimated forward speed in meters/sec
TURN_SPEED_DPS = 45.0       # estimated turn speed in degrees/sec
WAYPOINT_TOLERANCE_M = 0.05 # close enough to waypoint (meters)
HEADING_TOLERANCE_DEG = 10  # close enough to target heading
POSITION_UPDATE_HZ = 10     # how often we update dead reckoning
MOVE_STEP_DURATION = 0.3    # seconds per move step before re-evaluating


@dataclass
class Waypoint:
    x: float       # meters, local frame
    y: float       # meters, local frame
    heading: float  # degrees, 0 = initial forward direction


class PatrolBehavior:
    """
    Navigate a sequence of waypoints using dead reckoning.

    Position is estimated from IMU heading + timed forward movement.
    Heading comes from polling IMU yaw (or integrating turn commands
    if yaw isn't available from stock firmware).

    Usage:
        patrol = PatrolBehavior(dog)
        patrol.set_waypoints([Waypoint(1, 0, 0), Waypoint(1, 1, 90)])
        await patrol.start()
    """

    def __init__(self, dog: DogComms):
        self._dog = dog
        self._waypoints: list[Waypoint] = []
        self._running = False
        self._task: Optional[asyncio.Task] = None

        # Dead reckoning state
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0  # degrees

        # Callbacks
        self._on_waypoint_reached: Optional[Callable] = None
        self._on_patrol_complete: Optional[Callable] = None
        self._on_position_update: Optional[Callable] = None

    @property
    def running(self) -> bool:
        return self._running

    @property
    def position(self) -> dict:
        return {"x": self._x, "y": self._y, "heading": self._heading}

    @property
    def waypoints(self) -> list[Waypoint]:
        return list(self._waypoints)

    def on_waypoint_reached(self, callback: Callable) -> None:
        self._on_waypoint_reached = callback

    def on_patrol_complete(self, callback: Callable) -> None:
        self._on_patrol_complete = callback

    def on_position_update(self, callback: Callable) -> None:
        self._on_position_update = callback

    def set_waypoints(self, waypoints: list[Waypoint]) -> None:
        self._waypoints = list(waypoints)

    async def start(self) -> None:
        """Start navigating the waypoint sequence."""
        if self._running:
            return
        if not self._waypoints:
            logger.warning("No waypoints set")
            return

        self._running = True
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._task = asyncio.create_task(self._run())
        logger.info("Patrol started with %d waypoints", len(self._waypoints))

    async def stop(self) -> None:
        """Stop patrol and halt the dog."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        await self._dog.stop()
        logger.info("Patrol stopped at (%.2f, %.2f) heading %.1f",
                     self._x, self._y, self._heading)

    async def _run(self) -> None:
        """Main patrol loop: navigate each waypoint in sequence."""
        try:
            for i, wp in enumerate(self._waypoints):
                if not self._running:
                    break
                logger.info("Navigating to waypoint %d: (%.2f, %.2f) heading %.1f",
                            i, wp.x, wp.y, wp.heading)
                await self._navigate_to(wp)
                if self._running and self._on_waypoint_reached:
                    await self._fire(self._on_waypoint_reached, i, wp)

            await self._dog.stop()
            self._running = False
            logger.info("Patrol complete")
            if self._on_patrol_complete:
                await self._fire(self._on_patrol_complete)
        except asyncio.CancelledError:
            pass

    async def _navigate_to(self, wp: Waypoint) -> None:
        """Navigate to a single waypoint: turn toward it, walk, then face target heading."""
        while self._running:
            dx = wp.x - self._x
            dy = wp.y - self._y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < WAYPOINT_TOLERANCE_M:
                break

            # Compute bearing to waypoint
            target_bearing = math.degrees(math.atan2(dy, dx))
            heading_error = self._normalize_angle(target_bearing - self._heading)

            if abs(heading_error) > HEADING_TOLERANCE_DEG:
                # Turn toward waypoint
                await self._turn(heading_error)
            else:
                # Move forward
                await self._move_forward_step()

        # Final heading adjustment
        if self._running:
            final_error = self._normalize_angle(wp.heading - self._heading)
            if abs(final_error) > HEADING_TOLERANCE_DEG:
                await self._turn(final_error)

    async def _turn(self, angle_deg: float) -> None:
        """Turn by approximately angle_deg degrees (positive = left)."""
        if not self._running:
            return

        if angle_deg > 0:
            await self._dog.turn_left()
        else:
            await self._dog.turn_right()

        # Estimate turn duration
        duration = min(abs(angle_deg) / TURN_SPEED_DPS, 2.0)
        await asyncio.sleep(duration)
        await self._dog.stop()

        # Update dead reckoning heading
        self._heading += angle_deg
        self._heading = self._heading % 360
        await self._notify_position()

        # Brief pause for stability
        await asyncio.sleep(0.1)

    async def _move_forward_step(self) -> None:
        """Move forward for one step duration."""
        if not self._running:
            return

        await self._dog.move_forward()
        await asyncio.sleep(MOVE_STEP_DURATION)
        await self._dog.stop()

        # Update dead reckoning position
        rad = math.radians(self._heading)
        dist = FORWARD_SPEED_MPS * MOVE_STEP_DURATION
        self._x += dist * math.cos(rad)
        self._y += dist * math.sin(rad)
        await self._notify_position()

        await asyncio.sleep(0.1)

    async def _notify_position(self) -> None:
        if self._on_position_update:
            await self._fire(self._on_position_update, self.position)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to -180..+180."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    @staticmethod
    async def _fire(callback: Callable, *args) -> None:
        result = callback(*args)
        if asyncio.iscoroutine(result):
            await result
