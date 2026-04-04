"""Dead reckoning mixin for transports that estimate position from motion commands."""

import math
import time


class DeadReckoningMixin:
    """
    Mixin providing dead reckoning position/heading estimation.

    Tracks x/y position and heading based on motion commands.
    Inherit alongside Transport and call _init_dead_reckoning() in __init__.
    """

    FORWARD_SPEED = 0.10  # m/s
    TURN_SPEED = 45.0     # deg/s

    def _init_dead_reckoning(self) -> None:
        self._heading = 0.0
        self._x = 0.0
        self._y = 0.0
        self._motion_cmd = 1  # STOP
        self._last_motion_time = 0.0

    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._motion_cmd = 1
        self._last_motion_time = time.monotonic()

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
