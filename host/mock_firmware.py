"""
Mock firmware transport — simulates the custom C++ firmware's behavior.

Pushes telemetry at firmware rates (50Hz IMU, 20Hz sonar) instead of
the stock CMD protocol's request-response pattern. Uses the same
mock room geometry and raycast logic as mock_serial.py.
"""

import asyncio
import json
import logging
import math
import random
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT

logger = logging.getLogger(__name__)

DEFAULT_NOISE = {
    "packet_drop_pct": 0.0,
    "latency_base_ms": 0.0,
    "latency_jitter_ms": 0.0,
    "sonar_noise_mm": 0.0,
    "sonar_outlier_pct": 0.0,
    "imu_drift_dps": 0.0,
}


class MockFirmwareTransport(Transport):
    """
    Simulates the custom firmware with high-frequency push telemetry.

    - IMU at 50Hz (pitch/roll/yaw + accel/gyro)
    - Sonar at 20Hz
    - Battery at 1Hz
    - Same mock room geometry as MockTransport
    - Responds to JSON commands (cmd_move, etc.) translated from CMD protocol
    """

    FORWARD_SPEED = 0.10
    TURN_SPEED = 45.0

    def __init__(self):
        self._open = False
        self._start_time = 0.0
        self._last_motion_time = 0.0
        self._motion_cmd = 1
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._yaw_integrated = 0.0
        self._noise = dict(DEFAULT_NOISE)
        self._imu_drift_accum = 0.0
        self._last_imu_time = 0.0

        # Telemetry cache (updated by _telem_loop)
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm = 0
        self._battery_mv = 7400
        self._last_response: Optional[str] = None
        self._telem_task: Optional[asyncio.Task] = None

        # Mock room geometry (same triangle as MockTransport)
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
        for key in DEFAULT_NOISE:
            if key in params:
                self._noise[key] = float(params[key])

    def get_noise_params(self) -> dict:
        return dict(self._noise)

    async def open(self) -> None:
        self._open = True
        self._start_time = time.monotonic()
        self._last_imu_time = self._start_time
        self._telem_task = asyncio.create_task(self._telem_loop())
        logger.info("MockFirmwareTransport opened (sim+)")

    async def close(self) -> None:
        if self._telem_task:
            self._telem_task.cancel()
            try:
                await self._telem_task
            except asyncio.CancelledError:
                pass
        self._open = False
        logger.info("MockFirmwareTransport closed")

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("Mock firmware not open")

        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return

        inner = data[4:-2]
        parts = inner.split("|")
        if not parts:
            return

        func = parts[0]
        response = self._handle_cmd(func, parts[1:])
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError("Mock firmware not open")
        resp = self._last_response
        self._last_response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._yaw_integrated = 0.0
        self._motion_cmd = 1
        self._last_motion_time = time.monotonic()
        self._imu_drift_accum = 0.0

    def get_position(self) -> tuple[float, float, float]:
        self._step_dead_reckoning()
        return (self._x, self._y, 0.0)

    def get_heading(self) -> float:
        self._step_dead_reckoning()
        return self._heading

    # --- CMD handling (returns CMD-formatted responses for DogComms compat) ---

    def _handle_cmd(self, func: str, args: list[str]) -> Optional[str]:
        if func == "1":
            return "CMD|1|OK|$"
        elif func == "2":
            return "CMD|2|OK|$"
        elif func == "3":
            sub = int(args[0]) if args else 1
            self._step_dead_reckoning()
            self._motion_cmd = sub
            self._last_motion_time = time.monotonic()
            return "CMD|3|OK|$"
        elif func == "4":
            return f"CMD|4|{self._sonar_mm}|$"
        elif func == "5":
            return f"CMD|5|{self._imu['pitch']}|{self._imu['roll']}|$"
        elif func == "6":
            return f"CMD|6|{self._battery_mv}|$"
        return None

    # --- Background telemetry loop (simulates firmware push) ---

    async def _telem_loop(self):
        """Push telemetry at firmware rates."""
        imu_interval = 1.0 / 50   # 50Hz
        sonar_interval = 1.0 / 20  # 20Hz
        last_sonar = 0.0

        try:
            while self._open:
                now = time.monotonic()

                # IMU update (50Hz)
                self._update_imu(now)

                # Sonar update (20Hz)
                if now - last_sonar >= sonar_interval:
                    self._update_sonar()
                    last_sonar = now

                await asyncio.sleep(imu_interval)
        except asyncio.CancelledError:
            pass

    def _update_imu(self, now: float) -> None:
        elapsed = now - self._start_time
        dt = now - self._last_imu_time
        self._last_imu_time = now
        if dt <= 0 or dt > 0.5:
            return

        pitch = 1.5 * math.sin(elapsed * 0.5)
        roll = 0.8 * math.cos(elapsed * 0.3)

        # Gyro simulation (deg/s)
        gz = 0.0
        if self._motion_cmd == 5:
            gz = -self.TURN_SPEED
        elif self._motion_cmd == 6:
            gz = self.TURN_SPEED

        # IMU drift
        if self._noise["imu_drift_dps"] > 0 and dt > 0:
            self._imu_drift_accum += self._noise["imu_drift_dps"] * dt
            pitch += self._imu_drift_accum

        # Yaw integration
        self._yaw_integrated += gz * dt

        self._imu = {
            "pitch": round(pitch, 1),
            "roll": round(roll, 1),
            "yaw": round(self._yaw_integrated, 1),
        }

    def _update_sonar(self) -> None:
        dist_mm = self._raycast_ultrasonic()

        if self._noise["sonar_noise_mm"] > 0:
            dist_mm += random.gauss(0, self._noise["sonar_noise_mm"])
        if self._noise["sonar_outlier_pct"] > 0:
            if random.random() * 100 < self._noise["sonar_outlier_pct"]:
                dist_mm = random.uniform(50, 3000)

        self._sonar_mm = max(0, int(dist_mm))

    # --- Physics (shared with MockTransport) ---

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

        return int(min_dist * 1000)
