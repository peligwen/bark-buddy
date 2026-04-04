"""
Physics simulation transport for MechDog.

Uses the pure-Python physics engine (no external deps). Pushes telemetry
at firmware rates (50Hz IMU, 20Hz sonar, 1Hz battery). Same data flow
as real hardware.

Usage:
    transport = SimTransport()
    dog = DogComms(transport)
    await dog.connect()
"""

import asyncio
import logging
import math
import random
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT
from sim.physics import DogPhysics, FORWARD_SPEED, TURN_SPEED

logger = logging.getLogger(__name__)

SIM_TIMESTEP = 1.0 / 240.0
BATTERY_MV = 7400

DEFAULT_NOISE = {
    "packet_drop_pct": 0.0,
    "latency_base_ms": 0.0,
    "latency_jitter_ms": 0.0,
    "sonar_noise_mm": 0.0,
    "sonar_outlier_pct": 0.0,
    "imu_drift_dps": 0.0,
}


class SimTransport(Transport):
    """
    Physics simulation transport with push telemetry.

    Runs the physics engine in a background task, pushing IMU (50Hz),
    sonar (20Hz), and battery (1Hz) into caches. Same data flow as
    real firmware.
    """

    FORWARD_SPEED = FORWARD_SPEED
    TURN_SPEED = TURN_SPEED

    def __init__(self, speed_factor: float = 1.0):
        self._speed_factor = speed_factor
        self._open = False
        self._physics = DogPhysics()

        # Motion state
        self._motion_cmd = 1  # STOP

        # CMD response
        self._last_response: Optional[str] = None

        # Telemetry cache
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._sonar_mm = 0
        self._battery_mv = BATTERY_MV

        # Noise
        self._noise = dict(DEFAULT_NOISE)
        self._imu_drift_accum = 0.0

        # Background task
        self._sim_task: Optional[asyncio.Task] = None

    # --- Noise ---

    def set_noise_params(self, params: dict) -> None:
        for key in DEFAULT_NOISE:
            if key in params:
                self._noise[key] = float(params[key])

    def get_noise_params(self) -> dict:
        return dict(self._noise)

    # --- Transport interface ---

    async def open(self) -> None:
        if self._open:
            return

        # Default room (triangle with partition)
        self._physics.add_wall((1.5, 0.0), (-0.75, 1.3))
        self._physics.add_wall((-0.75, 1.3), (-0.75, -1.3))
        self._physics.add_wall((-0.75, -1.3), (1.5, 0.0))
        self._physics.add_wall((-0.3, 0.5), (-0.3, -0.5))

        # Let robot settle
        for _ in range(480):
            self._physics.step(SIM_TIMESTEP)

        self._open = True
        self._sim_task = asyncio.create_task(self._sim_loop())
        logger.info("SimTransport opened (physics engine, speed=%.1fx)",
                     self._speed_factor)

    async def close(self) -> None:
        if self._sim_task:
            self._sim_task.cancel()
            try:
                await self._sim_task
            except asyncio.CancelledError:
                pass
        self._open = False
        logger.info("SimTransport closed")

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("SimTransport not open")
        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return
        response = self._process_cmd(data)
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError("SimTransport not open")
        resp = self._last_response
        self._last_response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    # --- Position / heading ---

    def reset(self) -> None:
        self.reset_pose()

    def reset_pose(self, position: list = None, yaw: float = 0.0) -> None:
        x = position[0] if position else 0.0
        z = position[2] if position and len(position) > 2 else 0.0
        self._physics.reset(x=x, z=z, yaw=-math.radians(yaw))
        for _ in range(240):
            self._physics.step(SIM_TIMESTEP)

    def get_position(self) -> tuple[float, float, float]:
        p = self._physics.pos
        return (p.x, p.z, p.y)  # Y-up to ground plane: (forward, left, height)

    def get_heading(self) -> float:
        _, _, yaw = self._physics.orient.to_euler()
        return -math.degrees(yaw)  # negate to match position Z-flip

    def get_joint_states(self) -> dict:
        return self._physics.get_joint_states()

    # --- Room setup ---

    def add_wall(self, start: tuple, end: tuple, height: float = 0.3,
                 thickness: float = 0.02) -> None:
        self._physics.add_wall(start, end, height, thickness)

    def add_box_room(self, width: float = 2.0, depth: float = 2.0,
                     height: float = 0.3) -> None:
        hw, hd = width / 2, depth / 2
        self._physics.add_wall((-hw, -hd), (hw, -hd), height)
        self._physics.add_wall((hw, -hd), (hw, hd), height)
        self._physics.add_wall((hw, hd), (-hw, hd), height)
        self._physics.add_wall((-hw, hd), (-hw, -hd), height)

    def clear_walls(self) -> None:
        self._physics.clear_walls()

    # --- Background sim + telemetry loop ---

    async def _sim_loop(self):
        """Step physics and push telemetry at firmware rates."""
        imu_interval = 1.0 / 50
        sonar_interval = 1.0 / 20
        last_sonar = 0.0
        steps_per_tick = max(1, int(imu_interval * self._speed_factor / SIM_TIMESTEP))

        try:
            while self._open:
                # Step physics
                for _ in range(steps_per_tick):
                    self._physics.step(SIM_TIMESTEP)
                    self._physics.apply_movement_force(self._motion_cmd, SIM_TIMESTEP)

                # Update telemetry caches
                self._update_imu()

                now = time.monotonic()
                if now - last_sonar >= sonar_interval / self._speed_factor:
                    self._update_sonar()
                    last_sonar = now

                await asyncio.sleep(imu_interval / self._speed_factor)
        except asyncio.CancelledError:
            pass

    def _update_imu(self) -> None:
        pitch, roll, yaw = self._physics.read_imu()

        if self._noise["imu_drift_dps"] > 0:
            self._imu_drift_accum += self._noise["imu_drift_dps"] / 50.0
            pitch += self._imu_drift_accum

        self._imu = {
            "pitch": round(pitch, 1),
            "roll": round(roll, 1),
            "yaw": round(yaw, 1),
        }

    def _update_sonar(self) -> None:
        dist_mm = self._physics.raycast_ultrasonic()

        if self._noise["sonar_noise_mm"] > 0:
            dist_mm += random.gauss(0, self._noise["sonar_noise_mm"])
        if self._noise["sonar_outlier_pct"] > 0:
            if random.random() * 100 < self._noise["sonar_outlier_pct"]:
                dist_mm = random.uniform(50, 3000)

        self._sonar_mm = max(0, int(dist_mm))

    # --- CMD processing ---

    def _process_cmd(self, data: str) -> Optional[str]:
        data = data.strip()
        if not data.startswith("CMD|") or not data.endswith("|$"):
            return None

        parts = data[4:-2].split("|")
        func = parts[0]

        if func == "1":
            return "CMD|1|OK|$"

        elif func == "2":
            return "CMD|2|OK|$"

        elif func == "3":
            self._motion_cmd = int(parts[1]) if len(parts) > 1 else 1
            self._physics.set_motion(self._motion_cmd)
            return "CMD|3|OK|$"

        elif func == "4":
            return f"CMD|4|{self._sonar_mm}|$"

        elif func == "5":
            return f"CMD|5|{self._imu['pitch']}|{self._imu['roll']}|$"

        elif func == "6":
            return f"CMD|6|{self._battery_mv}|$"

        return None

    @property
    def sim_time(self) -> float:
        return self._physics.time
