"""
PyBullet-backed transport for the MechDog simulation.

Implements the Transport interface from comms.py, translating CMD protocol
strings into PyBullet physics operations. Supports fast-forward mode for
test-driven development.

Usage:
    transport = SimTransport(speed_factor=10)  # 10x fast-forward
    dog = DogComms(transport)
    await dog.connect()
    await dog.move_forward()
    imu = await dog.read_imu()
"""

import asyncio
import logging
import math
import os
from typing import Optional

import pybullet as p
import pybullet_data

from comms import Transport

logger = logging.getLogger(__name__)

# Simulation constants
SIM_TIMESTEP = 1.0 / 240.0    # PyBullet default timestep
SPAWN_HEIGHT = 0.14            # meters above ground to spawn robot

# Joint index mapping (from URDF load order)
JOINT_FL_HIP = 1
JOINT_FL_KNEE = 2
JOINT_FR_HIP = 4
JOINT_FR_KNEE = 5
JOINT_RL_HIP = 7
JOINT_RL_KNEE = 8
JOINT_RR_HIP = 10
JOINT_RR_KNEE = 11

# Ultrasonic sensor link index
ULTRASONIC_LINK = 0

# Standing pose (radians)
STAND_HIP = 0.3         # slight forward lean for standing
STAND_KNEE = -0.6       # bent knees for standing

# Movement speeds (matching stock firmware estimates from patrol.py)
FORWARD_SPEED = 0.10    # m/s
TURN_SPEED = 45.0       # deg/s

# Ultrasonic cone: 5 rays in a ~30° cone
ULTRASONIC_MAX_RANGE = 3.0   # meters
ULTRASONIC_CONE_HALF = 15.0  # degrees half-angle
ULTRASONIC_NUM_RAYS = 5

# Simulated battery
BATTERY_MV = 7400

# Hip and knee joint indices for convenience
_HIPS = [JOINT_FL_HIP, JOINT_FR_HIP, JOINT_RL_HIP, JOINT_RR_HIP]
_KNEES = [JOINT_FL_KNEE, JOINT_FR_KNEE, JOINT_RL_KNEE, JOINT_RR_KNEE]
_FOOT_LINKS = [3, 6, 9, 12]


class SimTransport(Transport):
    """
    PyBullet physics transport implementing the Transport interface.

    Processes CMD protocol strings by driving a simulated MechDog in
    PyBullet and returning simulated sensor responses.
    """

    def __init__(self, speed_factor: float = 1.0, gui: bool = False):
        """
        Args:
            speed_factor: Simulation speed multiplier. >1.0 runs more
                         physics steps per CMD cycle for faster-than-realtime.
            gui: If True, open PyBullet GUI window for visual debugging.
        """
        self._speed_factor = speed_factor
        self._gui = gui
        self._open = False
        self._client: Optional[int] = None
        self._robot: Optional[int] = None
        self._plane: Optional[int] = None
        self._walls: list[int] = []

        # Motion state
        self._motion_cmd = 1  # MOTION_STOP
        self._balance_enabled = False
        self._sim_time = 0.0

        # Response queue
        self._response: Optional[str] = None

    async def open(self) -> None:
        if self._open:
            return

        mode = p.GUI if self._gui else p.DIRECT
        self._client = p.connect(mode)
        c = self._client

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=c)
        p.setGravity(0, 0, -9.81, physicsClientId=c)
        p.setTimeStep(SIM_TIMESTEP, physicsClientId=c)

        # Ground plane
        self._plane = p.loadURDF("plane.urdf", physicsClientId=c)
        p.changeDynamics(self._plane, -1, lateralFriction=1.0, physicsClientId=c)

        # Load robot
        urdf_path = os.path.join(os.path.dirname(__file__), "mechdog.urdf")
        self._robot = p.loadURDF(
            urdf_path, [0, 0, SPAWN_HEIGHT], useFixedBase=False,
            physicsClientId=c,
        )

        # Set foot friction
        for link_idx in _FOOT_LINKS:
            p.changeDynamics(self._robot, link_idx,
                             lateralFriction=1.0, rollingFriction=0.01,
                             physicsClientId=c)

        # Initialize standing pose — reset joint states first for stability
        for hip in _HIPS:
            p.resetJointState(self._robot, hip, STAND_HIP, physicsClientId=c)
        for knee in _KNEES:
            p.resetJointState(self._robot, knee, STAND_KNEE, physicsClientId=c)
        self._set_standing_pose()

        # Let robot settle
        for _ in range(480):
            p.stepSimulation(physicsClientId=c)
        self._sim_time = 480 * SIM_TIMESTEP

        self._open = True
        logger.info("SimTransport opened (speed_factor=%.1f)", self._speed_factor)

    async def close(self) -> None:
        if self._client is not None:
            p.disconnect(self._client)
            self._client = None
        self._open = False
        self._robot = None
        logger.info("SimTransport closed")

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("SimTransport not open")
        self._response = self._process_cmd(data)

    async def recv(self, timeout: float = 0.5) -> Optional[str]:
        if not self._open:
            raise ConnectionError("SimTransport not open")
        resp = self._response
        self._response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    # --- Room setup for mapping tests ---

    def add_wall(self, start: tuple, end: tuple, height: float = 0.3,
                 thickness: float = 0.02) -> int:
        """Add a wall segment to the simulation for ultrasonic mapping tests."""
        c = self._client
        sx, sy = start
        ex, ey = end
        cx, cy = (sx + ex) / 2, (sy + ey) / 2
        dx, dy = ex - sx, ey - sy
        length = math.sqrt(dx * dx + dy * dy)
        angle = math.atan2(dy, dx)

        col = p.createCollisionShape(p.GEOM_BOX,
                                     halfExtents=[length / 2, thickness / 2, height / 2],
                                     physicsClientId=c)
        vis = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[length / 2, thickness / 2, height / 2],
                                  rgbaColor=[0.5, 0.5, 0.5, 1],
                                  physicsClientId=c)
        quat = p.getQuaternionFromEuler([0, 0, angle])
        body = p.createMultiBody(0, col, vis, [cx, cy, height / 2], quat,
                                 physicsClientId=c)
        self._walls.append(body)
        return body

    def add_box_room(self, width: float = 2.0, depth: float = 2.0,
                     height: float = 0.3) -> None:
        """Add a rectangular room around the origin."""
        hw, hd = width / 2, depth / 2
        self.add_wall((-hw, -hd), (hw, -hd), height)  # front
        self.add_wall((hw, -hd), (hw, hd), height)     # right
        self.add_wall((hw, hd), (-hw, hd), height)     # back
        self.add_wall((-hw, hd), (-hw, -hd), height)   # left

    def clear_walls(self) -> None:
        """Remove all walls from the simulation."""
        c = self._client
        for body in self._walls:
            p.removeBody(body, physicsClientId=c)
        self._walls.clear()

    # --- Pose reset utility (for tests) ---

    def reset_pose(self, position: list = None, yaw: float = 0.0) -> None:
        """Reset robot to a given position and heading with standing pose.

        Args:
            position: [x, y, z] or None for [0, 0, SPAWN_HEIGHT]
            yaw: heading in degrees
        """
        c = self._client
        if position is None:
            position = [0, 0, SPAWN_HEIGHT]
        orn = p.getQuaternionFromEuler([0, 0, math.radians(yaw)])
        p.resetBasePositionAndOrientation(self._robot, position, orn,
                                          physicsClientId=c)
        for hip in _HIPS:
            p.resetJointState(self._robot, hip, STAND_HIP, physicsClientId=c)
        for knee in _KNEES:
            p.resetJointState(self._robot, knee, STAND_KNEE, physicsClientId=c)
        self._set_standing_pose()
        for _ in range(240):
            p.stepSimulation(physicsClientId=c)

    # --- CMD processing ---

    def _process_cmd(self, data: str) -> str:
        """Process a CMD string and return a response."""
        data = data.strip()
        if not data.startswith("CMD|") or not data.endswith("|$"):
            return "CMD|ERR|$"

        parts = data[4:-2].split("|")
        func = int(parts[0])

        if func == 1:
            sub = int(parts[1]) if len(parts) > 1 else 0
            if sub == 3:
                self._balance_enabled = bool(int(parts[2])) if len(parts) > 2 else False
            return "CMD|1|OK|$"

        elif func == 2:
            action_code = int(parts[2]) if len(parts) > 2 else 1
            self._execute_action(action_code)
            return "CMD|2|OK|$"

        elif func == 3:
            motion_code = int(parts[1]) if len(parts) > 1 else 1
            self._motion_cmd = motion_code
            self._step_motion()
            return "CMD|3|OK|$"

        elif func == 4:
            dist_mm = self._read_ultrasonic_sim()
            return f"CMD|4|{dist_mm}|$"

        elif func == 5:
            pitch, roll = self._read_imu_sim()
            return f"CMD|5|{pitch:.1f}|{roll:.1f}|$"

        elif func == 6:
            return f"CMD|6|{BATTERY_MV}|$"

        return "CMD|ERR|$"

    # --- Physics stepping ---

    def _step_motion(self) -> None:
        """Step the simulation forward based on current motion command."""
        c = self._client
        cmd = self._motion_cmd
        # speed_factor multiplies physics steps per CMD cycle
        steps_per_cmd = max(1, int(0.1 * self._speed_factor / SIM_TIMESTEP))

        for _ in range(steps_per_cmd):
            self._set_standing_pose()

            if cmd in (3, 4):
                # Forward/backward: level body, then set linear velocity
                pos, orn = p.getBasePositionAndOrientation(self._robot,
                                                           physicsClientId=c)
                yaw = p.getEulerFromQuaternion(orn)[2]
                level_orn = p.getQuaternionFromEuler([0, 0, yaw])
                p.resetBasePositionAndOrientation(self._robot, pos, level_orn,
                                                   physicsClientId=c)
                sign = 1.0 if cmd == 3 else -1.0
                p.resetBaseVelocity(self._robot,
                                    [sign * FORWARD_SPEED * math.cos(yaw),
                                     sign * FORWARD_SPEED * math.sin(yaw), 0],
                                    physicsClientId=c)
            elif cmd in (5, 6):
                # Turn: kinematic yaw update (stock firmware uses
                # stepping turn gait that physically relocates feet)
                pos, orn = p.getBasePositionAndOrientation(self._robot,
                                                           physicsClientId=c)
                yaw = p.getEulerFromQuaternion(orn)[2]
                sign = 1.0 if cmd == 5 else -1.0
                new_yaw = yaw + sign * math.radians(TURN_SPEED) * SIM_TIMESTEP
                new_orn = p.getQuaternionFromEuler([0, 0, new_yaw])
                p.resetBasePositionAndOrientation(self._robot, pos, new_orn,
                                                   physicsClientId=c)
                p.resetBaseVelocity(self._robot, [0, 0, 0], [0, 0, 0],
                                    physicsClientId=c)
            # cmd 1 (stop), 2 (stand), 7/8 (shift): just hold standing pose

            p.stepSimulation(physicsClientId=c)
            self._sim_time += SIM_TIMESTEP

    def _set_standing_pose(self) -> None:
        """Set all joints to standing position."""
        c = self._client
        for hip in _HIPS:
            p.setJointMotorControl2(self._robot, hip, p.POSITION_CONTROL,
                                    targetPosition=STAND_HIP, force=5.0,
                                    physicsClientId=c)
        for knee in _KNEES:
            p.setJointMotorControl2(self._robot, knee, p.POSITION_CONTROL,
                                    targetPosition=STAND_KNEE, force=5.0,
                                    physicsClientId=c)

    def _execute_action(self, code: int) -> None:
        """Execute an action group (simplified — just step sim forward)."""
        c = self._client
        steps = int(1.0 / SIM_TIMESTEP)
        for _ in range(steps):
            self._set_standing_pose()
            p.stepSimulation(physicsClientId=c)
            self._sim_time += SIM_TIMESTEP

    # --- Sensor simulation ---

    def _read_imu_sim(self) -> tuple[float, float]:
        """Read simulated IMU from body orientation."""
        _, orn = p.getBasePositionAndOrientation(self._robot,
                                                  physicsClientId=self._client)
        euler = p.getEulerFromQuaternion(orn)
        roll_deg = math.degrees(euler[0])
        pitch_deg = math.degrees(euler[1])
        return pitch_deg, roll_deg

    def _read_ultrasonic_sim(self) -> int:
        """Read simulated ultrasonic distance using ray casting."""
        c = self._client
        link_state = p.getLinkState(self._robot, ULTRASONIC_LINK,
                                    physicsClientId=c)
        sensor_pos = link_state[0]
        sensor_orn = link_state[1]

        rot_matrix = p.getMatrixFromQuaternion(sensor_orn)
        forward = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]

        min_dist = ULTRASONIC_MAX_RANGE

        for i in range(ULTRASONIC_NUM_RAYS):
            if ULTRASONIC_NUM_RAYS == 1:
                angle_offset = 0
            else:
                t = i / (ULTRASONIC_NUM_RAYS - 1) - 0.5
                angle_offset = math.radians(ULTRASONIC_CONE_HALF * 2 * t)

            cos_a = math.cos(angle_offset)
            sin_a = math.sin(angle_offset)
            ray_dir = [
                forward[0] * cos_a - forward[1] * sin_a,
                forward[0] * sin_a + forward[1] * cos_a,
                forward[2],
            ]

            ray_end = [
                sensor_pos[0] + ray_dir[0] * ULTRASONIC_MAX_RANGE,
                sensor_pos[1] + ray_dir[1] * ULTRASONIC_MAX_RANGE,
                sensor_pos[2] + ray_dir[2] * ULTRASONIC_MAX_RANGE,
            ]

            result = p.rayTest(sensor_pos, ray_end, physicsClientId=c)
            hit_id = result[0][0]
            if hit_id != -1 and hit_id != self._robot and hit_id != self._plane:
                hit_fraction = result[0][2]
                dist = hit_fraction * ULTRASONIC_MAX_RANGE
                if dist < min_dist:
                    min_dist = dist

        return int(min_dist * 1000)

    # --- Utility ---

    def get_position(self) -> tuple[float, float, float]:
        """Get the robot's current world position (x, y, z)."""
        pos, _ = p.getBasePositionAndOrientation(self._robot,
                                                  physicsClientId=self._client)
        return pos

    def get_heading(self) -> float:
        """Get the robot's current heading in degrees."""
        _, orn = p.getBasePositionAndOrientation(self._robot,
                                                  physicsClientId=self._client)
        euler = p.getEulerFromQuaternion(orn)
        return math.degrees(euler[2])

    def get_joint_states(self) -> dict:
        """Read all joint angles and foot positions for telemetry.

        Returns dict with hip/knee angles (radians) and foot positions (meters)
        for all four legs.
        """
        if not self._open or self._robot is None:
            return {}
        c = self._client
        joints = {}
        leg_names = ["fl", "fr", "rl", "rr"]
        hip_indices = [JOINT_FL_HIP, JOINT_FR_HIP, JOINT_RL_HIP, JOINT_RR_HIP]
        knee_indices = [JOINT_FL_KNEE, JOINT_FR_KNEE, JOINT_RL_KNEE, JOINT_RR_KNEE]

        for name, hip_idx, knee_idx, foot_link in zip(
            leg_names, hip_indices, knee_indices, _FOOT_LINKS
        ):
            hip_state = p.getJointState(self._robot, hip_idx, physicsClientId=c)
            knee_state = p.getJointState(self._robot, knee_idx, physicsClientId=c)
            foot_state = p.getLinkState(self._robot, foot_link, physicsClientId=c)
            joints[name] = {
                "hip": hip_state[0],    # angle in radians
                "knee": knee_state[0],  # angle in radians
                "foot": list(foot_state[0]),  # [x, y, z] world position
            }

        return joints

    @property
    def sim_time(self) -> float:
        """Current simulation time in seconds."""
        return self._sim_time
