"""
Pure-Python rigid body physics for MechDog simulation.

No external dependencies. Simulates:
- Rigid body dynamics (position, orientation, velocity)
- 4-leg forward kinematics matching kinematics.h
- Ground contact with spring-damper normal + Coulomb friction
- Trot gait generator
- 2D ultrasonic raycast against wall segments

Dimensions and standing pose match the official MechDog spec and
firmware/test/kinematics.h exactly.
"""

import json
import logging
import math
import os
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

# ---- Dimensions (meters) — from kinematics.h / official spec ----
UPPER_LEN = 0.055
LOWER_LEN = 0.060
HIP_OFFSET_X = 0.085   # front/back from body center
HIP_OFFSET_Z = 0.030   # left/right from body center
HIP_OFFSET_Y = -0.025  # below body center
BODY_L = 0.170
BODY_W = 0.060
BODY_H = 0.035
FOOT_R = 0.008
BODY_MASS = 0.40        # kg (from URDF)
LEG_MASS = 0.045        # kg per leg (upper + lower + foot)
TOTAL_MASS = BODY_MASS + 4 * LEG_MASS

# Standing pose (radians)
STAND_HIP = 0.524       # ~30 degrees
STAND_KNEE = -0.611     # ~-35 degrees

# Inertia (body + legs at their offsets — parallel axis theorem)
Ixx = BODY_MASS * (BODY_W ** 2 + BODY_H ** 2) / 12
Iyy = BODY_MASS * (BODY_L ** 2 + BODY_H ** 2) / 12
Izz = BODY_MASS * (BODY_L ** 2 + BODY_W ** 2) / 12
for _hx, _hz in [(HIP_OFFSET_X, HIP_OFFSET_Z), (HIP_OFFSET_X, -HIP_OFFSET_Z),
                  (-HIP_OFFSET_X, HIP_OFFSET_Z), (-HIP_OFFSET_X, -HIP_OFFSET_Z)]:
    Ixx += LEG_MASS * (_hz ** 2)
    Iyy += LEG_MASS * (_hx ** 2 + _hz ** 2)
    Izz += LEG_MASS * (_hx ** 2)

# Ground contact parameters (critically damped for stability)
CONTACT_K = 800.0       # spring stiffness (N/m)
CONTACT_C = 40.0        # damping (N·s/m)
CONTACT_MU = 0.8        # friction coefficient
CONTACT_SUBSTEPS = 4    # sub-step contact resolution

# Joint PD control
JOINT_KP = 20.0         # proportional gain
JOINT_KD = 2.0          # derivative gain
JOINT_MAX_TORQUE = 1.0  # N·m

# Gait parameters
GAIT_HIP_AMPLITUDE = 0.14   # radians (~8 degrees)
GAIT_KNEE_AMPLITUDE = 0.09  # radians (~5 degrees)
GAIT_LIFT_HEIGHT = 0.012    # additional knee bend during swing
GAIT_FREQUENCY = 1.5        # Hz

# Movement
FORWARD_SPEED = 0.10    # m/s
TURN_SPEED = 45.0        # deg/s

# ---- Config loading ----

# All tunable params that can be overridden via gait_config.json
_TUNABLE_PARAMS = {
    "hip_amplitude": "GAIT_HIP_AMPLITUDE",
    "knee_amplitude": "GAIT_KNEE_AMPLITUDE",
    "lift_height": "GAIT_LIFT_HEIGHT",
    "gait_frequency": "GAIT_FREQUENCY",
    "contact_k": "CONTACT_K",
    "contact_c": "CONTACT_C",
    "contact_mu": "CONTACT_MU",
    "joint_kp": "JOINT_KP",
    "joint_kd": "JOINT_KD",
    "max_torque": "JOINT_MAX_TORQUE",
    "forward_speed": "FORWARD_SPEED",
    "turn_speed": "TURN_SPEED",
}

GAIT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "gait_config.json")


_DIRECTION_GAIT_KEYS = ("hip_amplitude", "lift_height", "gait_frequency")
_DIRECTIONS = ("forward", "backward", "turn_left", "turn_right")

# Per-direction gait overrides loaded from config (populated by load_gait_config)
_direction_overrides: dict[str, dict] = {}


def load_gait_config(path: str = None) -> dict:
    """Load tunable parameters from gait_config.json.

    Supports two formats:
    - Per-direction: {"directions": {"forward": {...}, "backward": {...}, ...}}
    - Flat (legacy): {"params": {"hip_amplitude": ...}} — applied to all directions

    Patches module-level constants for non-gait params (contact, joint, etc.).
    Stores per-direction gait params in _direction_overrides for GaitGenerator.
    Returns the raw loaded data.
    """
    global _direction_overrides
    path = path or GAIT_CONFIG_PATH
    if not os.path.exists(path):
        return {}

    with open(path) as f:
        data = json.load(f)

    g = globals()
    applied = {}

    # Load per-direction gait params
    directions = data.get("directions", {})
    if directions:
        for dir_name in _DIRECTIONS:
            if dir_name in directions:
                _direction_overrides[dir_name] = {
                    k: float(v) for k, v in directions[dir_name].items()
                    if k in _DIRECTION_GAIT_KEYS
                }

    # Load flat params (non-gait go to module globals, gait go to all directions)
    params = data.get("params", {})
    for config_key, module_attr in _TUNABLE_PARAMS.items():
        if config_key in params:
            value = float(params[config_key])
            if config_key not in _DIRECTION_GAIT_KEYS:
                g[module_attr] = value
            elif not directions:
                # Legacy flat format: apply gait params to all directions
                for dir_name in _DIRECTIONS:
                    if dir_name not in _direction_overrides:
                        _direction_overrides[dir_name] = {}
                    _direction_overrides[dir_name][config_key] = value
                g[module_attr] = value
            applied[config_key] = value

    if applied or _direction_overrides:
        logger.info("Loaded gait config from %s (%d directions, %d global params)",
                     path, len(_direction_overrides), len(applied))

    return data


# Auto-load config on import
_loaded_config = load_gait_config()


# ---- Complementary Filter ----
# Python mirror of firmware/include/cf_filter.h :: cf_update().
# Keep in sync: CF_ALPHA, axis conventions (gy=pitch rate, gx=roll rate).
# Used by: replay/analysis tools, future sim noise injection, onboard model dev.

CF_ALPHA = 0.95  # matches cf_filter.h default


class CFFilter:
    """Complementary filter: blends gyro integration with accel-derived angles."""

    __slots__ = ('pitch', 'roll', 'yaw', '_init')

    def __init__(self):
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0
        self._init = False

    def reset(self):
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0
        self._init = False

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Update pitch/roll/yaw from one IMU sample (units match firmware)."""
        accel_pitch = math.degrees(math.atan2(ax, math.sqrt(ay*ay + az*az)))
        accel_roll  = math.degrees(math.atan2(ay, math.sqrt(ax*ax + az*az)))

        if not self._init:
            self.pitch = accel_pitch
            self.roll  = accel_roll
            self._init = True
            return

        if 0.0 < dt < 1.0:
            self.pitch = CF_ALPHA * (self.pitch + gy * dt) + (1.0 - CF_ALPHA) * accel_pitch
            self.roll  = CF_ALPHA * (self.roll  + gx * dt) + (1.0 - CF_ALPHA) * accel_roll
            self.yaw  += gz * dt


# ---- Vec3 ----

class Vec3:
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, o):
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o):
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s):
        return Vec3(self.x * s, self.y * s, self.z * s)

    def __rmul__(self, s):
        return self.__mul__(s)

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, o):
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o):
        return Vec3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self):
        ln = self.length()
        if ln < 1e-12:
            return Vec3()
        return Vec3(self.x / ln, self.y / ln, self.z / ln)

    def __repr__(self):
        return f"Vec3({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"


# ---- Quaternion ----

class Quat:
    __slots__ = ('w', 'x', 'y', 'z')

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __mul__(self, o):
        """Quaternion multiplication."""
        return Quat(
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        )

    def conjugate(self):
        return Quat(self.w, -self.x, -self.y, -self.z)

    def normalized(self):
        ln = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if ln < 1e-12:
            return Quat()
        return Quat(self.w / ln, self.x / ln, self.y / ln, self.z / ln)

    def rotate(self, v: Vec3) -> Vec3:
        """Rotate a Vec3 by this quaternion."""
        qv = Quat(0, v.x, v.y, v.z)
        r = self * qv * self.conjugate()
        return Vec3(r.x, r.y, r.z)

    def to_euler(self):
        """Convert to (pitch, roll, yaw) in radians.

        Y-up frame: X=forward, Y=up, Z=left.
        Pitch = rotation around Z (nose up/down).
        Roll = rotation around X (lean left/right).
        Yaw = rotation around Y (heading).
        Decomposition order: Y * Z * X (yaw, then pitch, then roll).
        """
        # Yaw (Y-axis rotation)
        siny = 2 * (self.w * self.y + self.x * self.z)
        cosy = 1 - 2 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny, cosy)
        # Pitch (Z-axis rotation) — use asin for gimbal-safe extraction
        sinp = 2 * (self.w * self.z - self.x * self.y)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        # Roll (X-axis rotation)
        sinr = 2 * (self.w * self.x + self.y * self.z)
        cosr = 1 - 2 * (self.x * self.x + self.z * self.z)
        roll = math.atan2(sinr, cosr)
        return pitch, roll, yaw

    @staticmethod
    def from_euler(pitch, roll, yaw):
        """Create quaternion from (pitch, roll, yaw) radians.

        Y-up frame: applies yaw (Y), then pitch (Z), then roll (X).
        """
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        # Q = Qy * Qz * Qx
        # Qy = (cy, 0, sy, 0), Qz = (cp, 0, 0, sp), Qx = (cr, sr, 0, 0)
        return Quat(
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * cr + cy * sp * sr,
            cy * sp * cr - sy * cp * sr,
        )

    @staticmethod
    def from_axis_angle(axis: Vec3, angle: float):
        s = math.sin(angle / 2)
        c = math.cos(angle / 2)
        n = axis.normalized()
        return Quat(c, n.x * s, n.y * s, n.z * s)


# ---- Leg ----

# Leg indices
FL, FR, RL, RR = 0, 1, 2, 3

# Hip offset from body center (in body frame)
_HIP_OFFSETS = [
    Vec3(+HIP_OFFSET_X, HIP_OFFSET_Y, +HIP_OFFSET_Z),  # FL
    Vec3(+HIP_OFFSET_X, HIP_OFFSET_Y, -HIP_OFFSET_Z),  # FR
    Vec3(-HIP_OFFSET_X, HIP_OFFSET_Y, +HIP_OFFSET_Z),  # RL
    Vec3(-HIP_OFFSET_X, HIP_OFFSET_Y, -HIP_OFFSET_Z),  # RR
]


@dataclass
class Leg:
    hip_angle: float = STAND_HIP
    knee_angle: float = STAND_KNEE
    hip_vel: float = 0.0
    knee_vel: float = 0.0
    hip_target: float = STAND_HIP
    knee_target: float = STAND_KNEE
    foot_world: Vec3 = field(default_factory=Vec3)
    contact: bool = False
    contact_force: Vec3 = field(default_factory=Vec3)


def leg_fk(leg_idx: int, hip_angle: float, knee_angle: float) -> Vec3:
    """Forward kinematics: joint angles → foot position in body frame."""
    hip = _HIP_OFFSETS[leg_idx]
    # Upper leg (rotates about Y axis at hip)
    upper_y = -UPPER_LEN * math.cos(hip_angle)
    upper_x = UPPER_LEN * math.sin(hip_angle)
    knee_x = hip.x + upper_x
    knee_y = hip.y + upper_y
    # Lower leg
    total = hip_angle + knee_angle
    lower_y = -LOWER_LEN * math.cos(total)
    lower_x = LOWER_LEN * math.sin(total)
    return Vec3(knee_x + lower_x, knee_y + lower_y, hip.z)


def standing_height() -> float:
    """Body center height when standing with default angles."""
    foot = leg_fk(FL, STAND_HIP, STAND_KNEE)
    return -foot.y + FOOT_R


# ---- Gait Math Kernel ----
# Python mirror of firmware/include/gait_math.h :: gait_tick().
# Keep in sync: phase convention (radians, 0→2π), amplitude units (degrees),
# diagonal pairing (FL+RR = sinA, FR+RL = sinB), differential turning.
#
# GaitDir values: 0=forward, 1=backward, 2=turn_left, 3=turn_right

_GAIT_FL, _GAIT_FR, _GAIT_RL, _GAIT_RR = 0, 1, 2, 3  # leg indices

_GAIT_FORWARD, _GAIT_BACKWARD, _GAIT_TURN_LEFT, _GAIT_TURN_RIGHT = 0, 1, 2, 3


def gait_tick(phase_rad: float, gdir: int,
              hip_amp_deg: float, knee_amp_deg: float,
              speed: float = 1.0) -> list:
    """Compute per-leg angle offsets (degrees from standing pose) for one tick.

    Returns list of 4 (hip_offset_deg, knee_offset_deg) for FL/FR/RL/RR.

    Mirrors firmware/include/gait_math.h :: gait_tick() exactly.
    Turning strategy: differential amplitude (both sides swing forward,
    slow side at 0.3×). This differs from GaitGenerator.update() below
    which uses opposite-sign turns for the physics sim — a known divergence.
    """
    sin_a = math.sin(phase_rad)
    sin_b = math.sin(phase_rad + math.pi)

    dir_sign  = -1.0 if gdir == _GAIT_BACKWARD   else 1.0
    left_mul  =  0.3 if gdir == _GAIT_TURN_LEFT  else 1.0
    right_mul =  0.3 if gdir == _GAIT_TURN_RIGHT else 1.0

    ha = hip_amp_deg  * speed
    ka = knee_amp_deg * speed

    return [
        (dir_sign * ha * sin_a * left_mul,  -ka * max(0.0, sin_a)),  # FL
        (dir_sign * ha * sin_b * right_mul, -ka * max(0.0, sin_b)),  # FR
        (dir_sign * ha * sin_b * left_mul,  -ka * max(0.0, sin_b)),  # RL
        (dir_sign * ha * sin_a * right_mul, -ka * max(0.0, sin_a)),  # RR
    ]


# ---- Gait Generator ----

# Direction command codes
DIR_STOP = 0
DIR_FORWARD = 3
DIR_BACKWARD = 4
DIR_LEFT = 5
DIR_RIGHT = 6

# Direction names used in gait_config.json
_DIR_NAMES = {
    DIR_FORWARD: "forward",
    DIR_BACKWARD: "backward",
    DIR_LEFT: "turn_left",
    DIR_RIGHT: "turn_right",
}


def _default_gait_params():
    """Default gait params per direction, using module-level constants."""
    return {
        "forward":   {"hip_amplitude": GAIT_HIP_AMPLITUDE,
                      "lift_height": GAIT_LIFT_HEIGHT,
                      "gait_frequency": GAIT_FREQUENCY},
        "backward":  {"hip_amplitude": GAIT_HIP_AMPLITUDE,
                      "lift_height": GAIT_LIFT_HEIGHT,
                      "gait_frequency": GAIT_FREQUENCY},
        "turn_left": {"hip_amplitude": GAIT_HIP_AMPLITUDE,
                      "lift_height": GAIT_LIFT_HEIGHT,
                      "gait_frequency": GAIT_FREQUENCY},
        "turn_right": {"hip_amplitude": GAIT_HIP_AMPLITUDE,
                       "lift_height": GAIT_LIFT_HEIGHT,
                       "gait_frequency": GAIT_FREQUENCY},
    }


class GaitGenerator:
    """Trot gait: diagonal pairs alternate stance/swing.

    Supports per-direction gait parameters (hip_amplitude, lift_height,
    gait_frequency) loaded from gait_config.json.
    """

    def __init__(self):
        self.phase = 0.0
        self.active = False
        self.direction = 0   # 0=stop, 3=fwd, 4=back, 5=left, 6=right
        self.dir_params = _default_gait_params()

    def _get_params(self) -> dict:
        """Get gait params for current direction."""
        name = _DIR_NAMES.get(self.direction)
        if name and name in self.dir_params:
            return self.dir_params[name]
        return self.dir_params["forward"]

    def update(self, dt: float) -> list[tuple[float, float]]:
        """Returns list of 4 (hip_target, knee_target) tuples."""
        if not self.active or self.direction == 0:
            return [(STAND_HIP, STAND_KNEE)] * 4

        p = self._get_params()
        hip_amp = p["hip_amplitude"]
        lift_h = p["lift_height"]
        freq = p["gait_frequency"]

        self.phase = (self.phase + freq * dt) % 1.0
        targets = []

        for leg_idx in range(4):
            # Diagonal pairing: FL+RR in phase, FR+RL offset by 0.5
            if leg_idx in (FL, RR):
                leg_phase = self.phase
            else:
                leg_phase = (self.phase + 0.5) % 1.0

            hip_t = STAND_HIP
            knee_t = STAND_KNEE

            if self.direction in (DIR_FORWARD, DIR_BACKWARD):
                # Forward/backward: hip oscillates, swing leg lifts
                sign = 1.0 if self.direction == DIR_FORWARD else -1.0
                # Front legs: positive hip angle = forward swing
                # Rear legs: invert so they push in the same direction
                leg_sign = 1.0 if leg_idx < 2 else -1.0
                hip_offset = sign * leg_sign * hip_amp * math.sin(leg_phase * 2 * math.pi)
                hip_t += hip_offset

                # Swing phase (0.0 - 0.5): lift foot
                if leg_phase < 0.5:
                    lift = math.sin(leg_phase * 2 * math.pi) * lift_h
                    knee_t -= lift * 8  # amplify for knee angle

            elif self.direction in (DIR_LEFT, DIR_RIGHT):
                # Turn: one side steps forward, other backward
                sign = 1.0 if self.direction == DIR_LEFT else -1.0
                # Left legs step forward for left turn, right legs backward
                z_sign = 1.0 if leg_idx in (FL, RL) else -1.0
                hip_offset = sign * z_sign * hip_amp * math.sin(leg_phase * 2 * math.pi)
                hip_t += hip_offset

                if leg_phase < 0.5:
                    lift = math.sin(leg_phase * 2 * math.pi) * lift_h
                    knee_t -= lift * 8

            targets.append((hip_t, knee_t))

        return targets


# ---- Wall (for ultrasonic raycast) ----

@dataclass
class Wall:
    cx: float
    cy: float
    length: float
    thickness: float
    height: float
    angle: float


# ---- Main Physics World ----

class DogPhysics:
    """
    Simulates the MechDog as a rigid body with 4 legs on a ground plane.

    Coordinate frame: X = forward, Y = up, Z = left (matching URDF).
    Ground is at Y = 0.
    """

    def __init__(self):
        # Body state
        self.pos = Vec3(0, standing_height(), 0)
        self.vel = Vec3()
        self.orient = Quat()  # identity = level
        self.omega = Vec3()   # angular velocity (body frame)

        # Legs
        self.legs = [Leg() for _ in range(4)]
        self._update_foot_positions()

        # Gait (apply per-direction config overrides if loaded)
        self.gait = GaitGenerator()
        if _direction_overrides:
            for dir_name, overrides in _direction_overrides.items():
                if dir_name in self.gait.dir_params:
                    self.gait.dir_params[dir_name].update(overrides)

        # Walls
        self.walls: list[Wall] = []

        # External forces (from movement commands)
        self._ext_force = Vec3()
        self._ext_torque = Vec3()

        # Time
        self.time = 0.0

    def reset(self, x=0.0, z=0.0, yaw=0.0):
        """Reset to standing at given position/heading."""
        self.pos = Vec3(x, standing_height(), z)
        self.vel = Vec3()
        self.orient = Quat.from_euler(0, 0, yaw)  # pitch=0, roll=0, yaw=yaw
        self.omega = Vec3()
        self.gait.phase = 0.0
        for leg in self.legs:
            leg.hip_angle = STAND_HIP
            leg.knee_angle = STAND_KNEE
            leg.hip_vel = 0.0
            leg.knee_vel = 0.0
            leg.hip_target = STAND_HIP
            leg.knee_target = STAND_KNEE
        self._update_foot_positions()

    def step(self, dt: float):
        """Advance simulation by dt seconds."""
        # 1. Update gait targets
        targets = self.gait.update(dt)
        for i, (ht, kt) in enumerate(targets):
            self.legs[i].hip_target = ht
            self.legs[i].knee_target = kt

        # 2. Joint PD control
        for leg in self.legs:
            hip_err = leg.hip_target - leg.hip_angle
            hip_torque = JOINT_KP * hip_err - JOINT_KD * leg.hip_vel
            hip_torque = max(-JOINT_MAX_TORQUE, min(JOINT_MAX_TORQUE, hip_torque))
            leg.hip_vel += hip_torque * dt / 0.02
            leg.hip_angle += leg.hip_vel * dt

            knee_err = leg.knee_target - leg.knee_angle
            knee_torque = JOINT_KP * knee_err - JOINT_KD * leg.knee_vel
            knee_torque = max(-JOINT_MAX_TORQUE, min(JOINT_MAX_TORQUE, knee_torque))
            leg.knee_vel += knee_torque * dt / 0.02
            leg.knee_angle += leg.knee_vel * dt

        # 3-5. Sub-step contact + integration for stability
        sub_dt = dt / CONTACT_SUBSTEPS
        for _ in range(CONTACT_SUBSTEPS):
            self._update_foot_positions()
            self._contact_step(sub_dt)
            # Apply movement tracking after each sub-step so it's not
            # overwhelmed by contact friction
            self._apply_movement(sub_dt)

        self.time += dt

    def _contact_step(self, dt: float):
        """One sub-step of contact resolution + body integration."""
        total_force = Vec3(0, -9.81 * TOTAL_MASS, 0)
        total_torque = Vec3()

        for i, leg in enumerate(self.legs):
            foot = leg.foot_world
            penetration = FOOT_R - foot.y

            if penetration > 0:
                leg.contact = True
                r = foot - self.pos
                foot_vel = self.vel + self.omega.cross(r)

                # Normal force (spring-damper)
                fn = CONTACT_K * penetration - CONTACT_C * foot_vel.y
                fn = max(0, fn)

                # Friction — reduced when gait is active (legs provide thrust)
                fh = math.sqrt(foot_vel.x ** 2 + foot_vel.z ** 2)
                mu = CONTACT_MU if not self.gait.active else CONTACT_MU * 0.02
                if fh > 1e-6:
                    ff = min(mu * fn, fh * CONTACT_C * 2)
                    fx = -foot_vel.x / fh * ff
                    fz = -foot_vel.z / fh * ff
                else:
                    fx, fz = 0.0, 0.0

                contact_f = Vec3(fx, fn, fz)
                leg.contact_force = contact_f
                total_force = total_force + contact_f
                total_torque = total_torque + r.cross(contact_f)
            else:
                leg.contact = False
                leg.contact_force = Vec3()

        # Linear integration
        acc = total_force * (1.0 / TOTAL_MASS)
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt

        if self.pos.y < FOOT_R:
            self.pos.y = FOOT_R
            self.vel.y = max(0, self.vel.y)

        # Angular integration with clamped acceleration
        alpha = Vec3(
            total_torque.x / Ixx if Ixx > 0 else 0,
            total_torque.y / Iyy if Iyy > 0 else 0,
            total_torque.z / Izz if Izz > 0 else 0,
        )
        # Clamp angular acceleration to prevent instability
        MAX_ALPHA = 200.0
        alpha.x = max(-MAX_ALPHA, min(MAX_ALPHA, alpha.x))
        alpha.y = max(-MAX_ALPHA, min(MAX_ALPHA, alpha.y))
        alpha.z = max(-MAX_ALPHA, min(MAX_ALPHA, alpha.z))

        self.omega = self.omega + alpha * dt

        # Per-second damping rates, converted to per-substep factors
        # pitch/roll: 99% decay/sec (strong stabilization)
        # yaw: 50% decay/sec (moderate — allows turning)
        pr_damp = 0.99 ** dt   # ~exp(-4.6 * dt)
        yaw_damp = 0.5 ** dt   # ~exp(-0.69 * dt)
        self.omega.x *= pr_damp
        self.omega.z *= pr_damp
        self.omega.y *= yaw_damp

        wx, wy, wz = self.omega.x, self.omega.y, self.omega.z
        mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        if mag > 1e-8:
            dq = Quat.from_axis_angle(Vec3(wx, wy, wz), mag * dt)
            self.orient = (dq * self.orient).normalized()

        # Linear damping: 5% decay/sec
        lin_damp = 0.95 ** dt
        self.vel.x *= lin_damp
        self.vel.y *= 0.5 ** dt  # stronger vertical damping for stability
        self.vel.z *= lin_damp

    def _update_foot_positions(self):
        """Compute foot world positions from joint angles and body state."""
        for i, leg in enumerate(self.legs):
            foot_body = leg_fk(i, leg.hip_angle, leg.knee_angle)
            foot_world = self.pos + self.orient.rotate(foot_body)
            leg.foot_world = foot_world

    # ---- Sensors ----

    def read_imu(self) -> tuple[float, float, float]:
        """Read pitch, roll, yaw in degrees from body orientation."""
        pitch, roll, yaw = self.orient.to_euler()
        return math.degrees(pitch), math.degrees(roll), math.degrees(yaw)

    def raycast_ultrasonic(self) -> int:
        """
        2D raycast from the ultrasonic sensor position along body forward.
        Returns distance in mm (max 3000).
        """
        # Sensor is at front of body, pointing forward
        sensor_offset = Vec3(BODY_L / 2 + 0.01, 0.01, 0)
        sensor_world = self.pos + self.orient.rotate(sensor_offset)

        # Forward direction in world frame
        fwd = self.orient.rotate(Vec3(1, 0, 0))

        # 2D raycast (XZ plane)
        ox, oy_2d = sensor_world.x, sensor_world.z
        dx, dy_2d = fwd.x, fwd.z
        ray_len = math.sqrt(dx * dx + dy_2d * dy_2d)
        if ray_len < 1e-9:
            return 3000
        dx /= ray_len
        dy_2d /= ray_len

        max_range = 3.0
        min_dist = max_range

        for w in self.walls:
            wcos = math.cos(w.angle)
            wsin = math.sin(w.angle)
            half_l = w.length / 2
            ax = w.cx - wcos * half_l
            ay = w.cy - wsin * half_l
            bx = w.cx + wcos * half_l
            by = w.cy + wsin * half_l

            sx, sy = bx - ax, by - ay
            denom = dx * sy - dy_2d * sx
            if abs(denom) < 1e-9:
                continue
            t = ((ax - ox) * sy - (ay - oy_2d) * sx) / denom
            s = ((ax - ox) * dy_2d - (ay - oy_2d) * dx) / denom
            if t > 0 and 0 <= s <= 1 and t < min_dist:
                min_dist = t

        return int(min_dist * 1000)

    # ---- Motion control ----

    def set_motion(self, cmd: int):
        """Set motion command (1=stop, 3=fwd, 4=back, 5=left, 6=right)."""
        if cmd in (DIR_FORWARD, DIR_BACKWARD, DIR_LEFT, DIR_RIGHT):
            self.gait.active = True
            self.gait.direction = cmd
        else:
            self.gait.active = False
            self.gait.direction = 0
            self._ext_force = Vec3()
            self._ext_torque = Vec3()
            for leg in self.legs:
                leg.hip_target = STAND_HIP
                leg.knee_target = STAND_KNEE

    def _apply_movement(self, dt: float):
        """Apply commanded movement velocity tracking per sub-step.

        Models leg thrust: in a real quadruped, legs push against the
        ground to generate forward motion. We track target velocity
        each sub-step so contact friction doesn't fully absorb it.
        """
        cmd = self.gait.direction
        if cmd == 0:
            return

        has_traction = sum(1 for l in self.legs if l.contact) >= 2
        if not has_traction:
            return

        # Tracking rate: tuned via parameter sweep (mu=0.02, rate=50)
        alpha = 1.0 - math.exp(-50.0 * dt)

        if cmd in (3, 4):
            fwd = self.orient.rotate(Vec3(1, 0, 0))
            sign = 1.0 if cmd == 3 else -1.0
            target_vx = fwd.x * FORWARD_SPEED * sign
            target_vz = fwd.z * FORWARD_SPEED * sign
            self.vel.x += (target_vx - self.vel.x) * alpha
            self.vel.z += (target_vz - self.vel.z) * alpha
        elif cmd in (5, 6):
            sign = -1.0 if cmd == 5 else 1.0
            target_wy = sign * math.radians(TURN_SPEED)
            self.omega.y += (target_wy - self.omega.y) * alpha

    def apply_movement_force(self, cmd: int, dt: float):
        """External API — sets gait direction. Actual tracking happens in step()."""
        pass  # Movement is now handled internally via _apply_movement

    # ---- Walls ----

    def add_wall(self, start: tuple, end: tuple,
                 height: float = 0.3, thickness: float = 0.02):
        sx, sy = start
        ex, ey = end
        cx, cy = (sx + ex) / 2, (sy + ey) / 2
        dx, dy = ex - sx, ey - sy
        length = math.sqrt(dx * dx + dy * dy)
        angle = math.atan2(dy, dx)
        self.walls.append(Wall(cx, cy, length, thickness, height, angle))

    def clear_walls(self):
        self.walls.clear()

    # ---- Joint state readout ----

    def get_joint_states(self) -> dict:
        names = ["fl", "fr", "rl", "rr"]
        joints = {}
        for i, name in enumerate(names):
            leg = self.legs[i]
            joints[name] = {
                "hip": leg.hip_angle,
                "knee": leg.knee_angle,
                "foot": [leg.foot_world.x, leg.foot_world.y, leg.foot_world.z],
            }
        return joints
