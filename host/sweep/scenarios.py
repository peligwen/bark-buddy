"""
Test scenarios for parameter sweeps.

Each scenario is a function:
    (phys: DogPhysics, params: dict, config: dict) -> dict[str, float|bool]

It sets up the world, runs physics for some duration, and returns metrics.
Scenarios are deterministic for a given param set.
"""

import math
from sim.physics import (
    DogPhysics, Vec3, Quat,
    standing_height, FOOT_R,
    STAND_HIP, STAND_KNEE,
)

SIM_DT = 1.0 / 240.0


def _step_n(phys: DogPhysics, seconds: float):
    """Step physics for a duration."""
    steps = int(seconds / SIM_DT)
    for _ in range(steps):
        phys.step(SIM_DT)


def _settle(phys: DogPhysics, seconds: float = 0.5):
    """Let the robot settle before a test."""
    _step_n(phys, seconds)


def _has_fallen(phys: DogPhysics, threshold_deg: float = 45.0) -> bool:
    """Check if the robot has fallen over."""
    pitch, roll, _ = phys.read_imu()
    if abs(pitch) > threshold_deg or abs(roll) > threshold_deg:
        return True
    if phys.pos.y < FOOT_R * 0.5:
        return True
    return False


def _track_walk(phys: DogPhysics, duration: float, direction: int = 3):
    """Walk for duration, tracking metrics. Returns metrics dict."""
    phys.set_motion(direction)

    start_pos = Vec3(phys.pos.x, phys.pos.y, phys.pos.z)
    max_pitch = 0.0
    max_roll = 0.0
    total_energy = 0.0
    fell = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 200)  # ~200 samples

    for step_i in range(steps):
        phys.step(SIM_DT)

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))

            # Approximate energy: sum of joint velocity squares
            for leg in phys.legs:
                total_energy += (leg.hip_vel ** 2 + leg.knee_vel ** 2) * SIM_DT

            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)  # stop

    end_pos = phys.pos
    dx = end_pos.x - start_pos.x
    dz = end_pos.z - start_pos.z
    distance = math.sqrt(dx * dx + dz * dz)

    return {
        "distance": round(distance, 4),
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "energy": round(total_energy, 4),
        "fell": fell,
        "final_height": round(phys.pos.y, 4),
    }


# ---- Scenarios ----

def flat_walk(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk forward on flat ground. Measures speed, stability, efficiency."""
    duration = config.get("duration", 5.0)
    _settle(phys)
    return _track_walk(phys, duration, direction=3)


def push_recovery(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walking robot gets a lateral push. Measures recovery while moving.

    Active gait provides dynamic stability that static stance lacks —
    a walking robot can recover from pushes a standing one cannot.
    """
    push_force = config.get("push_force", 0.15)  # N·s impulse
    walk_time_before = config.get("walk_time_before", 1.5)
    max_time = config.get("max_time", 3.0)
    settle_threshold = config.get("settle_threshold", 5.0)  # degrees

    _settle(phys, 0.5)

    # Start walking to get dynamic stability
    phys.set_motion(3)
    _step_n(phys, walk_time_before)

    # Record baseline
    pitch0, roll0, _ = phys.read_imu()

    # Apply lateral impulse
    from sim.physics import TOTAL_MASS
    phys.vel.z += push_force / TOTAL_MASS

    # Track recovery (keep walking)
    max_tilt = 0.0
    settle_time = max_time
    settled = False
    steps = int(max_time / SIM_DT)
    sample_interval = max(1, steps // 500)

    for step_i in range(steps):
        phys.step(SIM_DT)

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            tilt = max(abs(pitch), abs(roll))
            max_tilt = max(max_tilt, tilt)

            if not settled and step_i > 24 and tilt < settle_threshold:
                settle_time = step_i * SIM_DT
                settled = True

            if _has_fallen(phys):
                phys.set_motion(1)
                return {
                    "settle_time": max_time,
                    "max_tilt": round(max_tilt, 2),
                    "fell": True,
                    "recovered": False,
                }

    phys.set_motion(1)

    pitch_f, roll_f, _ = phys.read_imu()
    final_tilt = max(abs(pitch_f), abs(roll_f))

    return {
        "settle_time": round(settle_time, 4),
        "max_tilt": round(max_tilt, 2),
        "fell": False,
        "recovered": final_tilt < settle_threshold,
        "final_tilt": round(final_tilt, 2),
    }


def slope_climb(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk up an inclined surface. Tests gait robustness on slopes.

    Simulates slope by applying a constant backward gravity component.
    (True slope collision would need terrain mesh — this is a good proxy.)
    """
    slope_deg = config.get("slope_deg", 10.0)
    duration = config.get("duration", 5.0)

    _settle(phys)

    slope_rad = math.radians(slope_deg)
    # Tilt gravity to simulate slope: reduce effective vertical, add backward pull
    import sim.physics as pm
    original_step = phys.step

    gravity_pull = 9.81 * math.sin(slope_rad) * pm.TOTAL_MASS

    phys.set_motion(3)  # forward

    start_x = phys.pos.x
    max_pitch = 0.0
    fell = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 200)

    for step_i in range(steps):
        phys.step(SIM_DT)
        # Apply slope gravity pull (backward along X)
        phys.vel.x -= gravity_pull / pm.TOTAL_MASS * SIM_DT

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)
    net_distance = phys.pos.x - start_x

    return {
        "net_distance": round(net_distance, 4),
        "max_pitch": round(max_pitch, 2),
        "fell": fell,
        "made_progress": net_distance > 0.05,
    }


def obstacle_crossing(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk forward with a raised obstacle (rug/bump) in the path.

    Simulates a bump by temporarily raising the ground plane when the
    robot reaches a certain X position.
    """
    bump_height_mm = config.get("bump_height_mm", 5.0)
    bump_x_start = config.get("bump_x_start", 0.15)
    bump_x_end = config.get("bump_x_end", 0.25)
    duration = config.get("duration", 5.0)

    _settle(phys)
    phys.set_motion(3)

    start_x = phys.pos.x
    max_pitch = 0.0
    max_roll = 0.0
    fell = False
    crossed = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 200)
    bump_h = bump_height_mm / 1000.0

    import sim.physics as pm

    for step_i in range(steps):
        # Temporarily raise ground contact for feet on the bump
        on_bump = bump_x_start <= phys.pos.x <= bump_x_end
        if on_bump:
            # Shift foot contact threshold up by bump height
            for leg in phys.legs:
                if leg.foot_world.y < FOOT_R + bump_h:
                    penetration = (FOOT_R + bump_h) - leg.foot_world.y
                    if penetration > 0:
                        # Push body up
                        push = pm.CONTACT_K * penetration * SIM_DT
                        phys.vel.y += push / pm.TOTAL_MASS

        phys.step(SIM_DT)

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            if _has_fallen(phys):
                fell = True
                break

        if phys.pos.x > bump_x_end + 0.05:
            crossed = True

    phys.set_motion(1)
    distance = phys.pos.x - start_x

    return {
        "distance": round(distance, 4),
        "crossed": crossed,
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "fell": fell,
    }


def cliff_detect(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk toward a cliff edge. Tests if sonar + height can detect a drop.

    Simulates by removing ground support past a certain X position.
    Success = robot doesn't fall off. In real use, sonar would see the
    drop and trigger a stop behavior.
    """
    cliff_x = config.get("cliff_x", 0.30)
    duration = config.get("duration", 5.0)
    sonar_check_interval = config.get("sonar_check_hz", 20)

    _settle(phys)
    phys.set_motion(3)

    start_x = phys.pos.x
    fell_off = False
    max_x = start_x
    steps = int(duration / SIM_DT)
    sonar_step = max(1, int(240 / sonar_check_interval))
    sonar_readings = []

    import sim.physics as pm

    for step_i in range(steps):
        phys.step(SIM_DT)

        # Remove ground support past cliff edge
        if phys.pos.x > cliff_x:
            for leg in phys.legs:
                if leg.foot_world.x > cliff_x:
                    leg.contact = False
                    leg.contact_force = Vec3()

        max_x = max(max_x, phys.pos.x)

        if step_i % sonar_step == 0:
            sonar_mm = phys.raycast_ultrasonic()
            sonar_readings.append(sonar_mm)

        # Check for fall
        if phys.pos.y < -0.05:
            fell_off = True
            break

        if _has_fallen(phys):
            fell_off = True
            break

    phys.set_motion(1)
    overshoot = max(0, max_x - cliff_x)

    return {
        "fell_off": fell_off,
        "overshoot": round(overshoot, 4),
        "max_x": round(max_x, 4),
        "cliff_x": cliff_x,
        "sonar_readings_count": len(sonar_readings),
    }


def turn_accuracy(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Command a 90-degree turn, measure actual heading change."""
    target_deg = config.get("target_deg", 90.0)
    direction = config.get("direction", 5)  # 5=left, 6=right
    max_time = config.get("max_time", 5.0)

    _settle(phys)

    _, _, yaw0 = phys.read_imu()

    phys.set_motion(direction)

    import sim.physics as pm
    turn_speed = pm.TURN_SPEED  # deg/s
    estimated_time = abs(target_deg) / turn_speed if turn_speed > 0 else max_time
    turn_time = min(estimated_time * 1.2, max_time)  # 20% margin

    max_pitch = 0.0
    max_roll = 0.0
    fell = False
    steps = int(turn_time / SIM_DT)
    sample_interval = max(1, steps // 100)

    for step_i in range(steps):
        phys.step(SIM_DT)

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)
    _settle(phys, 0.5)

    _, _, yaw1 = phys.read_imu()
    actual_deg = yaw1 - yaw0
    # Normalize to [-180, 180]
    while actual_deg > 180:
        actual_deg -= 360
    while actual_deg < -180:
        actual_deg += 360

    heading_error = abs(abs(actual_deg) - target_deg)

    return {
        "target_deg": target_deg,
        "actual_deg": round(actual_deg, 2),
        "heading_error": round(heading_error, 2),
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "fell": fell,
    }


def start_stop(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk then stop. Measure overshoot distance after stop command."""
    walk_time = config.get("walk_time", 3.0)
    coast_time = config.get("coast_time", 2.0)

    _settle(phys)
    phys.set_motion(3)
    _step_n(phys, walk_time)

    # Record position at stop command
    stop_pos = Vec3(phys.pos.x, phys.pos.y, phys.pos.z)
    phys.set_motion(1)

    # Coast to a halt
    max_pitch = 0.0
    steps = int(coast_time / SIM_DT)
    sample_interval = max(1, steps // 100)

    for step_i in range(steps):
        phys.step(SIM_DT)
        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))

    dx = phys.pos.x - stop_pos.x
    dz = phys.pos.z - stop_pos.z
    overshoot = math.sqrt(dx * dx + dz * dz)

    return {
        "overshoot": round(overshoot, 4),
        "max_pitch_at_stop": round(max_pitch, 2),
        "fell": _has_fallen(phys),
    }


def sustained_walk(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk for 30s, check for drift and long-term stability."""
    duration = config.get("duration", 30.0)

    _settle(phys)
    phys.set_motion(3)

    start_pos = Vec3(phys.pos.x, phys.pos.y, phys.pos.z)
    _, _, start_yaw = phys.read_imu()

    max_pitch = 0.0
    max_roll = 0.0
    fell = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 300)  # ~300 samples over 30s
    pitch_samples = []

    for step_i in range(steps):
        phys.step(SIM_DT)

        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            pitch_samples.append(pitch)
            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)

    end_pos = phys.pos
    _, _, end_yaw = phys.read_imu()

    dx = end_pos.x - start_pos.x
    dz = end_pos.z - start_pos.z
    forward_dist = math.sqrt(dx * dx + dz * dz)

    # Lateral drift: distance from the line of travel
    if abs(dx) > 0.01:
        drift = abs(dz)  # lateral displacement
    else:
        drift = 0.0

    yaw_drift = end_yaw - start_yaw
    while yaw_drift > 180:
        yaw_drift -= 360
    while yaw_drift < -180:
        yaw_drift += 360

    # Pitch stability: std deviation
    if pitch_samples:
        mean_p = sum(pitch_samples) / len(pitch_samples)
        pitch_std = math.sqrt(sum((p - mean_p) ** 2 for p in pitch_samples) / len(pitch_samples))
    else:
        pitch_std = 0.0

    return {
        "distance": round(forward_dist, 4),
        "lateral_drift": round(drift, 4),
        "yaw_drift": round(yaw_drift, 2),
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "pitch_std": round(pitch_std, 2),
        "fell": fell,
    }


def walk_backward(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Walk backward on flat ground. Measures speed, stability, efficiency."""
    duration = config.get("duration", 5.0)
    _settle(phys)
    return _track_walk(phys, duration, direction=4)


def turn_left(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Turn left in place. Measures heading change, stability."""
    duration = config.get("duration", 3.0)

    _settle(phys)
    _, _, yaw0 = phys.read_imu()

    phys.set_motion(5)  # left turn

    max_pitch = 0.0
    max_roll = 0.0
    fell = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 100)

    for step_i in range(steps):
        phys.step(SIM_DT)
        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)
    _settle(phys, 0.3)

    _, _, yaw1 = phys.read_imu()
    heading_change = yaw1 - yaw0
    while heading_change > 180:
        heading_change -= 360
    while heading_change < -180:
        heading_change += 360

    return {
        "heading_change": round(heading_change, 2),
        "abs_heading_change": round(abs(heading_change), 2),
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "fell": fell,
    }


def turn_right(phys: DogPhysics, params: dict, config: dict) -> dict:
    """Turn right in place. Measures heading change, stability."""
    duration = config.get("duration", 3.0)

    _settle(phys)
    _, _, yaw0 = phys.read_imu()

    phys.set_motion(6)  # right turn

    max_pitch = 0.0
    max_roll = 0.0
    fell = False
    steps = int(duration / SIM_DT)
    sample_interval = max(1, steps // 100)

    for step_i in range(steps):
        phys.step(SIM_DT)
        if step_i % sample_interval == 0:
            pitch, roll, _ = phys.read_imu()
            max_pitch = max(max_pitch, abs(pitch))
            max_roll = max(max_roll, abs(roll))
            if _has_fallen(phys):
                fell = True
                break

    phys.set_motion(1)
    _settle(phys, 0.3)

    _, _, yaw1 = phys.read_imu()
    heading_change = yaw1 - yaw0
    while heading_change > 180:
        heading_change -= 360
    while heading_change < -180:
        heading_change += 360

    return {
        "heading_change": round(heading_change, 2),
        "abs_heading_change": round(abs(heading_change), 2),
        "max_pitch": round(max_pitch, 2),
        "max_roll": round(max_roll, 2),
        "fell": fell,
    }


# Registry
SCENARIOS = {
    "flat_walk": flat_walk,
    "walk_backward": walk_backward,
    "turn_left": turn_left,
    "turn_right": turn_right,
    "push_recovery": push_recovery,
    "slope_climb": slope_climb,
    "obstacle_crossing": obstacle_crossing,
    "cliff_detect": cliff_detect,
    "turn_accuracy": turn_accuracy,
    "start_stop": start_stop,
    "sustained_walk": sustained_walk,
}
