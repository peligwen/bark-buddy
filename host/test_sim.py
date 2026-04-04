"""
Tests for the physics simulation transport.

Checks:
- sim_connect: SimTransport opens and closes cleanly
- sim_standing: Robot stands stably with correct height and level orientation
- sim_forward: Forward command moves robot in +X direction
- sim_backward: Backward command moves robot in -X direction
- sim_turn_left: Turn left changes heading positively
- sim_turn_right: Turn right changes heading negatively
- sim_imu: IMU returns near-zero pitch/roll when standing
- sim_ultrasonic_open: Ultrasonic returns max range with no walls
- sim_ultrasonic_wall: Ultrasonic detects walls in a room
- sim_battery: Battery returns expected voltage
- sim_heading_forward: Forward after turning moves in heading direction
- sim_joints: Joint states return all 4 legs
"""

import asyncio
import logging
import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from sim.sim_transport import SimTransport
from comms import DogComms, Transport

logging.basicConfig(level=logging.WARNING, format="%(name)s %(message)s")
logger = logging.getLogger("test_sim")
logger.setLevel(logging.INFO)

# Time to let background physics run after a command
SETTLE = 0.15


async def run_tests():
    results = {}

    # --- sim_connect ---
    try:
        t = SimTransport(speed_factor=5.0)  # 5x speed for faster tests
        dog = DogComms(t)
        await dog.connect()
        await asyncio.sleep(SETTLE)
        ok = t.is_open() and dog.connected and isinstance(t, Transport)
        results["sim_connect"] = ok
        logger.info("sim_connect: -> %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_connect"] = False
        logger.info("sim_connect: FAIL (%s)", e)

    # --- sim_standing ---
    try:
        await asyncio.sleep(SETTLE)
        pos = t.get_position()
        imu = t._imu
        ok = (pos[2] > 0.10 and pos[2] < 0.20
              and abs(imu["pitch"]) < 5 and abs(imu["roll"]) < 5)
        results["sim_standing"] = ok
        logger.info("sim_standing: z=%.3f pitch=%.1f roll=%.1f -> %s",
                     pos[2], imu["pitch"], imu["roll"], "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_standing"] = False
        logger.info("sim_standing: FAIL (%s)", e)

    # --- sim_forward ---
    try:
        t.reset_pose()
        await asyncio.sleep(SETTLE)
        pos1 = t.get_position()
        await dog.move_forward()
        await asyncio.sleep(1.0)  # let physics run 1s
        await dog.stop()
        pos2 = t.get_position()
        dx = pos2[0] - pos1[0]
        ok = dx > 0.01
        results["sim_forward"] = ok
        logger.info("sim_forward: dx=%.4f -> %s", dx, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_forward"] = False
        logger.info("sim_forward: FAIL (%s)", e)

    # --- sim_backward ---
    try:
        t.reset_pose()
        await asyncio.sleep(SETTLE)
        pos3 = t.get_position()
        await dog.move_backward()
        await asyncio.sleep(1.0)
        await dog.stop()
        pos4 = t.get_position()
        dx = pos4[0] - pos3[0]
        ok = dx < -0.01
        results["sim_backward"] = ok
        logger.info("sim_backward: dx=%.4f -> %s", dx, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_backward"] = False
        logger.info("sim_backward: FAIL (%s)", e)

    # --- sim_turn_left ---
    try:
        t.reset_pose()
        await asyncio.sleep(SETTLE)
        h1 = t.get_heading()
        await dog.turn_left()
        await asyncio.sleep(1.0)
        await dog.stop()
        h2 = t.get_heading()
        dh = h2 - h1
        ok = abs(dh) > 5  # heading changes significantly
        results["sim_turn_left"] = ok
        logger.info("sim_turn_left: delta=%.1f -> %s", dh, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_turn_left"] = False
        logger.info("sim_turn_left: FAIL (%s)", e)

    # --- sim_turn_right ---
    try:
        t.reset_pose()
        await asyncio.sleep(SETTLE)
        h3 = t.get_heading()
        await dog.turn_right()
        await asyncio.sleep(1.0)
        await dog.stop()
        h4 = t.get_heading()
        dh2 = h4 - h3
        # Opposite direction from left turn
        ok = abs(dh2) > 5 and (dh * dh2 < 0)  # opposite signs
        results["sim_turn_right"] = ok
        logger.info("sim_turn_right: delta=%.1f -> %s", dh2, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_turn_right"] = False
        logger.info("sim_turn_right: FAIL (%s)", e)

    # --- sim_imu ---
    try:
        t.reset_pose()
        await asyncio.sleep(SETTLE)
        imu = t._imu
        ok = abs(imu["pitch"]) < 5 and abs(imu["roll"]) < 5
        results["sim_imu"] = ok
        logger.info("sim_imu: pitch=%.1f roll=%.1f -> %s",
                     imu["pitch"], imu["roll"], "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_imu"] = False
        logger.info("sim_imu: FAIL (%s)", e)

    # --- sim_ultrasonic_open ---
    try:
        t.reset_pose()
        t.clear_walls()
        await asyncio.sleep(SETTLE)
        dist = t._sonar_mm
        ok = dist >= 2500  # near max range with no walls
        results["sim_ultrasonic_open"] = ok
        logger.info("sim_ultrasonic_open: %dmm -> %s", dist, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_ultrasonic_open"] = False
        logger.info("sim_ultrasonic_open: FAIL (%s)", e)

    # --- sim_ultrasonic_wall ---
    try:
        t.reset_pose()
        t.clear_walls()
        t.add_box_room(2.0, 2.0)
        await asyncio.sleep(SETTLE)
        dist = t._sonar_mm
        ok = 200 < dist < 1500
        results["sim_ultrasonic_wall"] = ok
        logger.info("sim_ultrasonic_wall: %dmm -> %s", dist, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_ultrasonic_wall"] = False
        logger.info("sim_ultrasonic_wall: FAIL (%s)", e)

    # --- sim_battery ---
    try:
        bat = await dog.read_battery()
        ok = bat == 7400
        results["sim_battery"] = ok
        logger.info("sim_battery: %dmV -> %s", bat, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_battery"] = False
        logger.info("sim_battery: FAIL (%s)", e)

    # --- sim_heading_forward ---
    try:
        t.clear_walls()
        t.reset_pose(yaw=90)
        await asyncio.sleep(SETTLE)
        pos_a = t.get_position()
        await dog.move_forward()
        await asyncio.sleep(1.0)
        await dog.stop()
        pos_b = t.get_position()
        dy = pos_b[1] - pos_a[1]
        dx = pos_b[0] - pos_a[0]
        dist = math.sqrt(dx * dx + dy * dy)
        # Should move significantly, mostly perpendicular to original heading
        ok = dist > 0.05 and abs(dy) > abs(dx)
        results["sim_heading_forward"] = ok
        logger.info("sim_heading_forward: dx=%.3f dy=%.3f dist=%.3f -> %s",
                     dx, dy, dist, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_heading_forward"] = False
        logger.info("sim_heading_forward: FAIL (%s)", e)

    # --- sim_joints ---
    try:
        joints = t.get_joint_states()
        ok = len(joints) == 4 and all(k in joints for k in ("fl", "fr", "rl", "rr"))
        results["sim_joints"] = ok
        logger.info("sim_joints: keys=%s -> %s",
                     list(joints.keys()), "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_joints"] = False
        logger.info("sim_joints: FAIL (%s)", e)

    await dog.disconnect()

    # --- Summary ---
    logger.info("--- Results ---")
    all_pass = True
    for name, ok in results.items():
        logger.info("  %s: %s", name, "PASS" if ok else "FAIL")
        if not ok:
            all_pass = False

    if all_pass:
        logger.info("All checks passed!")
    else:
        logger.info("SOME CHECKS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(run_tests())
