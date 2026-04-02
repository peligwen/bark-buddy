"""
Tests for the PyBullet physics simulation transport.

Checks:
- sim_connect: SimTransport opens and closes cleanly
- sim_standing: Robot stands stably with correct height and level orientation
- sim_forward: Forward command moves robot in +X direction
- sim_backward: Backward command moves robot in -X direction
- sim_turn_left: Turn left increases heading (positive yaw)
- sim_turn_right: Turn right decreases heading (negative yaw)
- sim_imu: IMU returns near-zero pitch/roll when standing
- sim_ultrasonic_open: Ultrasonic returns max range with no walls
- sim_ultrasonic_wall: Ultrasonic detects walls in a room
- sim_battery: Battery returns expected voltage
- sim_room_walls: Box room walls are detectable from multiple angles
- sim_heading_forward: Forward after turning moves in heading direction
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


async def run_tests():
    results = {}

    # --- sim_connect ---
    try:
        t = SimTransport()
        dog = DogComms(t)
        await dog.connect()
        ok = t.is_open() and dog.connected and isinstance(t, Transport)
        results["sim_connect"] = ok
        logger.info("sim_connect: isinstance(Transport)=%s -> %s",
                     isinstance(t, Transport), "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_connect"] = False
        logger.info("sim_connect: FAIL (%s)", e)

    # --- sim_standing ---
    try:
        pos = t.get_position()
        imu = await dog.read_imu()
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
        pos1 = t.get_position()
        for _ in range(20):
            await dog.move_forward()
        pos2 = t.get_position()
        dx = pos2[0] - pos1[0]
        ok = 0.05 < dx < 0.5
        results["sim_forward"] = ok
        logger.info("sim_forward: dx=%.4f -> %s", dx, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_forward"] = False
        logger.info("sim_forward: FAIL (%s)", e)

    # --- sim_backward ---
    try:
        t.reset_pose()
        pos3 = t.get_position()
        for _ in range(20):
            await dog.move_backward()
        pos4 = t.get_position()
        dx = pos4[0] - pos3[0]
        ok = -0.5 < dx < -0.05
        results["sim_backward"] = ok
        logger.info("sim_backward: dx=%.4f -> %s", dx, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_backward"] = False
        logger.info("sim_backward: FAIL (%s)", e)

    # --- sim_turn_left ---
    try:
        t.reset_pose()
        h1 = t.get_heading()
        for _ in range(20):
            await dog.turn_left()
        h2 = t.get_heading()
        dh = h2 - h1
        ok = 45 < dh < 135
        results["sim_turn_left"] = ok
        logger.info("sim_turn_left: delta=%.1f -> %s", dh, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_turn_left"] = False
        logger.info("sim_turn_left: FAIL (%s)", e)

    # --- sim_turn_right ---
    try:
        t.reset_pose()
        h3 = t.get_heading()
        for _ in range(20):
            await dog.turn_right()
        h4 = t.get_heading()
        dh2 = h4 - h3
        ok = -135 < dh2 < -45
        results["sim_turn_right"] = ok
        logger.info("sim_turn_right: delta=%.1f -> %s", dh2, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_turn_right"] = False
        logger.info("sim_turn_right: FAIL (%s)", e)

    # --- sim_imu ---
    try:
        t.reset_pose()
        imu = await dog.read_imu()
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
        dist = await dog.read_ultrasonic()
        ok = dist == 3000
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
        dist = await dog.read_ultrasonic()
        ok = 500 < dist < 1500
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

    # --- sim_room_walls ---
    try:
        t.reset_pose()
        t.clear_walls()
        t.add_box_room(2.0, 2.0)
        d_fwd = await dog.read_ultrasonic()

        t.reset_pose(yaw=90)
        d_left = await dog.read_ultrasonic()

        ok = 500 < d_fwd < 1500 and 500 < d_left < 1500
        results["sim_room_walls"] = ok
        logger.info("sim_room_walls: fwd=%d left=%d -> %s",
                     d_fwd, d_left, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_room_walls"] = False
        logger.info("sim_room_walls: FAIL (%s)", e)

    # --- sim_heading_forward ---
    try:
        t.clear_walls()
        t.reset_pose(yaw=90)
        pos_a = t.get_position()
        for _ in range(20):
            await dog.move_forward()
        pos_b = t.get_position()
        dy = pos_b[1] - pos_a[1]
        dx = pos_b[0] - pos_a[0]
        ok = dy > 0.05 and abs(dx) < dy
        results["sim_heading_forward"] = ok
        logger.info("sim_heading_forward: dx=%.3f dy=%.3f -> %s",
                     dx, dy, "PASS" if ok else "FAIL")
    except Exception as e:
        results["sim_heading_forward"] = False
        logger.info("sim_heading_forward: FAIL (%s)", e)

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
