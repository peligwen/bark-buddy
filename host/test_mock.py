"""Smoke test: verify CMD protocol round-trip with MockTransport."""

import asyncio
import logging
import sys

sys.path.insert(0, ".")
from comms import DogComms
from mock_serial import MockTransport

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("test_mock")


async def main():
    transport = MockTransport()
    dog = DogComms(transport)

    results: dict[str, bool] = {}

    await dog.connect()

    # Test motion commands
    results["move_forward"] = await dog.move_forward()
    logger.info("move_forward: %s", results["move_forward"])

    results["turn_left"] = await dog.turn_left()
    logger.info("turn_left: %s", results["turn_left"])

    results["stop"] = await dog.stop()
    logger.info("stop: %s", results["stop"])

    results["stand"] = await dog.stand()
    logger.info("stand: %s", results["stand"])

    # Test balance toggle
    results["enable_balance"] = await dog.enable_balance()
    logger.info("enable_balance: %s", results["enable_balance"])

    results["disable_balance"] = await dog.disable_balance()
    logger.info("disable_balance: %s", results["disable_balance"])

    # Test action
    results["action_wave"] = await dog.action(1)
    logger.info("action_wave: %s", results["action_wave"])

    # Test IMU read
    imu = await dog.read_imu()
    results["read_imu"] = imu is not None and "pitch" in imu and "roll" in imu
    logger.info("read_imu: %s -> %s", results["read_imu"], imu)

    # Test battery read
    battery = await dog.read_battery()
    results["read_battery"] = battery is not None and battery > 0
    logger.info("read_battery: %s -> %smV", results["read_battery"], battery)

    await dog.disconnect()

    # Report
    logger.info("--- Results ---")
    all_ok = True
    for key, value in results.items():
        status = "PASS" if value else "FAIL"
        logger.info("  %s: %s", key, status)
        if not value:
            all_ok = False

    if all_ok:
        logger.info("All checks passed!")
    else:
        logger.error("Some checks failed!")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
