"""Quick smoke test: connect to MockTransport, send commands, receive telemetry."""

import asyncio
import logging
import sys

sys.path.insert(0, ".")
from comms import DogComms, MSG_PONG, MSG_ACK, MSG_TELEM_IMU, MSG_TELEM_STATUS
from mock_serial import MockTransport

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("test_mock")


async def main():
    transport = MockTransport()
    comms = DogComms(transport)

    results = {"pong": False, "ack": False, "imu": False, "status": False}

    def on_pong(msg):
        results["pong"] = True
        logger.info("Got pong")

    def on_ack(msg):
        results["ack"] = True
        logger.info("Got ack: ref_type=%s ok=%s", msg.get("ref_type"), msg.get("ok"))

    def on_imu(msg):
        results["imu"] = True
        logger.info(
            "Got IMU: pitch=%.1f roll=%.1f yaw=%.1f",
            msg.get("pitch", 0), msg.get("roll", 0), msg.get("yaw", 0),
        )

    def on_status(msg):
        results["status"] = True
        logger.info("Got status: mode=%s battery=%s%%", msg.get("mode"), msg.get("battery_pct"))

    comms.on(MSG_PONG, on_pong)
    comms.on(MSG_ACK, on_ack)
    comms.on(MSG_TELEM_IMU, on_imu)
    comms.on(MSG_TELEM_STATUS, on_status)

    await comms.connect()

    # Wait a moment for telemetry to start flowing
    await asyncio.sleep(1.5)

    # Send a move command
    logger.info("Sending cmd_move forward...")
    await comms.send_move("forward", 0.5)
    await asyncio.sleep(0.5)

    # Send stand
    logger.info("Sending cmd_stand...")
    await comms.send_stand()
    await asyncio.sleep(0.5)

    await comms.disconnect()

    # Report results
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
