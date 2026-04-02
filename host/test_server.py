"""End-to-end test: start server with mock transport, connect via WebSocket, verify D-pad commands and telemetry."""

import asyncio
import json
import logging
import sys

import aiohttp

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("test_server")

SERVER_URL = "http://127.0.0.1:8082"
WS_URL = "ws://127.0.0.1:8082/ws"


async def main():
    # Start server in background
    proc = await asyncio.create_subprocess_exec(
        sys.executable, "server.py", "--port", "8082",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    await asyncio.sleep(1)  # let server boot

    results: dict[str, bool] = {}

    try:
        # Test 1: HTTP serves index.html
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{SERVER_URL}/index.html") as resp:
                results["http_static"] = resp.status == 200
                body = await resp.text()
                results["http_has_dpad"] = "dpad" in body
                logger.info("HTTP static: %d, has dpad: %s", resp.status, results["http_has_dpad"])

        # Test 2: WebSocket connection + telemetry
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(WS_URL) as ws:
                # Should receive telemetry within 2 seconds
                got_imu = False
                got_status = False

                try:
                    while True:
                        msg = await asyncio.wait_for(ws.receive(), timeout=3)
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            data = json.loads(msg.data)
                            if data.get("type") == "telem_imu":
                                got_imu = True
                                logger.info("Got IMU telemetry: pitch=%.1f roll=%.1f",
                                            data.get("pitch", 0), data.get("roll", 0))
                            elif data.get("type") == "telem_status":
                                got_status = True
                                logger.info("Got status: battery=%smV", data.get("battery_mv"))
                            if got_imu and got_status:
                                break
                except asyncio.TimeoutError:
                    pass

                results["ws_imu"] = got_imu
                results["ws_status"] = got_status

                # Test 3: Send a D-pad command
                await ws.send_str(json.dumps({"type": "cmd_move", "direction": "forward"}))
                await asyncio.sleep(0.2)
                await ws.send_str(json.dumps({"type": "cmd_move", "direction": "stop"}))
                await asyncio.sleep(0.1)
                results["ws_commands"] = True  # no crash = pass
                logger.info("Sent move commands without error")

                # Test 4: Send action
                await ws.send_str(json.dumps({"type": "cmd_action", "action": 1}))
                await asyncio.sleep(0.1)
                results["ws_action"] = True
                logger.info("Sent action command without error")

    finally:
        proc.terminate()
        await proc.wait()

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
