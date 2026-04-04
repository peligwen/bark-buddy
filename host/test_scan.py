"""
Tests for ultrasonic scan behavior and map store.

Checks:
- scan_execute: ScanBehavior completes a full 360° sweep and returns points
- scan_cancel: ScanBehavior can be cancelled mid-scan
- scan_points_have_coords: Each scan point has valid x/y coordinates
- map_add_scan: MapStore accumulates scan results
- map_clear: MapStore.clear() removes all data
- map_bounds: MapStore computes correct bounding box
- ws_scan_command: WebSocket scan start/stop commands work
- ws_map_data: WebSocket map data request returns map
"""

import asyncio
import json
import logging
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from behaviors.scan import ScanBehavior, ScanResult, SCAN_STEP_DEG
from behaviors.map_store import MapStore
from comms import DogComms
from sim.sim_transport import SimTransport

logging.basicConfig(level=logging.WARNING, format="%(name)s %(message)s")
logger = logging.getLogger("test_scan")
logger.setLevel(logging.INFO)


async def run_tests():

    results = {}

    # --- Unit tests: ScanBehavior ---
    transport = SimTransport()
    dog = DogComms(transport)
    await dog.connect()

    # Test: scan_execute
    try:
        scan = ScanBehavior(dog)
        points_received = []
        scan.on_point(lambda p, pct: points_received.append((p, pct)))

        result = await scan.execute(origin_x=0, origin_y=0, origin_heading=0)
        expected_steps = int(360 / SCAN_STEP_DEG)
        ok = (result is not None and
              len(result.points) == expected_steps and
              len(points_received) == expected_steps)
        results["scan_execute"] = ok
        logger.info("scan_execute: %d points (expected %d) -> %s",
                     len(result.points) if result else 0, expected_steps,
                     "PASS" if ok else "FAIL")
    except Exception as e:
        results["scan_execute"] = False
        logger.info("scan_execute: FAIL (%s)", e)

    # Test: scan_points_have_coords
    try:
        ok = all(
            isinstance(p.x, float) and isinstance(p.y, float)
            and p.distance_mm > 0
            for p in result.points
        )
        results["scan_points_have_coords"] = ok
        logger.info("scan_points_have_coords: %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["scan_points_have_coords"] = False
        logger.info("scan_points_have_coords: FAIL (%s)", e)

    # Test: scan_cancel
    try:
        scan2 = ScanBehavior(dog)
        cancel_task = asyncio.create_task(scan2.execute())
        await asyncio.sleep(0.5)
        await scan2.cancel()
        cancel_result = await cancel_task
        ok = cancel_result is None
        results["scan_cancel"] = ok
        logger.info("scan_cancel: %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["scan_cancel"] = False
        logger.info("scan_cancel: FAIL (%s)", e)

    # Test: scan_result_serializes
    try:
        d = result.to_dict()
        ok = ("points" in d and "origin_x" in d and
              len(d["points"]) == len(result.points))
        results["scan_result_serializes"] = ok
        logger.info("scan_result_serializes: %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["scan_result_serializes"] = False
        logger.info("scan_result_serializes: FAIL (%s)", e)

    await dog.disconnect()

    # --- Unit tests: MapStore ---

    # Test: map_add_scan
    try:
        ms = MapStore()
        idx = ms.add_scan(result)
        ok = idx == 0 and ms.scan_count == 1 and ms.point_count == len(result.points)
        results["map_add_scan"] = ok
        logger.info("map_add_scan: %d points -> %s", ms.point_count, "PASS" if ok else "FAIL")
    except Exception as e:
        results["map_add_scan"] = False
        logger.info("map_add_scan: FAIL (%s)", e)

    # Test: map_bounds
    try:
        bounds = ms.get_bounds()
        ok = (bounds["min_x"] < bounds["max_x"] and
              bounds["min_y"] < bounds["max_y"])
        results["map_bounds"] = ok
        logger.info("map_bounds: x=[%.2f, %.2f] y=[%.2f, %.2f] -> %s",
                     bounds["min_x"], bounds["max_x"],
                     bounds["min_y"], bounds["max_y"],
                     "PASS" if ok else "FAIL")
    except Exception as e:
        results["map_bounds"] = False
        logger.info("map_bounds: FAIL (%s)", e)

    # Test: map_clear
    try:
        ms.clear()
        ok = ms.scan_count == 0 and ms.point_count == 0
        results["map_clear"] = ok
        logger.info("map_clear: %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["map_clear"] = False
        logger.info("map_clear: FAIL (%s)", e)

    # Test: map_to_dict
    try:
        ms2 = MapStore()
        ms2.add_scan(result)
        d = ms2.to_dict()
        ok = ("points" in d and "bounds" in d and
              d["scan_count"] == 1 and d["point_count"] > 0)
        results["map_to_dict"] = ok
        logger.info("map_to_dict: %s", "PASS" if ok else "FAIL")
    except Exception as e:
        results["map_to_dict"] = False
        logger.info("map_to_dict: FAIL (%s)", e)

    # --- Integration test: WebSocket scan command ---
    try:
        from aiohttp import web, ClientSession
        from server import Server

        transport2 = SimTransport()
        dog2 = DogComms(transport2)
        web_dir = os.path.join(os.path.dirname(__file__), "..", "web")

        srv = Server(dog2, os.path.abspath(web_dir))
        app = web.Application()
        app.router.add_get("/ws", srv._ws_handler)
        app.on_startup.append(srv._on_startup)
        app.on_shutdown.append(srv._on_shutdown)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "127.0.0.1", 8099)
        await site.start()

        async with ClientSession() as session:
            async with session.ws_connect("http://127.0.0.1:8099/ws") as ws:
                # Read initial status
                raw = await asyncio.wait_for(ws.receive_str(), timeout=2)
                status = json.loads(raw)

                # Send scan start
                await ws.send_str(json.dumps({"type": "cmd_scan", "action": "start"}))

                # Collect messages until scan_complete or timeout
                scan_points = 0
                scan_complete = False
                try:
                    while True:
                        raw = await asyncio.wait_for(ws.receive_str(), timeout=30)
                        msg = json.loads(raw)
                        if msg.get("type") == "scan_point":
                            scan_points += 1
                        elif msg.get("type") == "scan_complete":
                            scan_complete = True
                            break
                except asyncio.TimeoutError:
                    pass

                ok = scan_complete and scan_points > 0
                results["ws_scan_command"] = ok
                logger.info("ws_scan_command: %d points, complete=%s -> %s",
                             scan_points, scan_complete, "PASS" if ok else "FAIL")

                # Request map data
                await ws.send_str(json.dumps({"type": "cmd_map", "action": "get"}))
                # Read until we get map_data
                map_ok = False
                try:
                    for _ in range(20):
                        raw = await asyncio.wait_for(ws.receive_str(), timeout=2)
                        msg = json.loads(raw)
                        if msg.get("type") == "map_data":
                            map_ok = msg.get("point_count", 0) > 0
                            break
                except asyncio.TimeoutError:
                    pass

                results["ws_map_data"] = map_ok
                logger.info("ws_map_data: %s", "PASS" if map_ok else "FAIL")

        await runner.cleanup()

    except Exception as e:
        results["ws_scan_command"] = results.get("ws_scan_command", False)
        results["ws_map_data"] = results.get("ws_map_data", False)
        logger.info("ws integration: FAIL (%s)", e)

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
