"""
Bark-Buddy web server.

Serves static files from web/ and handles WebSocket connections for
real-time remote control and telemetry.
"""

import argparse
import asyncio
import json
import logging
import os

from aiohttp import web

from behaviors.balance import BalanceLayer
from behaviors.patrol import PatrolBehavior, Waypoint
from comms import DogComms, DIRECTION_MAP
from mock_serial import MockTransport

logger = logging.getLogger(__name__)

# Telemetry polling intervals
IMU_POLL_HZ = 10
ULTRASONIC_POLL_HZ = 5
BATTERY_POLL_HZ = 0.5


class Server:
    def __init__(self, dog: DogComms, web_dir: str):
        self._dog = dog
        self._web_dir = web_dir
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._poll_task: asyncio.Task | None = None
        self._balance = BalanceLayer(dog)
        self._patrol = PatrolBehavior(dog)
        self._mode = "remote"  # remote | patrol

    async def start(self, host: str = "0.0.0.0", port: int = 8080):
        app = web.Application()
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_static("/", self._web_dir, show_index=True)
        app.on_startup.append(self._on_startup)
        app.on_shutdown.append(self._on_shutdown)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        logger.info("Server running at http://%s:%d", host, port)
        await asyncio.Event().wait()

    async def _on_startup(self, app: web.Application):
        await self._dog.connect()
        await self._balance.start()

        # Register balance event callbacks
        self._balance.on_fall(self._on_fall)
        self._balance.on_recovered(self._on_recovered)

        # Register patrol callbacks
        self._patrol.on_waypoint_reached(self._on_waypoint_reached)
        self._patrol.on_patrol_complete(self._on_patrol_complete)
        self._patrol.on_position_update(self._on_position_update)

        self._poll_task = asyncio.create_task(self._telemetry_loop())
        self._reconnect_task = asyncio.create_task(self._reconnect_loop())

    async def _on_shutdown(self, app: web.Application):
        for task in (self._poll_task, getattr(self, '_reconnect_task', None)):
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        if self._patrol.running:
            await self._patrol.stop()
        await self._balance.stop()
        for ws in set(self._ws_clients):
            await ws.close()
        await self._dog.disconnect()

    async def _on_fall(self, imu: dict):
        """Broadcast fall event to all clients."""
        await self._broadcast({
            "type": "event_fall",
            "pitch": imu["pitch"],
            "roll": imu["roll"],
        })

    async def _on_recovered(self, imu: dict):
        """Broadcast recovery event to all clients."""
        await self._broadcast({"type": "event_recovered"})

    async def _on_waypoint_reached(self, index: int, wp: Waypoint):
        await self._broadcast({
            "type": "patrol_waypoint",
            "index": index,
            "x": wp.x, "y": wp.y, "heading": wp.heading,
        })

    async def _on_patrol_complete(self):
        self._mode = "remote"
        await self._broadcast({"type": "patrol_complete"})
        await self._broadcast({
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.fallen,
            "connected": self._dog.connected,
            "battery_mv": None,
        })

    async def _on_position_update(self, pos: dict):
        await self._broadcast({
            "type": "patrol_position",
            "x": round(pos["x"], 3),
            "y": round(pos["y"], 3),
            "heading": round(pos["heading"], 1),
        })

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        logger.info("WebSocket client connected (%d total)", len(self._ws_clients))

        # Send initial state
        await ws.send_str(json.dumps({
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.fallen,
            "connected": self._dog.connected,
            "battery_mv": None,
        }))

        try:
            async for raw_msg in ws:
                if raw_msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_message(raw_msg.data)
                elif raw_msg.type == web.WSMsgType.ERROR:
                    logger.warning("WebSocket error: %s", ws.exception())
        finally:
            self._ws_clients.discard(ws)
            logger.info("WebSocket client disconnected (%d remaining)", len(self._ws_clients))

        return ws

    async def _handle_ws_message(self, data: str):
        """Handle a JSON message from the browser."""
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON from browser: %s", data)
            return

        msg_type = msg.get("type")

        if msg_type == "cmd_move":
            direction = msg.get("direction", "stop")
            if direction in DIRECTION_MAP:
                await self._dog.move(direction)
            else:
                logger.warning("Unknown direction: %s", direction)

        elif msg_type == "cmd_stand":
            await self._dog.stand()

        elif msg_type == "cmd_balance":
            enabled = msg.get("enabled", True)
            if enabled:
                await self._balance.start()
            else:
                await self._balance.stop()
            # Broadcast updated balance state
            await self._broadcast({
                "type": "balance_state",
                "enabled": self._balance.enabled,
            })

        elif msg_type == "cmd_patrol":
            action = msg.get("action", "stop")
            if action == "start":
                waypoints_raw = msg.get("waypoints", [])
                waypoints = [
                    Waypoint(x=w["x"], y=w["y"], heading=w.get("heading", 0))
                    for w in waypoints_raw
                ]
                if waypoints:
                    self._patrol.set_waypoints(waypoints)
                    self._mode = "patrol"
                    await self._patrol.start()
                    await self._broadcast({
                        "type": "telem_status",
                        "mode": self._mode,
                        "balance": self._balance.enabled,
                        "fallen": self._balance.fallen,
                        "connected": self._dog.connected,
                        "battery_mv": None,
                    })
            elif action == "stop":
                await self._patrol.stop()
                self._mode = "remote"
                await self._broadcast({
                    "type": "telem_status",
                    "mode": self._mode,
                    "balance": self._balance.enabled,
                    "fallen": self._balance.fallen,
                    "connected": self._dog.connected,
                    "battery_mv": None,
                })

        elif msg_type == "cmd_action":
            code = msg.get("action", 1)
            await self._dog.action(code)

        else:
            logger.warning("Unknown WS message type: %s", msg_type)

    async def _broadcast(self, msg: dict):
        """Send a JSON message to all connected WebSocket clients."""
        data = json.dumps(msg)
        dead = set()
        for ws in self._ws_clients:
            try:
                await ws.send_str(data)
            except (ConnectionError, ConnectionResetError):
                dead.add(ws)
        self._ws_clients -= dead

    async def _telemetry_loop(self):
        """Poll IMU, ultrasonic, and battery, broadcast to browser clients."""
        imu_interval = 1.0 / IMU_POLL_HZ
        ultra_interval = 1.0 / ULTRASONIC_POLL_HZ
        battery_interval = 1.0 / BATTERY_POLL_HZ
        last_ultra = 0.0
        last_battery = 0.0
        imu_count = 0

        while True:
            try:
                now = asyncio.get_event_loop().time()

                # IMU polling via balance layer (handles fall detection too)
                imu = await self._balance.update()
                if imu and self._ws_clients:
                    await self._broadcast({
                        "type": "telem_imu",
                        "pitch": imu["pitch"],
                        "roll": imu["roll"],
                    })

                # Ultrasonic polling (5 Hz)
                if now - last_ultra >= ultra_interval:
                    dist = await self._dog.read_ultrasonic()
                    if dist is not None and self._ws_clients:
                        await self._broadcast({
                            "type": "telem_ultrasonic",
                            "distance_mm": dist,
                        })
                    last_ultra = now

                # Battery polling (0.5 Hz)
                if now - last_battery >= battery_interval:
                    battery = await self._dog.read_battery()
                    if battery is not None and self._ws_clients:
                        await self._broadcast({
                            "type": "telem_status",
                            "battery_mv": battery,
                            "connected": self._dog.connected,
                            "mode": self._mode,
                            "balance": self._balance.enabled,
                            "fallen": self._balance.fallen,
                        })
                    last_battery = now

                await asyncio.sleep(imu_interval)

            except asyncio.CancelledError:
                break
            except Exception:
                logger.exception("Telemetry loop error")
                await asyncio.sleep(1)


    async def _reconnect_loop(self):
        """Monitor connection and attempt reconnection with backoff."""
        backoff = 1
        while True:
            try:
                await asyncio.sleep(2)
                if not self._dog.connected:
                    logger.warning("Connection lost — attempting reconnect (backoff=%ds)", backoff)
                    await self._broadcast({
                        "type": "telem_status",
                        "connected": False,
                        "mode": self._mode,
                        "balance": self._balance.enabled,
                        "fallen": self._balance.fallen,
                        "battery_mv": None,
                    })
                    try:
                        await self._dog.disconnect()
                        await asyncio.sleep(backoff)
                        await self._dog.connect()
                        await self._balance.start()
                        backoff = 1
                        logger.info("Reconnected successfully")
                        await self._broadcast({
                            "type": "telem_status",
                            "connected": True,
                            "mode": self._mode,
                            "balance": self._balance.enabled,
                            "fallen": self._balance.fallen,
                            "battery_mv": None,
                        })
                    except Exception:
                        backoff = min(backoff * 2, 16)
                        logger.warning("Reconnect failed, next attempt in %ds", backoff)
                else:
                    backoff = 1
            except asyncio.CancelledError:
                break


async def main(args):
    if args.serial:
        from comms import SerialTransport
        transport = SerialTransport(port=args.serial)
        logger.info("Using serial transport: %s", args.serial)
    else:
        transport = MockTransport()
        logger.info("Using mock transport (no --serial specified)")

    dog = DogComms(transport)
    web_dir = os.path.join(os.path.dirname(__file__), "..", "web")
    web_dir = os.path.abspath(web_dir)
    server = Server(dog, web_dir)
    await server.start(host=args.host, port=args.port)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    parser = argparse.ArgumentParser(description="Bark-Buddy web server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--serial", default=None,
                        help="Serial port for MechDog (e.g. /dev/ttyUSB0). Omit for mock.")
    args = parser.parse_args()
    asyncio.run(main(args))
