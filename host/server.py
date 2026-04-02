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
from comms import DogComms, DIRECTION_MAP
from mock_serial import MockTransport

logger = logging.getLogger(__name__)

# Telemetry polling intervals
IMU_POLL_HZ = 10
BATTERY_POLL_HZ = 0.5


class Server:
    def __init__(self, dog: DogComms, web_dir: str):
        self._dog = dog
        self._web_dir = web_dir
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._poll_task: asyncio.Task | None = None
        self._balance = BalanceLayer(dog)
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

        self._poll_task = asyncio.create_task(self._telemetry_loop())

    async def _on_shutdown(self, app: web.Application):
        if self._poll_task:
            self._poll_task.cancel()
            try:
                await self._poll_task
            except asyncio.CancelledError:
                pass
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
        """Poll IMU via balance layer and battery, broadcast to browser clients."""
        imu_interval = 1.0 / IMU_POLL_HZ
        battery_interval = 1.0 / BATTERY_POLL_HZ
        last_battery = 0.0

        while True:
            try:
                # IMU polling via balance layer (handles fall detection too)
                imu = await self._balance.update()
                if imu and self._ws_clients:
                    await self._broadcast({
                        "type": "telem_imu",
                        "pitch": imu["pitch"],
                        "roll": imu["roll"],
                    })

                # Battery polling (less frequent)
                now = asyncio.get_event_loop().time()
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


async def main(args):
    transport = MockTransport()  # TODO: add --serial flag for real hardware
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
    args = parser.parse_args()
    asyncio.run(main(args))
