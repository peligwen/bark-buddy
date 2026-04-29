"""
Bark-Buddy web server.

Glue layer: aiohttp lifecycle, WebSocket session bring-up, and one
20 Hz pump that derives `event_fall` / `event_recovered` / `telem_odometry`
from cached transport state.

Behavioral logic lives in:
- `relay.TelemetryRelay`   — fan-out from host to all WS clients
- `router.CommandRouter`   — dispatches inbound WS messages
- `supervisor.TransportSupervisor` — owns the active transport, reconnect,
                                     mDNS auto-connect, USB hot-plug
- `behaviors/balance.py`, `behaviors/button_engage.py` — composable layers
"""

import argparse
import asyncio
import hashlib
import json
import logging
import os
import sys

from aiohttp import web

from behaviors.balance import BalanceLayer
from behaviors.button_engage import ButtonEngageBehavior
from dog import Dog
from dog.discover import find_serial_port, detect_serial_dog
from lock import ControlLock
from ota import OtaManager
from relay import TelemetryRelay
from router import CommandRouter
from status import build_telem_status
from supervisor import TransportSupervisor

logger = logging.getLogger(__name__)

BATTERY_POLL_HZ = 0.2

# Telemetry message types forwarded verbatim to browser clients.
_FORWARDED_TELEM = frozenset({
    "telem_sonar", "telem_battery", "telem_imu", "telem_status",
    "telem_event", "ota_status", "boot",
    "telem_button", "telem_gpio", "telem_i2c", "telem_servo_pins",
    "telem_joints",
})


class Server:
    def __init__(self, transport: Dog, web_dir: str, transport_label: str = "fw",
                 no_mdns: bool = False, open_browser: bool = False):
        self._web_dir = web_dir
        self._open_browser = open_browser

        self._motion = "stop"
        self._engaged = False
        self._web_hash = self._compute_web_hash(web_dir)
        self._last_servo_pins: dict | None = None
        self._client_names: dict[web.WebSocketResponse, str] = {}

        self._lock = ControlLock(timeout=30.0)
        self._relay = TelemetryRelay()
        self._button_event_queue: asyncio.Queue = asyncio.Queue()
        self._button_event_task: asyncio.Task | None = None
        self._pump_task: asyncio.Task | None = None

        # Behaviors are bound to the current transport; rebound on swap.
        self._balance = BalanceLayer(transport)
        self._button_engage = self._make_button_engage(transport)

        self._ota = OtaManager(
            get_transport_fn=lambda: self._supervisor.transport,
            get_transport_label_fn=lambda: self._supervisor.label,
        )

        self._supervisor = TransportSupervisor(
            transport, transport_label,
            no_mdns=no_mdns,
            on_replaced=self._on_transport_replaced,
            broadcast_status=self._broadcast_status,
        )

        self._router = CommandRouter(
            lock=self._lock,
            client_names=self._client_names,
            relay=self._relay,
            get_transport=lambda: self._supervisor.transport,
            balance=self._balance,
            set_motion=self._set_motion,
            on_reset=self._on_cmd_reset,
            on_restart_server=_restart_server,
        )

        self._register_transport_callbacks()

    # --- Behaviors ---

    def _make_button_engage(self, transport):
        return ButtonEngageBehavior(
            transport,
            lambda: self._engaged,
            lambda v: setattr(self, '_engaged', v),
            self._lock.is_locked,
        )

    def _set_motion(self, motion: str) -> None:
        self._motion = motion

    # --- Transport callback registration ---

    def _register_transport_callbacks(self):
        t = self._supervisor.transport if hasattr(self, '_supervisor') else None
        if t is None:
            return
        t.set_ack_callback(self._on_firmware_ack)
        t.set_telem_callback(self._on_firmware_telem)

    def _on_firmware_ack(self, msg: dict):
        if self._relay.has_clients():
            self._relay.enqueue(msg)

    def _on_firmware_telem(self, msg: dict):
        msg_type = msg.get("type", "")
        if msg_type == "telem_status":
            self._engaged = msg.get("engaged", self._engaged)
        if msg_type == "telem_servo_pins":
            self._last_servo_pins = msg
        if msg_type in _FORWARDED_TELEM:
            self._relay.enqueue(msg)
        if msg_type == "telem_button":
            event = msg.get("event", "")
            if event:
                try:
                    self._button_event_queue.put_nowait(event)
                except asyncio.QueueFull:
                    logger.warning("Button event queue full — dropping %s", event)

    async def _on_transport_replaced(self, new_transport, new_label: str):
        """Called by the supervisor inside the switch lock after a swap."""
        self._balance = BalanceLayer(new_transport)
        self._button_engage = self._make_button_engage(new_transport)
        self._router.set_balance(self._balance)
        new_transport.set_ack_callback(self._on_firmware_ack)
        new_transport.set_telem_callback(self._on_firmware_telem)
        if new_label == "mock" or "127.0.0.1" in new_label or "localhost" in new_label:
            self._supervisor.clear_detected_wifi()

    # --- aiohttp app ---

    @web.middleware
    async def _no_cache_middleware(self, request, handler):
        response = await handler(request)
        if request.path != "/ws":
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        return response

    async def _index_handler(self, request):
        return web.FileResponse(os.path.join(self._web_dir, "index.html"))

    async def start(self, host: str = "0.0.0.0", port: int = 8080):
        app = web.Application(middlewares=[self._no_cache_middleware])
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/", self._index_handler)
        self._ota.add_routes(app.router)
        app.router.add_static("/", self._web_dir)
        app.on_startup.append(self._on_startup)
        app.on_shutdown.append(self._on_shutdown)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, host, port)
        await site.start()
        logger.info("Server running at http://%s:%d", host, port)
        if self._open_browser:
            import webbrowser
            webbrowser.open(f"http://localhost:{port}")
        await asyncio.Event().wait()

    async def _on_startup(self, app: web.Application):
        await self._supervisor.transport.open()

        # Pull WiFi hint from boot/telem_status (delivered via telem callback).
        await asyncio.sleep(1)
        fw = self._supervisor.transport.firmware_info
        if fw.get("wifi") and fw.get("wifi_ip"):
            self._supervisor.seed_detected_wifi(fw["wifi_ip"], fw.get("tcp_port", 9000))
            logger.info("Firmware WiFi detected: %s (TCP port %s)",
                        fw["wifi_ip"], fw.get("tcp_port", 9000))

        await self._relay.start()
        self._button_event_task = asyncio.create_task(self._button_event_loop(),
                                                       name="button-events")
        self._pump_task = asyncio.create_task(self._pump_loop(), name="server-pump")
        await self._supervisor.start()

    async def _on_shutdown(self, app: web.Application):
        await self._supervisor.stop()
        # Best-effort disengage so the dog doesn't hold standing pose with no host.
        if self._supervisor.transport and self._supervisor.transport.is_open():
            await self._supervisor.transport.disengage_safe(timeout=1.0)
        await self._cancel_tasks(self._pump_task, self._button_event_task)
        await self._relay.stop()
        for ws in list(self._relay.clients()):
            await ws.close()
        if self._supervisor.transport and self._supervisor.transport.is_open():
            await self._supervisor.transport.close()

    @staticmethod
    async def _cancel_tasks(*tasks: "asyncio.Task | None") -> None:
        for task in tasks:
            if task is None:
                continue
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    # --- WebSocket session ---

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._relay.add_client(ws)
        logger.info("WebSocket client connected (%d total)", self._relay.client_count())

        # Initial state
        await self._relay.send_to(ws, self._build_status())
        await self._relay.send_to(ws, {"type": "version", "hash": self._web_hash})
        if self._lock.is_timed_out():
            self._lock.release()
        lock_msg = self._lock.status_msg()
        lock_msg["is_you"] = self._lock.is_locked_by(ws)
        await self._relay.send_to(ws, lock_msg)
        if self._last_servo_pins is not None:
            await self._relay.send_to(ws, self._last_servo_pins)

        try:
            async for raw_msg in ws:
                if raw_msg.type == web.WSMsgType.TEXT:
                    await self._router.dispatch(raw_msg.data, ws)
                elif raw_msg.type == web.WSMsgType.ERROR:
                    logger.warning("WebSocket error: %s", ws.exception())
        finally:
            self._relay.remove_client(ws)
            self._client_names.pop(ws, None)
            if self._lock.release_if_holder(ws):
                await self._router.broadcast_lock_status()
            logger.info("WebSocket client disconnected (%d remaining)",
                        self._relay.client_count())

        return ws

    # --- Pump: 20 Hz odometry + balance fan-out, 1/5 Hz battery status ---

    async def _pump_loop(self):
        imu_interval = 0.05
        battery_interval = 1.0 / BATTERY_POLL_HZ
        last_battery = 0.0
        while True:
            try:
                now = asyncio.get_running_loop().time()

                balance_event = self._balance.check()
                if balance_event["fallen"]:
                    await self._relay.broadcast({"type": "event_fall",
                                                  "imu": balance_event["imu"]})
                elif balance_event["recovered"]:
                    await self._relay.broadcast({"type": "event_recovered"})

                t = self._supervisor.transport
                if t and self._relay.has_clients():
                    await self._relay.broadcast({
                        "type": "telem_odometry",
                        "motion": self._motion,
                        "heading": round(t.get_heading(), 1),
                        "x": 0.0,
                        "y": 0.0,
                    })

                if now - last_battery >= battery_interval:
                    battery = t.get_battery_mv() if t else None
                    if battery is not None:
                        await self._broadcast_status(battery_mv=battery)
                    last_battery = now

                await asyncio.sleep(imu_interval)
            except asyncio.CancelledError:
                break
            except Exception:
                logger.exception("Pump loop error")
                await asyncio.sleep(1)

    async def _button_event_loop(self):
        while True:
            try:
                event = await self._button_event_queue.get()
            except asyncio.CancelledError:
                break
            try:
                await self._button_engage.on_button_event(event)
            except Exception:
                logger.exception("button event handler failed for %s", event)

    # --- Status assembly ---

    def _build_status(self, battery_mv: int | None = None) -> dict:
        return build_telem_status(
            transport=self._supervisor.transport,
            balance=self._balance,
            transport_label=self._supervisor.label,
            available_fw_version=self._ota.available_version,
            battery_mv=battery_mv,
        )

    async def _broadcast_status(self, battery_mv: int | None = None) -> None:
        status = self._build_status(battery_mv=battery_mv)
        if status.get("ota_status") == "complete":
            self._supervisor.clear_detected_wifi()
        await self._relay.broadcast(status)

    # --- Host commands ---

    async def _on_cmd_reset(self) -> None:
        self._motion = "stop"
        await self._broadcast_status()
        self._web_hash = self._compute_web_hash(self._web_dir)
        await self._relay.broadcast({"type": "reset"})
        await self._relay.broadcast({"type": "version", "hash": self._web_hash})

    @staticmethod
    def _compute_web_hash(web_dir: str) -> str:
        h = hashlib.md5()
        for name in sorted(os.listdir(web_dir)):
            path = os.path.join(web_dir, name)
            if os.path.isfile(path):
                with open(path, "rb") as f:
                    h.update(f.read())
        return h.hexdigest()[:8]


def _restart_server():
    """Re-exec the server. Strip --restart so the new process is a normal server."""
    import os, sys, signal, subprocess
    logger.info("Restarting server process...")
    new_argv = [a for a in sys.argv if a != "--restart"]
    subprocess.Popen([sys.executable] + new_argv, start_new_session=True)
    os.kill(os.getpid(), signal.SIGTERM)


async def main(args):
    import socket as _sock
    try:
        with _sock.create_connection(('127.0.0.1', args.port), timeout=0.2):
            print(f"\nbark is already running on port {args.port}. Use 'bark kill' to stop it first.")
            sys.exit(1)
    except OSError:
        pass

    fw_tcp = getattr(args, 'fw_tcp', None)
    if fw_tcp:
        from _paths import parse_fw_tcp
        host, tcp_port = parse_fw_tcp(fw_tcp)
        transport = Dog(host=host, tcp_port=tcp_port)
        is_local = host in ("127.0.0.1", "localhost")
        transport_label = "mock" if is_local else f"fw-wifi:{host}"
        logger.info("Using TCP firmware transport: %s:%d", host, tcp_port)
    else:
        serial_port = getattr(args, 'serial', None) or find_serial_port()
        if serial_port:
            logger.info("Found serial port: %s", serial_port)
            try:
                transport, transport_label = await detect_serial_dog(serial_port)
            except ConnectionError as e:
                logger.error("%s", e)
                print(f"\nNo MechDog detected on {serial_port}.\n"
                      "Plug in a device or run 'bark mock'.")
                sys.exit(1)
        else:
            print("\nNo MechDog detected. Plug in a device or run 'bark mock'.")
            sys.exit(1)

    web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "web"))
    server = Server(transport, web_dir, transport_label=transport_label,
                    no_mdns=getattr(args, 'no_mdns', False),
                    open_browser=getattr(args, 'open_browser', False))
    await server.start(host=args.host, port=args.port)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    parser = argparse.ArgumentParser(description="Bark-Buddy web server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8456)
    parser.add_argument("--serial", default=None,
                        help="Serial port (e.g. /dev/cu.usbserial-10). Auto-detected if omitted.")
    parser.add_argument("--fw-tcp", default=None, metavar="HOST[:PORT]",
                        help="Connect to firmware over TCP (e.g. 127.0.0.1:9001 for bark mock)")
    parser.add_argument("--no-mdns", action="store_true",
                        help="Disable mDNS auto-discovery")
    parser.add_argument("--restart", action="store_true",
                        help="Restart a running server via WebSocket command")
    args = parser.parse_args()

    if args.restart:
        import aiohttp
        async def do_restart():
            async with aiohttp.ClientSession() as s:
                try:
                    async with s.ws_connect(f'http://localhost:{args.port}/ws') as ws:
                        await ws.send_str('{"type":"cmd_restart_server"}')
                        print(f"Restart command sent to localhost:{args.port}")
                except Exception as e:
                    print(f"Could not connect to server: {e}")
        asyncio.run(do_restart())
    else:
        asyncio.run(main(args))
