"""
Bark-Buddy web server.

Serves static files from web/ and handles WebSocket connections for
real-time remote control and telemetry.
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
from ota import OtaManager
from lock import ControlLock
from behaviors.button_engage import ButtonEngageBehavior
from dog import Dog, DIRECTIONS
from dog.discover import find_serial_port, detect_serial_dog

logger = logging.getLogger(__name__)

BATTERY_POLL_HZ = 0.2

# Transport priority — higher number = higher priority for auto-switching
_TRANSPORT_PRIORITY: dict[str, int] = {
    "mock":     0,
    "fw-wifi":  1,   # WiFi custom firmware (mDNS discovered)
    "fw-usb":   2,   # USB custom firmware (highest priority)
}

def _transport_priority(label: str) -> int:
    if label == "mock":
        return 0
    if ":" in label:
        prefix, rest = label.split(":", 1)
        if prefix == "fw":
            is_usb = rest.startswith("/dev/") or rest.startswith("COM")
            return _TRANSPORT_PRIORITY["fw-usb" if is_usb else "fw-wifi"]
    return _TRANSPORT_PRIORITY.get(label, 0)


class Server:
    def __init__(self, transport: Dog, web_dir: str, transport_label: str = "fw",
                 no_mdns: bool = False, open_browser: bool = False):
        self._transport = transport
        self._web_dir = web_dir
        self._transport_label = transport_label
        self._no_mdns = no_mdns
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._poll_task: asyncio.Task | None = None
        self._reconnect_task: asyncio.Task | None = None
        self._detected_wifi: dict | None = None
        self._balance = BalanceLayer(transport)
        self._engaged: bool = False
        self._button_engage = ButtonEngageBehavior(
            transport,
            lambda: self._engaged,
            lambda v: setattr(self, '_engaged', v),
            lambda: self._lock.is_locked(),
        )
        self._mode = "remote"  # valid states: "remote"
        self._motion = "stop"  # last motion direction
        self._web_hash = self._compute_web_hash(web_dir)
        # Control lock
        self._lock = ControlLock(timeout=30.0)
        self._client_names: dict[web.WebSocketResponse, str] = {}
        self._ota = OtaManager(
            get_transport_fn=lambda: self._transport,
            get_transport_label_fn=lambda: self._transport_label,
        )
        self._device_monitor: "DeviceMonitor | None" = None
        self._switch_lock = asyncio.Lock()
        self._mdns_browser: "MdnsBrowser | None" = None
        self._open_browser = open_browser
        self._last_servo_pins: dict | None = None
        self._register_transport_callbacks()

    def _register_transport_callbacks(self):
        """Wire ack and telem forwarding from the current transport."""
        if self._transport and hasattr(self._transport, "set_ack_callback"):
            self._transport.set_ack_callback(self._on_firmware_ack)
        if self._transport and hasattr(self._transport, "set_telem_callback"):
            self._transport.set_telem_callback(self._on_firmware_telem)

    def _on_firmware_ack(self, msg: dict):
        """Forward all firmware acks to WebSocket clients."""
        if self._ws_clients:
            asyncio.ensure_future(self._broadcast(msg))

    def _on_firmware_telem(self, msg: dict):
        """Forward telemetry push messages directly to all WebSocket clients."""
        msg_type = msg.get("type", "")
        if msg_type == "telem_status":
            self._engaged = msg.get("engaged", self._engaged)
        if msg_type == "telem_servo_pins":
            self._last_servo_pins = msg
        if msg_type in ("telem_sonar", "telem_battery", "telem_imu", "telem_status",
                        "telem_event", "ota_status", "boot",
                        "telem_button", "telem_gpio", "telem_i2c", "telem_servo_pins"):
            asyncio.ensure_future(self._broadcast(msg))
        if msg_type == "telem_button":
            event = msg.get("event", "")
            if event:
                asyncio.ensure_future(self._button_engage.on_button_event(event))

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
        await self._transport.open()

        # Read firmware WiFi info from boot/telem_status (comes via telem callback)
        await asyncio.sleep(1)
        fw = self._transport.firmware_info
        if fw.get("wifi") and fw.get("wifi_ip"):
            self._detected_wifi = {
                "connected": True,
                "ip": fw["wifi_ip"],
                "port": fw.get("tcp_port", 9000),
            }
            logger.info("Firmware WiFi detected: %s (TCP port %s)", fw["wifi_ip"], fw.get("tcp_port", 9000))

        self._poll_task = asyncio.create_task(self._telemetry_loop())
        self._reconnect_task = asyncio.create_task(self._reconnect_loop())
        from device_monitor import DeviceMonitor
        self._device_monitor = DeviceMonitor(
            on_added=self._on_device_added,
            on_removed=self._on_device_removed,
        )
        await self._device_monitor.start()

        if not self._no_mdns:
            from mdns_browser import MdnsBrowser
            self._mdns_browser = MdnsBrowser(
                on_found=self._on_mdns_found,
                on_lost=self._on_mdns_lost,
            )
            await self._mdns_browser.start()

    async def _on_shutdown(self, app: web.Application):
        if self._mdns_browser:
            await self._mdns_browser.stop()
        if self._device_monitor:
            await self._device_monitor.stop()
        for task in (self._poll_task, self._reconnect_task):
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        for ws in set(self._ws_clients):
            await ws.close()
        if self._transport and self._transport.is_open():
            await self._transport.close()

    async def _replace_transport(self, new_transport, new_label: str):
        """Teardown current transport and swap in a new one."""
        async with self._switch_lock:
            for task in (self._poll_task, self._reconnect_task):
                if task:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
            self._poll_task = None
            self._reconnect_task = None

            # Close old transport
            try:
                if self._transport and self._transport.is_open():
                    await self._transport.close()
            except Exception:
                pass

            # Swap in new transport, balance, button_engage
            from behaviors.balance import BalanceLayer
            from behaviors.button_engage import ButtonEngageBehavior
            self._transport = new_transport
            self._register_transport_callbacks()
            self._balance = BalanceLayer(new_transport)
            self._button_engage = ButtonEngageBehavior(
                new_transport,
                lambda: self._engaged,
                lambda v: setattr(self, '_engaged', v),
                lambda: self._lock.is_locked(),
            )
            self._transport_label = new_label

            # Open new transport
            try:
                await new_transport.open()
            except Exception as e:
                logger.warning("_replace_transport open failed: %s — staying on %s", e, new_label)

            # Restart tasks
            self._poll_task = asyncio.create_task(self._telemetry_loop())
            self._reconnect_task = asyncio.create_task(self._reconnect_loop())
            await self._broadcast_status()

    @staticmethod
    def _compute_web_hash(web_dir: str) -> str:
        """Hash web files to detect when clients need to reload."""
        h = hashlib.md5()
        for name in sorted(os.listdir(web_dir)):
            path = os.path.join(web_dir, name)
            if os.path.isfile(path):
                with open(path, "rb") as f:
                    h.update(f.read())
        return h.hexdigest()[:8]

    async def _broadcast_lock_status(self) -> None:
        if self._lock.is_timed_out():
            self._lock.release()
        msg = self._lock.status_msg()
        dead = set()
        for ws in self._ws_clients:
            m = dict(msg)
            m["is_you"] = self._lock.is_locked_by(ws)
            try:
                await ws.send_str(json.dumps(m))
            except (ConnectionError, ConnectionResetError):
                dead.add(ws)
        self._ws_clients -= dead

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        logger.info("WebSocket client connected (%d total)", len(self._ws_clients))

        # Send initial state
        status = {
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.is_fallen,
            "connected": self._transport.is_open() if self._transport else False,
            "transport": self._transport_label,
            "engaged": self._transport.get_engaged() if self._transport else False,
            "ramping": self._transport.get_ramping() if self._transport else False,
            "battery_cutoff": self._transport.get_battery_cutoff() if self._transport else False,
            "battery_present": self._transport.get_battery_present() if self._transport else True,
            "fw_version": self._transport.get_fw_version() if self._transport else "",
            "available_fw_version": self._ota.available_version,
        }
        if self._transport:
            _r = self._transport.get_ramping()
            _e = self._transport.get_engaged()
            status["lifecycle"] = "ramping" if _r else ("active" if _e else "disengaged")
        else:
            status["lifecycle"] = "unknown"
        await ws.send_str(json.dumps(status))

        # Send version hash for stale client detection
        await ws.send_str(json.dumps({
            "type": "version", "hash": self._web_hash,
        }))

        # Send lock status
        if self._lock.is_timed_out():
            self._lock.release()
        lock_msg = self._lock.status_msg()
        lock_msg["is_you"] = self._lock.is_locked_by(ws)
        await ws.send_str(json.dumps(lock_msg))

        # Replay last known servo-pin mapping so browser never shows stale defaults
        if self._last_servo_pins is not None:
            await ws.send_str(json.dumps(self._last_servo_pins))

        try:
            async for raw_msg in ws:
                if raw_msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_message(raw_msg.data, ws)
                elif raw_msg.type == web.WSMsgType.ERROR:
                    logger.warning("WebSocket error: %s", ws.exception())
        finally:
            self._ws_clients.discard(ws)
            self._client_names.pop(ws, None)
            # Release lock if this client held it
            if self._lock.release_if_holder(ws):
                await self._broadcast_lock_status()
            logger.info("WebSocket client disconnected (%d remaining)", len(self._ws_clients))

        return ws

    async def _handle_ws_message(self, data: str, ws: web.WebSocketResponse):
        """Handle a JSON message from the browser."""
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON from browser: %s", data)
            return

        msg_type = msg.get("type")

        # --- Identity ---
        if msg_type == "cmd_identify":
            self._client_names[ws] = msg.get("name", "Operator")
            return

        # --- Lock commands (always allowed) ---
        if msg_type == "cmd_lock":
            name = msg.get("name", "Anonymous")
            if self._lock.can_control(ws):
                self._lock.acquire(ws, name)
                await self._broadcast_lock_status()
            elif self._lock.holder is not None:
                # Send challenge to current holder
                await self._lock.holder.send_str(json.dumps({
                    "type": "lock_challenge",
                    "challenger": name,
                }))
                # Notify challenger they need to wait
                await ws.send_str(json.dumps({
                    "type": "lock_denied",
                    "holder": self._lock.holder_name,
                }))
            return

        if msg_type == "cmd_unlock":
            if self._lock.release_if_holder(ws):
                await self._broadcast_lock_status()
            return

        if msg_type == "cmd_lock_yield":
            # Current holder yields to a challenger
            if self._lock.release_if_holder(ws):
                await self._broadcast_lock_status()
            return

        # --- Control commands (gated by lock) ---
        if msg_type in ("cmd_move", "cmd_stand", "cmd_balance",
                         "cmd_engage"):
            if self._lock.is_timed_out():
                self._lock.release()
            if not self._lock.can_control(ws):
                await ws.send_str(json.dumps({
                    "type": "lock_denied",
                    "holder": self._lock.holder_name,
                }))
                return
            # Auto-acquire lock on first control if no one holds it
            if self._lock.holder is None:
                self._lock.acquire(ws, self._client_names.get(ws, "Operator"))
                await self._broadcast_lock_status()
            # Refresh lock timeout on any control action
            if self._lock.is_locked_by(ws):
                self._lock.touch()

        if msg_type == "cmd_move":
            direction = msg.get("direction", "stop")
            if direction in DIRECTIONS:
                speed = msg.get("speed", 1.0)
                await self._transport.send_json({"type": "cmd_move", "direction": direction, "speed": speed})
                self._motion = direction
            else:
                logger.warning("Unknown direction: %s", direction)

        elif msg_type == "cmd_stand":
            await self._transport.send_json({"type": "cmd_stand"})
            self._motion = "stand"

        elif msg_type == "cmd_balance":
            enabled = msg.get("enabled", True)
            await self._balance.set_enabled(enabled)
            await self._broadcast({
                "type": "balance_state",
                "enabled": self._balance.enabled,
            })

        # --- Non-gated commands ---
        elif msg_type == "cmd_reset":
            self._motion = "stop"
            self._mode = "remote"
            await self._broadcast_status()
            # Recompute web hash in case files changed
            self._web_hash = self._compute_web_hash(self._web_dir)
            await self._broadcast({"type": "reset"})
            await self._broadcast({"type": "version", "hash": self._web_hash})

        elif msg_type == "cmd_restart_server":
            logger.warning("=== SERVER RESTART REQUESTED ===")
            try:
                await self._broadcast({"type": "server_restarting"})
            except Exception:
                pass
            _restart_server()

        elif msg_type in ("cmd_engage", "cmd_ota_update", "cmd_servo", "cmd_transform",
                          "cmd_gait_params", "cmd_probe_pin",
                          "cmd_balance_config", "cmd_offset", "cmd_servo_pin",
                          "cmd_buzzer", "cmd_led", "cmd_gpio", "cmd_i2c"):
            # Firmware-direct passthrough — forward as-is.
            # cmd_engage: lock gating already ran in the block above; forwarding here.
            if msg_type == "cmd_servo":
                logger.debug("Passthrough cmd_servo idx=%s pulse=%s",
                             msg.get("index"), msg.get("pulse_us"))
            if self._transport:
                try:
                    await self._transport.send_json(msg)
                except Exception as e:
                    logger.warning("Passthrough send failed (%s): %s", msg_type, e)

        else:
            logger.warning("Unknown WS message type: %s", msg_type)

    async def _broadcast_status(self, battery_mv=None):
        """Broadcast current status to all clients."""
        status = {
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.is_fallen,
            "connected": self._transport.is_open() if self._transport else False,
            "battery_mv": battery_mv,
            "engaged": self._transport.get_engaged() if self._transport else False,
            "ramping": self._transport.get_ramping() if self._transport else False,
            "battery_cutoff": self._transport.get_battery_cutoff() if self._transport else False,
            "battery_present": self._transport.get_battery_present() if self._transport else True,
            "fw_version": self._transport.get_fw_version() if self._transport else "",
            "available_fw_version": self._ota.available_version,
            "transport": self._transport_label,
        }
        ota_status = (self._transport.firmware_info if self._transport else {}).get('ota_status')
        if ota_status:
            status["ota_status"] = ota_status
            if ota_status == "complete":
                self._detected_wifi = None
        await self._broadcast(status)

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
        """Run odometry + balance checks; firmware pushes telem via set_telem_callback."""
        imu_interval = 0.05   # 20 Hz odometry/balance check
        battery_interval = 1.0 / BATTERY_POLL_HZ
        last_battery = 0.0

        while True:
            try:
                now = asyncio.get_running_loop().time()

                # Balance check (reads cached IMU — no I/O)
                balance_event = self._balance.check()
                if balance_event["fallen"]:
                    await self._broadcast({
                        "type": "event_fall",
                        "imu": balance_event["imu"],
                    })
                elif balance_event["recovered"]:
                    await self._broadcast({"type": "event_recovered"})

                # Odometry broadcast — heading from IMU yaw; no dead-reckoning position
                if self._transport and self._ws_clients:
                    await self._broadcast({
                        "type": "telem_odometry",
                        "motion": self._motion,
                        "heading": round(self._transport.get_heading(), 1),
                    })

                # Battery status broadcast
                if now - last_battery >= battery_interval:
                    battery = self._transport.get_battery_mv() if self._transport else None
                    if battery is not None:
                        await self._broadcast_status(battery_mv=battery)
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
                if self._transport and not self._transport.is_open():
                    logger.warning("Connection lost — attempting reconnect (backoff=%ds)", backoff)
                    await self._broadcast_status()
                    try:
                        await self._transport.close()
                        await asyncio.sleep(backoff)
                        await self._transport.open()
                        backoff = 1
                        logger.info("Reconnected successfully")
                        await self._broadcast_status()
                    except Exception:
                        backoff = min(backoff * 2, 16)
                        logger.warning("Reconnect failed, next attempt in %ds", backoff)
                else:
                    backoff = 1
            except asyncio.CancelledError:
                break


    async def _on_mdns_found(self, ip: str, port: int, props: dict):
        """Called when a MechDog is discovered via mDNS."""
        if self._detected_wifi and self._detected_wifi.get("ip") == ip:
            return
        self._detected_wifi = {"connected": True, "ip": ip, "port": port,
                               "fw_version": props.get("fw_version", "")}
        current_priority = _transport_priority(self._transport_label)
        wifi_priority = _TRANSPORT_PRIORITY["fw-wifi"]
        if wifi_priority > current_priority:
            logger.info("mDNS: auto-connecting to %s:%d", ip, port)
            try:
                transport = Dog(host=ip, tcp_port=port)
                await self._replace_transport(transport, f"fw:{ip}")
            except Exception as e:
                logger.warning("mDNS: auto-connect to %s:%d failed: %s", ip, port, e)
        else:
            logger.info("mDNS: found %s:%d (not switching, current=%s priority=%d)",
                        ip, port, self._transport_label, current_priority)
            await self._broadcast_status()

    async def _on_mdns_lost(self, ip: str):
        """Called when a discovered MechDog disappears from mDNS."""
        if not self._detected_wifi or self._detected_wifi.get("ip") != ip:
            return
        self._detected_wifi = None
        if ip in self._transport_label:
            logger.warning("mDNS: active device lost (%s) — disconnected", ip)
            await self._broadcast_status()

    async def _on_device_added(self, port: str):
        """Called when a USB serial device is plugged in."""
        current_priority = _transport_priority(self._transport_label)
        # Probe what's on the port
        try:
            transport, label = await detect_serial_dog(port)
        except Exception as e:
            logger.warning("Hot-plug: probe failed for %s: %s", port, e)
            return
        new_priority = _transport_priority(label)
        if new_priority > current_priority:
            logger.info("Hot-plug: auto-switching to %s (priority %d > %d)",
                        label, new_priority, current_priority)
            await self._replace_transport(transport, label)
        else:
            logger.info("Hot-plug: ignoring %s (priority %d <= current %d)",
                        label, new_priority, current_priority)

    async def _on_device_removed(self, port: str):
        """Called when a USB serial device is unplugged."""
        if port not in self._transport_label:
            return
        logger.warning("Hot-plug: active device removed: %s", port)
        # Fall back to WiFi firmware if available
        if self._detected_wifi and self._detected_wifi.get("ip"):
            ip = self._detected_wifi["ip"]
            tcp_port = self._detected_wifi.get("port", 9000)
            try:
                transport = Dog(host=ip, tcp_port=tcp_port)
                await self._replace_transport(transport, f"fw:{ip}")
                return
            except Exception as e:
                logger.warning("Hot-plug: WiFi fallback failed: %s", e)
        logger.warning("Hot-plug: no fallback transport available — disconnected")
        await self._broadcast_status()


def _restart_server():
    """Restart the server by spawning a replacement and killing self."""
    import os, sys, subprocess, signal
    logger.info("Restarting server process...")
    subprocess.Popen([sys.executable] + sys.argv,
                     start_new_session=True)
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
        # Explicit TCP connection (mock or remote WiFi)
        parts = fw_tcp.split(":")
        host = parts[0]
        tcp_port = int(parts[1]) if len(parts) > 1 else 9000
        transport = Dog(host=host, tcp_port=tcp_port)
        is_local = host in ("127.0.0.1", "localhost")
        transport_label = "mock" if is_local else f"fw-wifi:{host}"
        logger.info("Using TCP firmware transport: %s:%d", host, tcp_port)
    else:
        # Auto-detect USB serial
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
        # Send restart command to running server
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
