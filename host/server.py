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
import re

from config_util import read_config_local_value as _read_config_local_value

from aiohttp import web

from behaviors.balance import BalanceLayer
from behaviors.map_store import MapStore
from behaviors.scan import ScanBehavior
from comms import DogComms, DIRECTION_MAP
from sim.sim_transport import SimTransport

logger = logging.getLogger(__name__)

# Telemetry polling intervals
# Note: REPL transports (USB/WiFi) are slow (~0.5s per round-trip),
# so these rates are capped by actual throughput on real hardware.
# Base rates — overridden for sim+ (see _telemetry_loop)
IMU_POLL_HZ = 2
ULTRASONIC_POLL_HZ = 2
BATTERY_POLL_HZ = 0.2

# Transport priority — higher number = higher priority for auto-switching
_TRANSPORT_PRIORITY: dict[str, int] = {
    "sim":      0,
    "wifi":     1,   # WebREPL WiFi (stock firmware)
    "hybrid":   2,   # USB stock firmware
    "fw-wifi":  3,   # WiFi custom firmware (mDNS discovered)
    "fw-usb":   4,   # USB custom firmware (highest priority)
}

def _transport_priority(label: str) -> int:
    """Return priority for a transport label.

    Labels: 'sim', 'wifi:{host}', 'hybrid:{port}', 'fw:{ip}' (WiFi), 'fw:{/dev/...}' (USB)
    USB custom firmware gets priority 4 > WiFi custom firmware priority 3.
    """
    if ":" not in label:
        return _TRANSPORT_PRIORITY.get(label, 0)
    prefix, rest = label.split(":", 1)
    if prefix == "fw":
        # Distinguish USB fw (/dev/... or COM...) from WiFi fw
        is_usb = rest.startswith("/dev/") or rest.startswith("COM")
        return _TRANSPORT_PRIORITY["fw-usb" if is_usb else "fw-wifi"]
    return _TRANSPORT_PRIORITY.get(prefix, 0)


def _read_available_fw_version() -> str:
    """Parse FW_VERSION from firmware/include/config.h source."""
    config_path = os.path.join(os.path.dirname(__file__), "..", "firmware", "include", "config.h")
    try:
        with open(config_path) as f:
            m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', f.read())
            return m.group(1) if m else ""
    except FileNotFoundError:
        return ""


def _firmware_binary_path() -> str:
    """Path to the built firmware binary."""
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "firmware",
        ".pio", "build", "mechdog", "firmware.bin"
    ))


def _compute_sha256(path: str) -> str:
    """Return lowercase hex SHA-256 digest of the file at path."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def find_serial_port() -> str | None:
    """Auto-detect a USB serial port for MechDog."""
    import glob
    ports = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyUSB*")
    return ports[0] if ports else None


class Server:
    def __init__(self, dog: DogComms, web_dir: str, transport=None, transport_label="sim",
                 wifi_host: str | None = None, wifi_password: str | None = None,
                 no_mdns: bool = False, open_browser: bool = False):
        self._dog = dog
        self._web_dir = web_dir
        self._transport = transport
        self._transport_label = transport_label
        self._wifi_host = wifi_host
        self._wifi_password = wifi_password or _read_config_local_value("WEBREPL_PASS", "bark")
        self._no_mdns = no_mdns
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._poll_task: asyncio.Task | None = None
        self._reconnect_task: asyncio.Task | None = None
        self._detected_wifi: dict | None = None
        self._balance = BalanceLayer(dog)
        self._scan = ScanBehavior(dog)
        self._map = MapStore()
        self._mode = "remote"  # remote | scan
        self._motion = "stop"  # last motion direction
        self._action = None    # last action code or None
        self._web_hash = self._compute_web_hash(web_dir)
        # Control lock
        self._lock_holder: web.WebSocketResponse | None = None
        self._lock_name: str = ""
        self._lock_time: float = 0.0
        self._lock_timeout: float = 30.0  # seconds of inactivity before auto-release
        self._client_names: dict[web.WebSocketResponse, str] = {}
        self._available_fw_version = _read_available_fw_version()
        self._binary_sha256: str | None = None
        self._device_monitor: "DeviceMonitor | None" = None
        self._user_override_transport: bool = False  # True when user manually selected transport
        self._switch_lock = asyncio.Lock()
        self._mdns_browser: "MdnsBrowser | None" = None
        self._open_browser = open_browser

    @web.middleware
    async def _no_cache_middleware(self, request, handler):
        response = await handler(request)
        if request.path != "/ws":
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        return response

    async def _index_handler(self, request):
        return web.FileResponse(os.path.join(self._web_dir, "index.html"))

    async def _tuning_handler(self, request):
        return web.FileResponse(os.path.join(self._web_dir, "tuning.html"))

    async def start(self, host: str = "0.0.0.0", port: int = 8080):
        app = web.Application(middlewares=[self._no_cache_middleware])
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/", self._index_handler)
        app.router.add_get("/tuning", self._tuning_handler)
        app.router.add_get("/api/firmware/status",  self._handle_firmware_status)
        app.router.add_post("/api/firmware/build",  self._handle_firmware_build)
        app.router.add_get("/api/firmware/binary",  self._handle_firmware_binary)
        app.router.add_post("/api/firmware/update", self._handle_firmware_update)
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
        await self._dog.connect()
        # Don't auto-enable homeostasis — it breaks stock firmware gait.

        # Check if dog has WiFi available
        if 'usb' in self._transport_label:
            # Stock firmware: query WiFi status via REPL
            try:
                wifi = await self._transport.check_wifi_status()
                self._detected_wifi = wifi
                if wifi.get("connected") and wifi.get("ip"):
                    self._wifi_host = wifi["ip"]
                    logger.info("Dog WiFi detected: %s (%s)", wifi.get("ip"), wifi.get("ssid"))
            except Exception:
                self._detected_wifi = {"connected": False}
        elif 'fw' in self._transport_label:
            # Custom firmware: WiFi info comes from boot/status messages
            await asyncio.sleep(1)  # wait for boot message
            fw = self._transport.firmware_info
            if fw.get("wifi") and fw.get("wifi_ip"):
                self._wifi_host = fw["wifi_ip"]
                self._detected_wifi = {
                    "connected": True,
                    "ip": fw["wifi_ip"],
                    "port": fw.get("tcp_port", 9000),
                }
                logger.info("Firmware WiFi detected: %s (TCP port %s)", fw["wifi_ip"], fw.get("tcp_port", 9000))

        # Register balance event callbacks
        self._balance.on_fall(self._on_fall)
        self._balance.on_recovered(self._on_recovered)

        # Register scan callbacks
        self._scan.on_point(self._on_scan_point)
        self._scan.on_complete(self._on_scan_complete)

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
        if self._scan.running:
            await self._scan.cancel()
        await self._balance.stop()
        for ws in set(self._ws_clients):
            await ws.close()
        await self._dog.disconnect()

    async def _replace_transport(self, new_transport, new_label: str):
        """Teardown current transport and swap in a new one.

        Acquires _switch_lock, cancels both poll_task and reconnect_task before
        touching the transport, then reconnects and restarts both tasks.
        This is the single canonical method for all transport switching.
        """
        async with self._switch_lock:
            # Cancel telemetry loop
            if self._poll_task:
                self._poll_task.cancel()
                try:
                    await self._poll_task
                except asyncio.CancelledError:
                    pass
                self._poll_task = None
            # Cancel reconnect loop — prevents racing against _dog.connect()
            if self._reconnect_task:
                self._reconnect_task.cancel()
                try:
                    await self._reconnect_task
                except asyncio.CancelledError:
                    pass
                self._reconnect_task = None
            # Graceful LED shutdown on old transport
            try:
                if self._transport and self._transport.is_open():
                    await self._transport.set_led_status("shutdown")
            except Exception:
                pass
            # Disconnect old dog
            try:
                await self._dog.disconnect()
            except Exception:
                pass
            # Swap in new transport, dog, balance, scan
            from comms import DogComms
            from behaviors.balance import BalanceLayer
            from behaviors.scan import ScanBehavior
            self._transport = new_transport
            self._dog = DogComms(new_transport)
            self._balance = BalanceLayer(self._dog)
            self._balance.on_fall(self._on_fall)
            self._balance.on_recovered(self._on_recovered)
            self._scan = ScanBehavior(self._dog)
            self._scan.on_point(self._on_scan_point)
            self._scan.on_complete(self._on_scan_complete)
            self._transport_label = new_label
            # Connect new dog
            try:
                await self._dog.connect()
            except Exception as e:
                logger.warning("_replace_transport connect failed: %s — staying on %s", e, new_label)
            # Restart tasks
            self._poll_task = asyncio.create_task(self._telemetry_loop())
            self._reconnect_task = asyncio.create_task(self._reconnect_loop())
            await self._broadcast_status()

    async def _switch_transport(self, mode: str) -> dict:
        """Switch between transport modes: 'usb', 'wifi', 'sim'. Returns status dict."""
        self._user_override_transport = (mode != "sim")

        # Build the new transport object
        try:
            if mode == "usb":
                from repl_transport import ReplTransport
                port = find_serial_port()
                if not port:
                    return {"ok": False, "error": "No USB serial device found"}
                transport = ReplTransport(port=port)
                label = f"usb:{port.split('/')[-1]}"
            elif mode == "usb-fw":
                from firmware_transport import FirmwareTransport
                port = find_serial_port()
                if not port:
                    return {"ok": False, "error": "No USB serial device found"}
                transport = FirmwareTransport(port=port)
                label = f"fw:{port.split('/')[-1]}"
            elif mode == "wifi-fw":
                if not self._wifi_host:
                    return {"ok": False, "error": "No WiFi host configured"}
                from firmware_transport import FirmwareTransport
                transport = FirmwareTransport(host=self._wifi_host)
                label = f"fw:{self._wifi_host}"
            elif mode == "wifi":
                if not self._wifi_host:
                    return {"ok": False, "error": "No WiFi host configured. Start with --wifi"}
                from webrepl_transport import WebReplTransport
                transport = WebReplTransport(
                    host=self._wifi_host, password=self._wifi_password
                )
                label = f"wifi:{self._wifi_host}"
            elif mode == "sim":
                transport = SimTransport()
                label = "sim"
            elif mode == "hybrid":
                from hybrid_transport import HybridTransport
                port = find_serial_port()
                if not port:
                    return {"ok": False, "error": "No USB serial device found"}
                transport = HybridTransport(port=port)
                label = f"hybrid:{port.split('/')[-1]}"
            else:
                return {"ok": False, "error": f"Unknown mode: {mode}"}

            await self._replace_transport(transport, label)
            logger.info("Switched transport to %s", label)
            return {"ok": True, "transport": label}

        except Exception as e:
            logger.exception("Failed to switch transport to %s", mode)
            # Fall back to sim
            await self._replace_transport(SimTransport(), "sim")
            return {"ok": False, "error": str(e), "transport": "sim"}

    async def _add_live_point(self, distance_mm: int) -> None:
        """Add an ultrasonic reading as a map point using current position/heading."""
        import math
        pos = self._transport.get_position()
        heading = self._transport.get_heading()
        if pos is None or heading is None:
            return
        rad = math.radians(heading)
        dist_m = distance_mm / 1000.0
        x = pos[0] + dist_m * math.cos(rad)
        y = pos[1] + dist_m * math.sin(rad)
        point = self._map.add_point(x=x, y=y, z=0.09, distance_mm=distance_mm, source="ultrasonic")
        # Broadcast live point for real-time 2D map
        if point and self._ws_clients:
            await self._broadcast({
                "type": "scan_point",
                "x": round(x, 3),
                "y": round(y, 3),
                "distance_mm": distance_mm,
                "progress": 0,
                "confidence": round(point.confidence, 2),
            })

    async def _update_led_status(self):
        """Update sonar LED based on current state."""
        if self._lock_holder:
            await self._transport.set_led_status("controlled")
        else:
            await self._transport.set_led_status("connected")

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

    async def _on_scan_point(self, point, progress: int):
        """Broadcast each scan point as it's captured."""
        await self._broadcast({
            "type": "scan_point",
            "angle": round(point.angle, 1),
            "distance_mm": point.distance_mm,
            "x": round(point.x, 3),
            "y": round(point.y, 3),
            "progress": progress,
        })

    def _scan_task_done(self, task: asyncio.Task):
        """Handle scan task completion, including unexpected errors."""
        exc = task.exception() if not task.cancelled() else None
        if exc:
            logger.error("Scan task failed: %s", exc)
            self._mode = "remote"
            asyncio.create_task(self._broadcast_status())

    async def _on_scan_complete(self, result):
        """Store scan result and broadcast the full map."""
        self._map.add_scan(result)
        self._mode = "remote"
        await self._broadcast({"type": "scan_complete"})
        await self._broadcast_status()
        await self._broadcast({
            "type": "map_data",
            **self._map.to_dict(),
        })

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

    # --- Control lock ---

    def _lock_timed_out(self) -> bool:
        """Return True if the lock holder has timed out (sync, no side effects)."""
        if self._lock_holder and self._lock_time:
            import time
            return time.monotonic() - self._lock_time > self._lock_timeout
        return False

    async def _check_lock_timeout(self) -> None:
        """Release lock via _release_lock() if the holder has timed out."""
        if self._lock_timed_out():
            await self._release_lock()

    def _is_locked_by(self, ws: web.WebSocketResponse) -> bool:
        """Check if the given client holds the lock (does not trigger release)."""
        if self._lock_timed_out():
            return False
        return self._lock_holder is ws

    def _is_locked(self) -> bool:
        """Check if any client holds the lock (does not trigger release)."""
        if self._lock_timed_out():
            return False
        return self._lock_holder is not None

    def _can_control(self, ws: web.WebSocketResponse) -> bool:
        """Check if the given client is allowed to send commands (does not trigger release)."""
        if self._lock_timed_out():
            return True
        return self._lock_holder is None or self._lock_holder is ws

    def _lock_status_msg(self) -> dict:
        timed_out = self._lock_timed_out()
        return {
            "type": "lock_status",
            "locked": self._lock_holder is not None and not timed_out,
            "holder": self._lock_name if self._lock_holder and not timed_out else None,
            "is_you": False,  # overridden per-client
        }

    async def _acquire_lock(self, ws: web.WebSocketResponse, name: str) -> None:
        """Acquire the control lock for a client and wake the dog."""
        import time as _t
        self._lock_holder = ws
        self._lock_name = name
        self._lock_time = _t.monotonic()
        await self._dog.wake()

    async def _release_lock(self) -> None:
        """Release the control lock and put the dog to sleep."""
        self._lock_holder = None
        self._lock_name = ""
        self._lock_time = 0.0
        await self._dog.sleep()

    async def _broadcast_lock_status(self) -> None:
        await self._check_lock_timeout()
        msg = self._lock_status_msg()
        dead = set()
        for ws in self._ws_clients:
            m = dict(msg)
            m["is_you"] = (ws is self._lock_holder)
            try:
                await ws.send_str(json.dumps(m))
            except (ConnectionError, ConnectionResetError):
                dead.add(ws)
        self._ws_clients -= dead
        await self._update_led_status()

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        logger.info("WebSocket client connected (%d total)", len(self._ws_clients))

        # Send initial state
        transport = getattr(self._dog, '_transport', None)
        lifecycle = getattr(transport, 'get_lifecycle', lambda: 'unknown')()
        status = {
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.fallen,
            "connected": self._dog.connected,
            "scanning": self._scan.running,
            "transport": self._transport_label,
            "lifecycle": lifecycle,
        }
        fw_version_ws = getattr(transport, 'get_fw_version', lambda: '')()
        status["fw_version"] = fw_version_ws
        status["available_fw_version"] = self._available_fw_version
        wifi_info = getattr(self, '_detected_wifi', None)
        if wifi_info and wifi_info.get("connected"):
            status["wifi_available"] = True
            status["wifi_ip"] = wifi_info.get("ip", "")
            status["wifi_ssid"] = wifi_info.get("ssid", "")
        noise = self._transport.get_noise_params()
        if noise is not None:
            status["noise_params"] = noise
        await ws.send_str(json.dumps(status))

        # Send version hash for stale client detection
        await ws.send_str(json.dumps({
            "type": "version", "hash": self._web_hash,
        }))

        # Send lock status
        await self._check_lock_timeout()
        lock_msg = self._lock_status_msg()
        lock_msg["is_you"] = (ws is self._lock_holder)
        await ws.send_str(json.dumps(lock_msg))

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
            if self._lock_holder is ws:
                await self._release_lock()
                await self._broadcast_lock_status()
            logger.info("WebSocket client disconnected (%d remaining)", len(self._ws_clients))

        return ws

    async def _handle_ws_message(self, data: str, ws: web.WebSocketResponse):
        """Handle a JSON message from the browser."""
        import time as _time
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
            if self._can_control(ws):
                await self._acquire_lock(ws, name)
                await self._broadcast_lock_status()
            elif self._lock_holder is not None:
                # Send challenge to current holder
                await self._lock_holder.send_str(json.dumps({
                    "type": "lock_challenge",
                    "challenger": name,
                }))
                # Notify challenger they need to wait
                await ws.send_str(json.dumps({
                    "type": "lock_denied",
                    "holder": self._lock_name,
                }))
            return

        if msg_type == "cmd_unlock":
            if self._lock_holder is ws:
                await self._release_lock()
                await self._broadcast_lock_status()
            return

        if msg_type == "cmd_lock_yield":
            # Current holder yields to a challenger
            if self._lock_holder is ws:
                await self._release_lock()
                await self._broadcast_lock_status()
            return

        # --- Control commands (gated by lock) ---
        if msg_type in ("cmd_move", "cmd_stand", "cmd_balance",
                         "cmd_action", "cmd_scan"):
            await self._check_lock_timeout()
            if not self._can_control(ws):
                await ws.send_str(json.dumps({
                    "type": "lock_denied",
                    "holder": self._lock_name,
                }))
                return
            # Auto-acquire lock on first control if no one holds it
            if self._lock_holder is None:
                await self._acquire_lock(ws, self._client_names.get(ws, "Operator"))
                await self._broadcast_lock_status()
            # Refresh lock timeout on any control action
            if self._lock_holder is ws:
                self._lock_time = _time.monotonic()

        if msg_type == "cmd_move":
            if self._mode == "scan":
                return
            direction = msg.get("direction", "stop")
            if direction in DIRECTION_MAP:
                await self._dog.move(direction)
                self._motion = direction
                self._action = None
            else:
                logger.warning("Unknown direction: %s", direction)

        elif msg_type == "cmd_stand":
            if self._mode == "scan":
                return
            await self._dog.stand()
            self._motion = "stand"
            self._action = None

        elif msg_type == "cmd_balance":
            enabled = msg.get("enabled", True)
            if enabled:
                await self._balance.start()
            else:
                await self._balance.stop()
            await self._broadcast({
                "type": "balance_state",
                "enabled": self._balance.enabled,
            })

        elif msg_type == "cmd_pose":
            pose_name = msg.get("pose", "stand")
            # On real hardware, translate to stand/sit/lie commands
            pose_actions = {"sit": 4, "lie_down": 5, "stand": None}
            action_code = pose_actions.get(pose_name)
            if action_code:
                await self._dog.action(action_code)
            else:
                await self._dog.stand()
            self._motion = "stop"

        elif msg_type == "cmd_action":
            code = msg.get("action", 1)
            await self._dog.action(code)
            self._action = code
            self._motion = "stop"

        elif msg_type == "cmd_scan":
            action = msg.get("action", "start")
            if action == "start" and not self._scan.running:
                # Use transport's dead-reckoned position as scan origin
                ox, oy, heading = 0.0, 0.0, 0.0
                pos = self._transport.get_position()
                if pos is not None:
                    ox, oy = pos[0], pos[1]
                h = self._transport.get_heading()
                if h is not None:
                    heading = h
                self._mode = "scan"
                await self._broadcast_status()
                self._scan.start(
                    origin_x=ox, origin_y=oy,
                    origin_heading=heading,
                    done_callback=self._scan_task_done,
                )
            elif action == "stop":
                await self._scan.cancel()
                self._mode = "remote"
                await self._broadcast_status()

        # --- Non-gated commands ---
        elif msg_type == "cmd_map":
            action = msg.get("action", "get")
            if action == "get":
                await self._broadcast({
                    "type": "map_data",
                    **self._map.to_dict(),
                })
            elif action == "clear":
                self._map.clear()
                await self._broadcast({
                    "type": "map_data",
                    **self._map.to_dict(),
                })

        elif msg_type == "cmd_sim_noise":
            if self._transport.get_noise_params() is not None:
                self._transport.set_noise_params(msg.get("params", {}))
                await ws.send_str(json.dumps({
                    "type": "sim_noise_ack", "ok": True,
                    "params": self._transport.get_noise_params(),
                }))
            else:
                await ws.send_str(json.dumps({
                    "type": "sim_noise_ack", "ok": False, "error": "Not in sim mode",
                }))

        elif msg_type == "cmd_transport":
            mode = msg.get("mode", "sim")
            if msg.get("wifi_host"):
                self._wifi_host = msg["wifi_host"]
            result = await self._switch_transport(mode)
            await ws.send_str(json.dumps({"type": "transport_result", **result}))

        elif msg_type == "cmd_wifi_setup":
            ssid = msg.get("ssid", "")
            password = msg.get("password", "")
            if not ssid:
                await ws.send_str(json.dumps({"type": "wifi_setup_result", "ok": False, "error": "No SSID"}))
            else:
                result = await self._transport.setup_wifi(ssid, password)
                if result.get("ok"):
                    self._wifi_host = result.get("ip")
                    self._detected_wifi = {"connected": True, "ip": result["ip"], "ssid": ssid}
                await ws.send_str(json.dumps({"type": "wifi_setup_result", **result}))

        elif msg_type == "cmd_reset":
            if self._transport:
                self._transport.reset()
            self._motion = "stop"
            self._action = None
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

        elif msg_type in ("cmd_test_mode", "cmd_servo", "cmd_transform",
                          "cmd_gait_params", "cmd_shutdown"):
            # Firmware-direct passthrough — forward as-is
            if hasattr(self._transport, "send_json"):
                try:
                    await self._transport.send_json(msg)
                except Exception as e:
                    logger.warning("Passthrough send failed (%s): %s", msg_type, e)
            else:
                logger.warning("Transport has no send_json — cannot passthrough %s", msg_type)

        else:
            logger.warning("Unknown WS message type: %s", msg_type)

    async def _handle_firmware_status(self, request: web.Request) -> web.Response:
        transport = getattr(self._dog, '_transport', None)
        current = getattr(transport, 'get_fw_version', lambda: '')()
        available = self._available_fw_version
        is_wifi = 'fw:' in self._transport_label and '/dev/' not in self._transport_label
        binary_path = _firmware_binary_path()
        binary_exists = os.path.exists(binary_path)
        sha256_hex = self._binary_sha256 if binary_exists else None
        return web.json_response({
            "current_version": current,
            "available_version": available,
            "update_available": bool(current and available and current != available),
            "transport": self._transport_label,
            "can_ota": is_wifi,
            "binary_ready": binary_exists,
            "sha256": sha256_hex,
        })

    async def _do_firmware_build(self) -> dict:
        """Run pio build and return result dict."""
        firmware_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "firmware")
        )
        try:
            # Use subprocess_exec (not shell) — args passed as list, no injection risk
            proc = await asyncio.create_subprocess_exec(
                "pio", "run",
                cwd=firmware_dir,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            try:
                stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=180.0)
                ok = proc.returncode == 0
                output = stdout.decode(errors="replace")
            except asyncio.TimeoutError:
                proc.kill()
                return {"ok": False, "error": "Build timed out after 180s", "output": ""}
        except FileNotFoundError:
            return {"ok": False, "error": "pio not found in PATH", "output": ""}
        binary_path = _firmware_binary_path()
        binary_ready = ok and os.path.exists(binary_path)
        if binary_ready:
            self._binary_sha256 = _compute_sha256(binary_path)
        return {
            "ok": ok,
            "output": output[-3000:],
            "binary_ready": binary_ready,
        }

    async def _handle_firmware_build(self, request: web.Request) -> web.Response:
        result = await self._do_firmware_build()
        return web.json_response(result)

    async def _handle_firmware_binary(self, request: web.Request) -> web.Response:
        path = _firmware_binary_path()
        if not os.path.exists(path):
            return web.json_response({"error": "No firmware binary. Run /api/firmware/build first."}, status=404)
        return web.FileResponse(path, headers={
            "Content-Type": "application/octet-stream",
            "Content-Disposition": "attachment; filename=firmware.bin",
        })

    async def _handle_firmware_update(self, request: web.Request) -> web.Response:
        """Orchestrate OTA: build, then send cmd_ota_update to firmware."""
        # 1. Build
        build_data = await self._do_firmware_build()
        if not build_data.get("ok"):
            return web.json_response(
                {"ok": False, "error": "Build failed", "output": build_data.get("output")},
                status=500
            )
        # 2. Determine binary URL using request host
        host_parts = request.host.split(":")
        host_ip = host_parts[0]
        host_port = host_parts[1] if len(host_parts) > 1 else "8080"
        binary_url = f"http://{host_ip}:{host_port}/api/firmware/binary"
        # 3. Compute SHA-256 of the freshly built binary
        sha256_hex = _compute_sha256(_firmware_binary_path())
        self._binary_sha256 = sha256_hex
        # 4. Send OTA command
        transport = getattr(self._dog, '_transport', None)
        if not transport or not hasattr(transport, 'send_json'):
            return web.json_response({"ok": False, "error": "No firmware transport"}, status=400)
        try:
            await transport.send_json({
                "type": "cmd_ota_update",
                "url": binary_url,
                "sha256": sha256_hex,
            })
        except Exception as e:
            return web.json_response({"ok": False, "error": f"Failed to send OTA command: {e}"}, status=500)
        return web.json_response({
            "ok": True,
            "binary_url": binary_url,
            "new_version": self._available_fw_version,
            "sha256": sha256_hex,
        })

    async def _broadcast_status(self, battery_mv=None):
        """Broadcast current status to all clients."""
        transport = getattr(self._dog, '_transport', None)
        lifecycle = getattr(transport, 'get_lifecycle', lambda: 'unknown')()
        status = {
            "type": "telem_status",
            "mode": self._mode,
            "balance": self._balance.enabled,
            "fallen": self._balance.fallen,
            "connected": self._dog.connected,
            "scanning": self._scan.running,
            "battery_mv": battery_mv,
            "lifecycle": lifecycle,
        }
        if self._scan.running:
            status["scan_progress"] = self._scan.progress
        fw_version = getattr(transport, 'get_fw_version', lambda: '')()
        status["fw_version"] = fw_version
        status["available_fw_version"] = self._available_fw_version
        ota_status = (getattr(getattr(self._dog, '_transport', None), 'firmware_info', {}) or {}).get('ota_status')
        if ota_status:
            status["ota_status"] = ota_status
            if ota_status == "complete":
                self._detected_wifi = None  # Allow re-discovery after reboot
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
        """Poll IMU, ultrasonic, and battery, broadcast to browser clients."""
        # Use firmware-rate polling for transports with push telemetry
        from firmware_transport import FirmwareTransport
        from hybrid_transport import HybridTransport
        is_fast = isinstance(self._transport, (SimTransport, FirmwareTransport, HybridTransport))
        imu_interval = 1.0 / (20 if is_fast else IMU_POLL_HZ)
        ultra_interval = 1.0 / (20 if is_fast else ULTRASONIC_POLL_HZ)
        battery_interval = 1.0 / BATTERY_POLL_HZ
        wall_regen_interval = 1.0
        last_ultra = 0.0
        last_battery = 0.0
        last_wall_regen = 0.0
        last_point_count = 0

        while True:
            try:
                now = asyncio.get_running_loop().time()

                # IMU polling via balance layer (handles fall detection too)
                imu = await self._balance.update()
                if imu and self._ws_clients:
                    imu_msg = {
                        "type": "telem_imu",
                        "pitch": imu["pitch"],
                        "roll": imu["roll"],
                    }
                    # Include sim joint data when available (diagnostic only)
                    if self._transport:
                        joints = self._transport.get_joint_states()
                        if joints:
                            imu_msg["joints"] = joints
                    await self._broadcast(imu_msg)

                    # Odometry: dead-reckoned position + heading (separate from IMU)
                    if self._transport and self._ws_clients:
                        odom = {"type": "telem_odometry", "motion": self._motion}
                        pos = self._transport.get_position()
                        heading = self._transport.get_heading()
                        if pos is not None:
                            odom["x"] = round(pos[0], 4)
                            odom["y"] = round(pos[1], 4)
                        if heading is not None:
                            odom["heading"] = round(heading, 1)
                        await self._broadcast(odom)

                # Ultrasonic polling — continues during scan for live mapping
                if now - last_ultra >= ultra_interval:
                    dist = await self._dog.read_ultrasonic()
                    if dist is not None and self._ws_clients:
                        await self._broadcast({
                            "type": "telem_ultrasonic",
                            "distance_mm": dist,
                        })
                        # Continuous mapping: add each reading as a point
                        if dist < 3000 and self._transport:
                            await self._add_live_point(dist)
                    last_ultra = now

                # Battery polling (0.2 Hz = 5s interval)
                if now - last_battery >= battery_interval:
                    battery = await self._dog.read_battery()
                    if battery is not None and self._ws_clients:
                        await self._broadcast_status(battery_mv=battery)
                    last_battery = now

                # Point cloud maintenance (skip during scan)
                if not self._scan.running:
                    self._map.consolidate()
                    self._map.decay_tick()

                # Regenerate walls every second
                if now - last_wall_regen >= wall_regen_interval and self._ws_clients:
                    await self._broadcast({
                        "type": "map_data",
                        **self._map.to_dict(),
                    })
                    last_wall_regen = now

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
                    await self._broadcast_status()
                    try:
                        await self._dog.disconnect()
                        await asyncio.sleep(backoff)
                        await self._dog.connect()
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
        # Suppress re-announcements for the same device
        if self._detected_wifi and self._detected_wifi.get("ip") == ip:
            return
        self._detected_wifi = {"connected": True, "ip": ip, "port": port,
                               "fw_version": props.get("fw_version", "")}
        current_priority = _transport_priority(self._transport_label)
        wifi_priority = _TRANSPORT_PRIORITY["fw-wifi"]  # mDNS-discovered = custom firmware over WiFi
        if not self._user_override_transport and wifi_priority > current_priority:
            logger.info("mDNS: auto-connecting to %s:%d", ip, port)
            try:
                from firmware_transport import FirmwareTransport
                transport = FirmwareTransport(host=ip, tcp_port=port)
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
            logger.warning("mDNS: active device lost (%s) — falling back to sim", ip)
            from sim.sim_transport import SimTransport
            await self._replace_transport(SimTransport(), "sim")

    async def _on_device_added(self, port: str):
        """Called when a USB serial device is plugged in."""
        if self._user_override_transport:
            logger.info("Hot-plug: ignoring %s (user override active)", port)
            return
        current_priority = _transport_priority(self._transport_label)
        # Probe what's on the port
        try:
            transport, label = await _detect_serial_transport(port)
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
        # Only act if the removed port matches the active transport
        if port not in self._transport_label:
            return
        logger.warning("Hot-plug: active device removed: %s — falling back", port)
        # User's chosen device is gone — clear the override
        self._user_override_transport = False
        # Fall back: mDNS-discovered WiFi (if available), else sim
        if self._detected_wifi and self._detected_wifi.get("ip"):
            ip = self._detected_wifi["ip"]
            tcp_port = self._detected_wifi.get("port", 9000)
            try:
                from firmware_transport import FirmwareTransport
                transport = FirmwareTransport(host=ip, tcp_port=tcp_port)
                await self._replace_transport(transport, f"fw:{ip}")
                return
            except Exception as e:
                logger.warning("Hot-plug: WiFi fallback failed: %s", e)
        from sim.sim_transport import SimTransport
        await self._replace_transport(SimTransport(), "sim")


def _restart_server():
    """Restart the server by spawning a replacement and killing self."""
    import os, sys, subprocess, signal
    logger.info("Restarting server process...")
    subprocess.Popen([sys.executable] + sys.argv,
                     start_new_session=True)
    os.kill(os.getpid(), signal.SIGTERM)


def _probe_serial_sync(port: str) -> str:
    """Blocking serial probe — runs in a thread via asyncio.to_thread.

    Returns 'fw', 'hybrid', or 'fallback'.
    """
    import serial
    import time
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        # Custom firmware takes up to 2.5s to boot (500ms settle + 2000ms softstart).
        # Wait long enough to catch a response even after DTR reset.
        time.sleep(3.0)

        # Try JSON ping first (custom firmware) — retry a few times in case of timing
        resp = ""
        for _ in range(3):
            ser.write(b'{"type":"ping"}\n')
            time.sleep(0.5)
            resp += ser.read(ser.in_waiting).decode(errors="replace")
            if '"pong"' in resp:
                break
        if '"pong"' in resp:
            ser.close()
            return "fw"

        # Try Ctrl+C for MicroPython REPL → use hybrid mode
        ser.write(b'\x03\x03\r\n')
        time.sleep(0.5)
        resp = ser.read(ser.in_waiting).decode(errors="replace")
        ser.close()

        if ">>>" in resp or "MicroPython" in resp:
            return "hybrid"

    except Exception as e:
        logger.warning("Detection failed: %s", e)

    return "fallback"


async def _detect_serial_transport(port: str):
    """Auto-detect firmware type on a serial port.

    Detection order:
    1. JSON ping → custom firmware (FirmwareTransport)
    2. Ctrl+C → MicroPython → hybrid mode (HybridTransport)
    3. Fallback → hybrid mode (best-effort)

    The blocking serial I/O is run off the event loop via asyncio.to_thread.
    """
    logger.info("Auto-detecting firmware on %s...", port)
    result = await asyncio.to_thread(_probe_serial_sync, port)

    if result == "fw":
        from firmware_transport import FirmwareTransport
        transport = FirmwareTransport(port=port)
        label = f"fw:{port.split('/')[-1]}"
        logger.info("Detected custom firmware on %s", port)
        return transport, label

    if result == "hybrid":
        from hybrid_transport import HybridTransport
        transport = HybridTransport(port=port)
        label = f"hybrid:{port.split('/')[-1]}"
        logger.info("Detected MicroPython on %s, using hybrid mode", port)
        return transport, label

    # Fallback to hybrid (most likely stock firmware)
    from hybrid_transport import HybridTransport
    transport = HybridTransport(port=port)
    label = f"hybrid:{port.split('/')[-1]}"
    logger.info("Defaulting to hybrid transport for %s", port)
    return transport, label


async def main(args):
    wifi_host = None
    wifi_password = args.wifi_password or _read_config_local_value("WEBREPL_PASS", "bark")

    if args.sim:
        transport = SimTransport()
        transport_label = "sim"
        logger.info("Using physics simulation")
    elif args.wifi:
        # WiFi to real hardware
        host_port = args.wifi.split(":")
        wifi_host = host_port[0]
        port = int(host_port[1]) if len(host_port) > 1 else 8266
        from webrepl_transport import WebReplTransport
        transport = WebReplTransport(host=wifi_host, port=port, password=wifi_password)
        transport_label = f"wifi:{wifi_host}"
        logger.info("Using WebREPL transport: %s:%d", wifi_host, port)
    else:
        # Auto-detect: USB serial → hybrid/custom/stock, fallback → sim
        serial_port = args.serial if args.serial else find_serial_port()
        if serial_port:
            logger.info("Found serial port: %s", serial_port)
            transport, transport_label = await _detect_serial_transport(serial_port)
        else:
            transport = SimTransport()
            transport_label = "sim"
            logger.info("No hardware found, using physics simulation")

    dog = DogComms(transport)
    web_dir = os.path.join(os.path.dirname(__file__), "..", "web")
    web_dir = os.path.abspath(web_dir)
    server = Server(dog, web_dir, transport=transport, transport_label=transport_label,
                    wifi_host=wifi_host, wifi_password=wifi_password,
                    no_mdns=args.no_mdns,
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
    parser.add_argument("--sim", action="store_true",
                        help="Force PyBullet simulation (default if no hardware found)")
    parser.add_argument("--serial", default=None,
                        help="Serial port (e.g. /dev/cu.usbserial-10). Auto-detected if omitted.")
    parser.add_argument("--wifi", default=None,
                        help="WiFi address (e.g. 192.168.1.163)")
    parser.add_argument("--wifi-password", default=None,
                        help="WebREPL password (default: WEBREPL_PASS from config_local.h, or 'bark')")
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
