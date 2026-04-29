"""OTA firmware update: build, binary serve, cmd_ota_update orchestration."""

import asyncio
import hashlib
import logging
import os
import re

from aiohttp import web

logger = logging.getLogger(__name__)


def _firmware_dir() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "firmware"))


def _binary_path() -> str:
    return os.path.abspath(os.path.join(
        _firmware_dir(), ".pio", "build", "mechdog", "firmware.bin"
    ))


def _compute_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def read_available_fw_version() -> str:
    """Parse FW_VERSION from firmware/include/config.h."""
    config_path = os.path.join(_firmware_dir(), "include", "config.h")
    try:
        with open(config_path) as f:
            m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', f.read())
            return m.group(1) if m else ""
    except FileNotFoundError:
        return ""


class OtaManager:
    """Handles firmware OTA: build, binary serve, and OTA command dispatch."""

    def __init__(self, get_transport_fn, get_transport_label_fn):
        """
        get_transport_fn: callable -> current Dog instance (or None)
        get_transport_label_fn: callable -> current transport label string
        """
        self._get_transport = get_transport_fn
        self._get_label = get_transport_label_fn
        self._binary_sha256: str | None = None
        self.available_version = read_available_fw_version()

    def add_routes(self, router) -> None:
        router.add_get("/api/firmware/status",  self.handle_status)
        router.add_post("/api/firmware/build",  self.handle_build)
        router.add_get("/api/firmware/binary",  self.handle_binary)
        router.add_post("/api/firmware/update", self.handle_update)

    async def _do_build(self) -> dict:
        """Run pio build and return result dict."""
        firmware_dir = _firmware_dir()
        try:
            # Use create_subprocess_exec (not shell) -- args passed as list, no injection risk
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
        binary_path = _binary_path()
        binary_ready = ok and os.path.exists(binary_path)
        if binary_ready:
            self._binary_sha256 = _compute_sha256(binary_path)
        return {
            "ok": ok,
            "output": output[-3000:],
            "binary_ready": binary_ready,
        }

    async def handle_status(self, request: web.Request) -> web.Response:
        transport = self._get_transport()
        label = self._get_label()
        current = transport.get_fw_version() if transport else ""
        is_wifi = "fw:" in label and "/dev/" not in label
        path = _binary_path()
        binary_exists = os.path.exists(path)
        return web.json_response({
            "current_version": current,
            "available_version": self.available_version,
            "update_available": bool(current and self.available_version and
                                     current != self.available_version),
            "transport": label,
            "can_ota": is_wifi,
            "binary_ready": binary_exists,
            "sha256": self._binary_sha256 if binary_exists else None,
        })

    async def handle_build(self, request: web.Request) -> web.Response:
        return web.json_response(await self._do_build())

    async def handle_binary(self, request: web.Request) -> web.Response:
        path = _binary_path()
        if not os.path.exists(path):
            return web.json_response(
                {"error": "No firmware binary. Run /api/firmware/build first."}, status=404
            )
        return web.FileResponse(path, headers={
            "Content-Type": "application/octet-stream",
            "Content-Disposition": "attachment; filename=firmware.bin",
        })

    async def handle_update(self, request: web.Request) -> web.Response:
        build = await self._do_build()
        if not build.get("ok"):
            return web.json_response(
                {"ok": False, "error": "Build failed", "output": build.get("output")},
                status=500,
            )
        host_parts = request.host.split(":")
        host_ip = host_parts[0]
        host_port = host_parts[1] if len(host_parts) > 1 else "8080"
        binary_url = f"http://{host_ip}:{host_port}/api/firmware/binary"
        sha256_hex = _compute_sha256(_binary_path())
        self._binary_sha256 = sha256_hex

        transport = self._get_transport()
        if not transport:
            return web.json_response({"ok": False, "error": "No firmware transport"}, status=400)
        try:
            await transport.send_json({
                "type": "cmd_ota_update",
                "url": binary_url,
                "sha256": sha256_hex,
            })
        except Exception as e:
            return web.json_response(
                {"ok": False, "error": f"Failed to send OTA command: {e}"}, status=500
            )
        # Wait briefly for the firmware ack so the UI hears about an immediate
        # rejection (wifi_disabled, missing_auth, sig). Successful flashes
        # take much longer and report progress via ota_status.
        ack = await transport.recv_ack("cmd_ota_update", timeout=10.0)
        if ack is None:
            return web.json_response(
                {"ok": False, "error": "Timed out waiting for firmware ack"}, status=504,
            )
        if not ack.get("ok"):
            return web.json_response(
                {"ok": False, "error": ack.get("error", "rejected"),
                 "ack": ack},
                status=400,
            )
        return web.json_response({
            "ok": True,
            "binary_url": binary_url,
            "new_version": self.available_version,
            "sha256": sha256_hex,
        })
