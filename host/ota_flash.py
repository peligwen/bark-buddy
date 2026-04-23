"""
WiFi OTA flash orchestration.

Provides flash_wifi() -- a standalone async function that builds firmware and
flashes it to the MechDog over WiFi without needing the full bark server.

Usage (via bark_cli.py):
    asyncio.run(flash_wifi(host=None))   # mDNS discovery
    asyncio.run(flash_wifi(host="192.168.1.42"))  # direct
"""

import asyncio
import hashlib
import logging
import os
import sys
from pathlib import Path

from owner_key import sign_ota

# Ensure host/ is importable (same pattern as bark_cli.py _ensure_host_importable)
_HOST_DIR = str(Path(__file__).resolve().parent)
if _HOST_DIR not in sys.path:
    sys.path.insert(0, _HOST_DIR)

logger = logging.getLogger(__name__)

# Hard timeout (seconds) waiting for OTA status after sending the command
_OTA_WATCH_TIMEOUT = 120


def _firmware_binary_path() -> str:
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "firmware",
        ".pio", "build", "mechdog", "firmware.bin"
    ))


def _compute_sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


async def flash_wifi(host: str | None, tcp_port: int = 9000) -> int:
    """Orchestrate a full WiFi OTA firmware flash.

    Steps:
      1. Resolve target via mDNS (if host is None)
      2. Build firmware with pio run
      3. Compute SHA-256 of firmware binary
      4. Open Dog TCP connection
      5. Spin up a transient HTTP server on local_ip:0
      6. Send cmd_ota_update
      7. Watch ota_status messages until complete/failed/timeout
      8. Cleanup

    Returns 0 on success, 1 on any failure.
    """
    # ------------------------------------------------------------------
    # 1. Resolve target
    # ------------------------------------------------------------------
    fw_host = host
    fw_port = tcp_port

    if fw_host is None:
        print("[ota] No host specified -- browsing mDNS for _mechdog._tcp.local. (5 s)...")
        found_event = asyncio.Event()
        discovered: dict = {}

        async def _on_found(ip: str, port: int, props: dict) -> None:
            if not found_event.is_set():
                discovered["ip"] = ip
                discovered["port"] = port
                found_event.set()

        async def _on_lost(ip: str) -> None:
            pass

        from mdns_browser import MdnsBrowser
        browser = MdnsBrowser(on_found=_on_found, on_lost=_on_lost)
        await browser.start()
        try:
            await asyncio.wait_for(found_event.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            pass
        finally:
            await browser.stop()

        if not discovered:
            print("[ota] ERROR: No MechDog found on the network via mDNS.")
            print("[ota]        Make sure the robot is powered on and connected to WiFi,")
            print("[ota]        then try again, or pass --fw-tcp HOST:PORT explicitly.")
            return 1

        fw_host = discovered["ip"]
        fw_port = discovered.get("port", tcp_port)
        print(f"[ota] Found MechDog at {fw_host}:{fw_port}")

    # ------------------------------------------------------------------
    # 2. Build firmware
    # ------------------------------------------------------------------
    firmware_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "firmware")
    )
    print(f"[ota] Building firmware in {firmware_dir} ...")

    try:
        proc = await asyncio.create_subprocess_exec(
            "pio", "run",
            cwd=firmware_dir,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        # Stream output line-by-line
        assert proc.stdout is not None
        try:
            async with asyncio.timeout(180):
                async for raw_line in proc.stdout:
                    line = raw_line.decode(errors="replace").rstrip()
                    print(line)
                await proc.wait()
        except asyncio.TimeoutError:
            proc.kill()
            print("[ota] ERROR: Build timed out after 180 s.")
            return 1
    except FileNotFoundError:
        print("[ota] ERROR: 'pio' not found in PATH. Install PlatformIO first.")
        return 1

    if proc.returncode is None or proc.returncode != 0:
        print("[ota] ERROR: Build failed (pio run returned non-zero).")
        return 1

    # ------------------------------------------------------------------
    # 3. Compute SHA-256
    # ------------------------------------------------------------------
    binary_path = _firmware_binary_path()
    if not os.path.exists(binary_path):
        print(f"[ota] ERROR: Expected firmware binary not found: {binary_path}")
        return 1

    sha256_hex = _compute_sha256(binary_path)
    binary_size = os.path.getsize(binary_path)
    print(f"[ota] Firmware binary: {binary_size:,} bytes, sha256={sha256_hex[:16]}...")

    # ------------------------------------------------------------------
    # 4. Open Dog
    # ------------------------------------------------------------------
    from dog import Dog

    done_event = asyncio.Event()
    ota_result: dict = {"ok": False}

    def _telem_cb(msg: dict) -> None:
        msg_type = msg.get("type", "")

        if msg_type == "ota_status":
            status = msg.get("status", "")
            error = msg.get("error", "")
            if status == "downloading":
                print("[ota] downloading...")
            elif status == "flashing":
                print("[ota] flashing...")
            elif status == "complete":
                print("[ota] complete -- firmware updated, device rebooting.")
                ota_result["ok"] = True
                done_event.set()
            elif status == "failed":
                print(f"[ota] failed: {error}")
                ota_result["ok"] = False
                done_event.set()
            else:
                print(f"[ota] status={status!r} error={error!r}")

        elif msg_type == "ack" and msg.get("ref_type") == "cmd_ota_update":
            if not msg.get("ok"):
                error = msg.get("error", "unknown")
                if error == "sig":
                    print("[ota] ERROR: firmware rejected OTA — this device is owned by a different host.")
                    print("[ota]        Re-flash via USB to take ownership:  bark flash --usb")
                else:
                    print(f"[ota] ERROR: firmware rejected cmd_ota_update: {error}")
                ota_result["ok"] = False
                done_event.set()

    transport = Dog(host=fw_host, tcp_port=fw_port)
    transport.set_telem_callback(_telem_cb)

    print(f"[ota] Connecting to firmware at {fw_host}:{fw_port} ...")
    try:
        await transport.open()
    except Exception as exc:
        print(f"[ota] ERROR: Could not connect to firmware: {exc}")
        return 1

    runner = None
    exit_code = 1
    try:
        # ------------------------------------------------------------------
        # 5. Determine outbound IP
        # ------------------------------------------------------------------
        local_ip: str = transport._writer.get_extra_info("sockname")[0]
        print(f"[ota] Local IP (as seen by firmware): {local_ip}")

        # ------------------------------------------------------------------
        # 6. Spin up transient HTTP server
        # ------------------------------------------------------------------
        from aiohttp import web as aio_web

        async def _serve_binary(request: aio_web.Request) -> aio_web.Response:
            logger.debug("OTA HTTP: serving %s", binary_path)
            return aio_web.FileResponse(
                binary_path,
                headers={"Content-Type": "application/octet-stream"},
            )

        app = aio_web.Application()
        app.router.add_get("/firmware.bin", _serve_binary)

        runner = aio_web.AppRunner(app)
        await runner.setup()
        site = aio_web.TCPSite(runner, local_ip, 0)
        await site.start()

        http_port: int = site._server.sockets[0].getsockname()[1]  # type: ignore[union-attr]
        firmware_url = f"http://{local_ip}:{http_port}/firmware.bin"
        print(f"[ota] HTTP server ready at {firmware_url}")

        # ------------------------------------------------------------------
        # 7. Request nonce, sign, send authenticated OTA command
        # ------------------------------------------------------------------
        print("[ota] Requesting OTA nonce ...")
        nonce_event: asyncio.Event = asyncio.Event()
        nonce_holder: dict = {}

        def _nonce_cb(msg: dict) -> None:
            if msg.get("type") == "telem_ota_nonce":
                nonce_holder["nonce"] = msg.get("nonce", "")
                nonce_event.set()
            else:
                _telem_cb(msg)

        transport.set_telem_callback(_nonce_cb)
        await transport.send_json({"type": "cmd_ota_request_nonce"})
        try:
            await asyncio.wait_for(nonce_event.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            print("[ota] ERROR: timed out waiting for nonce from device.")
            return 1
        finally:
            transport.set_telem_callback(_telem_cb)

        nonce_hex = nonce_holder.get("nonce", "")
        if len(nonce_hex) != 64:
            print(f"[ota] ERROR: invalid nonce from device: {nonce_hex!r}")
            return 1

        print("[ota] Signing OTA command ...")
        sig_hex = sign_ota(nonce_hex, sha256_hex)

        print("[ota] Sending cmd_ota_update ...")
        await transport.send_json({
            "type":   "cmd_ota_update",
            "url":    firmware_url,
            "sha256": sha256_hex,
            "nonce":  nonce_hex,
            "sig":    sig_hex,
        })

        # ------------------------------------------------------------------
        # 8. Watch ota_status with hard timeout
        # ------------------------------------------------------------------
        print(f"[ota] Waiting for OTA result (timeout {_OTA_WATCH_TIMEOUT} s) ...")
        try:
            await asyncio.wait_for(done_event.wait(), timeout=_OTA_WATCH_TIMEOUT)
        except asyncio.TimeoutError:
            print(f"[ota] ERROR: OTA timed out after {_OTA_WATCH_TIMEOUT} s -- no completion signal received.")
            ota_result["ok"] = False

        exit_code = 0 if ota_result.get("ok") else 1

    finally:
        # ------------------------------------------------------------------
        # 10. Cleanup
        # ------------------------------------------------------------------
        if runner is not None:
            try:
                await runner.cleanup()
            except Exception:
                pass

        try:
            await transport.close()
        except Exception:
            # On successful OTA the firmware reboots and drops TCP -- expected
            pass

    return exit_code
