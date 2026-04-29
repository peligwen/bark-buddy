"""bark — Bark-Buddy CLI entry point."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
HOST_DIR = PROJECT_ROOT / "host"
FIRMWARE_DIR = PROJECT_ROOT / "firmware"
STOCK_FIRMWARE_BIN = FIRMWARE_DIR / "stock_firmware.bin"


def _ensure_host_importable():
    """Add host/ to sys.path so server.py's bare imports work."""
    host = str(HOST_DIR)
    if host not in sys.path:
        sys.path.insert(0, host)


def _add_server_flags(parser):
    parser.add_argument("--host", default="0.0.0.0",
                        help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8456,
                        help="HTTP port (default: 8456)")
    parser.add_argument("--serial", default=None,
                        help="Serial port override (e.g. /dev/cu.usbserial-10)")
    parser.add_argument("--fw-tcp", default=None, metavar="HOST[:PORT]",
                        help="Connect to firmware over TCP (e.g. 127.0.0.1:9001)")
    parser.add_argument("--no-mdns", action="store_true",
                        help="Disable mDNS auto-discovery")
    parser.add_argument("--no-browser", action="store_true",
                        help="Don't auto-open browser")


def cmd_serve(args):
    _ensure_host_importable()
    import asyncio
    args.restart = False
    args.open_browser = not getattr(args, "no_browser", False)
    from server import main as server_main
    asyncio.run(server_main(args))


def cmd_mock(args):
    import socket
    import time
    _ensure_host_importable()
    mock_bin = FIRMWARE_DIR / "test" / "bark-mock"
    if not mock_bin.exists():
        subprocess.run(
            ["make", "-C", str(FIRMWARE_DIR / "test"), "bark-mock"],
            check=True
        )
    proc = subprocess.Popen([str(mock_bin), "--tcp-port", "9001"])
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                s = socket.create_connection(("127.0.0.1", 9001), timeout=0.2)
                s.close()
                break
            except OSError:
                time.sleep(0.1)
        else:
            proc.terminate()
            raise RuntimeError("bark-mock did not open port 9001 within 5 s")
        args.fw_tcp = "127.0.0.1:9001"
        cmd_serve(args)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


def _do_stock_flash(args):
    if not STOCK_FIRMWARE_BIN.exists():
        print(f"[flash stock] ERROR: {STOCK_FIRMWARE_BIN} not found.")
        print("[flash stock] Expected Hiwonder Mechdog_250505_0x000.bin (~1.7 MB) at that path.")
        sys.exit(1)
    _ensure_host_importable()
    from server import find_serial_port
    port = getattr(args, "serial", None) or find_serial_port()
    if not port:
        print("[flash stock] ERROR: no USB serial port found. "
              "Pass --serial /dev/cu.usbserial-XXXX.")
        sys.exit(1)
    baud = getattr(args, "baud", 460800)
    cmd = [
        sys.executable, "-m", "esptool",
        "--chip", "esp32",
        "--port", port,
        "--baud", str(baud),
        "write_flash",
        "--flash_mode", "dio",
        "--flash_size", "4MB",
        "0x0", str(STOCK_FIRMWARE_BIN),
    ]
    print(f"[flash stock] Writing {STOCK_FIRMWARE_BIN.name} "
          f"({STOCK_FIRMWARE_BIN.stat().st_size} bytes) @ 0x0 on {port} ...")
    print("[flash stock] Saved actions (LittleFS/vfs) not included; servos will still engage.")
    print("[flash stock] To recover custom firmware afterwards: bark flash")
    result = subprocess.run(cmd)
    sys.exit(result.returncode)


def _do_usb_flash():
    _ensure_host_importable()
    from owner_key import ensure_owner_key
    ensure_owner_key()
    result = subprocess.run(
        ["pio", "run", "-t", "upload"],
        cwd=FIRMWARE_DIR,
    )
    sys.exit(result.returncode)


def _do_wifi_flash(args):
    import asyncio
    _ensure_host_importable()
    fw_tcp = getattr(args, 'fw_tcp', None)
    host = None
    tcp_port = 9000
    if fw_tcp:
        from _paths import parse_fw_tcp
        try:
            host, tcp_port = parse_fw_tcp(fw_tcp)
        except ValueError:
            print(f"[flash] ERROR: invalid port in --fw-tcp '{fw_tcp}'")
            sys.exit(1)
    from ota_flash import flash_wifi
    rc = asyncio.run(flash_wifi(host=host, tcp_port=tcp_port))
    sys.exit(rc)


def cmd_flash(args):
    if getattr(args, "flash_target", None) == "stock":
        _do_stock_flash(args)
        return
    # --fw-tcp implies wifi
    if getattr(args, 'fw_tcp', None):
        _do_wifi_flash(args)
        return

    use_usb = getattr(args, 'usb', False)
    use_wifi = getattr(args, 'wifi', False)

    if use_usb:
        _do_usb_flash()
    elif use_wifi:
        _do_wifi_flash(args)
    else:
        # Auto-detect: USB first
        _ensure_host_importable()
        from dog.discover import find_serial_port
        if find_serial_port():
            _do_usb_flash()
        else:
            print("[flash] No USB serial port found -- trying WiFi OTA ...")
            _do_wifi_flash(args)


def cmd_test(args):
    test_dir = FIRMWARE_DIR / "test"
    result = subprocess.run(["make", "-C", str(test_dir)])
    if result.returncode != 0:
        sys.exit(result.returncode)
    # Run each test binary
    targets = [
        "test_ik", "test_transform", "test_balance",
        "test_offsets", "test_gait_ik", "test_servos",
        "test_battery_monitor", "test_gait_tilt_fault",
    ]
    failed = []
    ran = 0
    for name in targets:
        binary = test_dir / name
        if binary.exists():
            ran += 1
            r = subprocess.run([str(binary)], cwd=test_dir)
            if r.returncode != 0:
                failed.append(name)
        else:
            print(f"WARNING: {name} binary not found, skipping")
    if failed:
        print(f"\n{len(failed)} test(s) FAILED: {', '.join(failed)}")
        sys.exit(1)
    if ran < len(targets):
        print(f"\nWARNING: only {ran}/{len(targets)} test(s) ran (some binaries missing)")
        sys.exit(1)
    print(f"\nAll {ran} test(s) passed.")


def cmd_kill(args):
    """Kill a running bark-buddy server by port."""
    import os
    import signal
    import subprocess
    import time

    port = args.port
    result = subprocess.run(
        ["lsof", "-ti", f":{port}"],
        capture_output=True, text=True,
    )
    pids = [int(p) for p in result.stdout.split() if p.strip()]
    if not pids:
        print(f"No server found on port {port}")
        return

    for pid in pids:
        print(f"Killing PID {pid} (port {port})...")
        os.kill(pid, signal.SIGTERM)

    time.sleep(1)
    for pid in pids:
        try:
            os.kill(pid, 0)
            print(f"PID {pid} still alive, sending SIGKILL...")
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    print("Done.")




def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    parser = argparse.ArgumentParser(
        prog="bark",
        description="Bark-Buddy robot dog CLI",
    )
    sub = parser.add_subparsers(dest="command")

    # Default (no subcommand) — start server
    _add_server_flags(parser)

    # bark mock
    p_mock = sub.add_parser("mock", help="Start server with mock firmware (compiled C++)")
    _add_server_flags(p_mock)

    # bark flash
    p_flash = sub.add_parser("flash", help="Build + upload firmware via PlatformIO")
    _flash_group = p_flash.add_mutually_exclusive_group()
    _flash_group.add_argument("--usb", action="store_true",
                              help="Force USB upload via PlatformIO")
    _flash_group.add_argument("--wifi", action="store_true",
                              help="Force WiFi OTA flash")
    _flash_group.add_argument("--fw-tcp", default=None, metavar="HOST[:PORT]",
                              help="Target firmware TCP address (implies --wifi; default port 9000)")
    flash_targets = p_flash.add_subparsers(dest="flash_target")
    p_flash_stock = flash_targets.add_parser(
        "stock",
        help="Flash Hiwonder vendor stock firmware via esptool (servo sanity check)",
    )
    p_flash_stock.add_argument("--serial", default=None,
                               help="Serial port override (e.g. /dev/cu.usbserial-10)")
    p_flash_stock.add_argument("--baud", type=int, default=460800,
                               help="esptool baud rate (default: 460800)")

    # bark test
    sub.add_parser("test", help="Build and run firmware native tests")

    # bark kill
    p_kill = sub.add_parser("kill", help="Stop a running bark-buddy server")
    p_kill.add_argument("--port", type=int, default=8456,
                        help="Port the server is listening on (default: 8456)")

    args = parser.parse_args()

    if args.command == "flash":
        cmd_flash(args)
    elif args.command == "test":
        cmd_test(args)
    elif args.command == "kill":
        cmd_kill(args)
    elif args.command == "mock":
        cmd_mock(args)
    else:
        # no subcommand — start server with auto-detect
        cmd_serve(args)


if __name__ == "__main__":
    main()
