"""bark — Bark-Buddy CLI entry point."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
HOST_DIR = PROJECT_ROOT / "host"
FIRMWARE_DIR = PROJECT_ROOT / "firmware"


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
    parser.add_argument("--wifi", default=None,
                        help="WiFi address override (e.g. 192.168.1.163)")
    parser.add_argument("--wifi-password", default=None,
                        help="WebREPL password")
    parser.add_argument("--no-mdns", action="store_true",
                        help="Disable mDNS auto-discovery")
    parser.add_argument("--no-browser", action="store_true",
                        help="Don't auto-open browser")


def cmd_serve(args):
    _ensure_host_importable()
    import asyncio
    # Patch args to match what server.main() expects
    args.restart = False
    args.open_browser = not getattr(args, "no_browser", False)
    from server import main as server_main
    asyncio.run(server_main(args))


def cmd_flash(args):
    result = subprocess.run(
        ["pio", "run", "-t", "upload"],
        cwd=FIRMWARE_DIR,
    )
    sys.exit(result.returncode)


def cmd_test(args):
    test_dir = FIRMWARE_DIR / "test"
    result = subprocess.run(["make", "-C", str(test_dir)])
    if result.returncode != 0:
        sys.exit(result.returncode)
    # Run each test binary
    targets = [
        "test_ik", "test_transform", "test_balance",
        "test_offsets", "test_gait_ik", "test_servos", "test_lifecycle",
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


def cmd_wifi_setup(args):
    _ensure_host_importable()
    from setup_wifi import main as wifi_main
    wifi_main()


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
    parser.set_defaults(sim=False)

    # bark sim
    p_sim = sub.add_parser("sim", help="Start server in simulation mode")
    _add_server_flags(p_sim)
    p_sim.set_defaults(sim=True)

    # bark flash
    sub.add_parser("flash", help="Build + upload firmware via PlatformIO")

    # bark test
    sub.add_parser("test", help="Build and run firmware native tests")

    # bark kill
    p_kill = sub.add_parser("kill", help="Stop a running bark-buddy server")
    p_kill.add_argument("--port", type=int, default=8456,
                        help="Port the server is listening on (default: 8456)")

    # bark wifi-setup
    sub.add_parser("wifi-setup", help="Interactive WiFi + WebREPL setup")

    args = parser.parse_args()

    if args.command == "flash":
        cmd_flash(args)
    elif args.command == "test":
        cmd_test(args)
    elif args.command == "kill":
        cmd_kill(args)
    elif args.command == "wifi-setup":
        cmd_wifi_setup(args)
    else:
        # "sim" or no subcommand — both start the server
        cmd_serve(args)


if __name__ == "__main__":
    main()
