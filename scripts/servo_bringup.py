"""
Standalone servo bring-up: connect via USB, stream firmware logs, sweep all 8 servos.

Usage:
    python scripts/servo_bringup.py [--port /dev/cu.usbserial-XXXX]

Sweeps each servo ±50 µs around its STANDING_POSE_US center, then disengages.
Streams every firmware frame to stdout throughout so it doubles as a log tail.
"""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "host"))

from dog.discover import find_serial_port
from dog.dog import Dog

# Mirrors firmware/include/config.h STANDING_POSE array (indices 0..7)
STANDING_POSE_US = [2096, 1621, 2170, 1611, 904, 1379, 830, 1389]

SERVO_NAMES = [
    "FL_hip", "FL_knee",
    "FR_hip", "FR_knee",
    "RL_hip", "RL_knee",
    "RR_hip", "RR_knee",
]

EXCURSION_US = 50
STEP_SLEEP_S = 0.5
INTER_SERVO_S = 1.0
ENGAGE_TIMEOUT_S = 4.0
MIN_BATTERY_MV = 6500

boot_event: asyncio.Event


def _ts() -> str:
    return f"{time.monotonic():.2f}"


def on_telem(msg: dict) -> None:
    t = msg.get("type", "?")
    # Print everything except high-frequency IMU to keep output readable
    if t == "telem_imu":
        return
    print(f"  [{_ts()}] {msg}")
    # Signal boot
    if t == "boot":
        boot_event.set()


async def wait_for_engaged(dog: Dog, timeout: float) -> bool:
    deadline = asyncio.get_event_loop().time() + timeout
    while asyncio.get_event_loop().time() < deadline:
        if dog.get_engaged() and not dog.get_ramping():
            return True
        if dog.get_battery_cutoff():
            return False
        await asyncio.sleep(0.1)
    return dog.get_engaged() and not dog.get_ramping()


async def sweep_servo(dog: Dog, index: int) -> bool:
    name = SERVO_NAMES[index]
    center = STANDING_POSE_US[index]
    steps = [center, center + EXCURSION_US, center, center - EXCURSION_US, center]
    print(f"\n--- servo {index} ({name})  center={center} µs ---")
    for pulse in steps:
        await dog.send_json({"type": "cmd_servo", "index": index, "pulse_us": pulse})
        ack = await dog.recv_ack("cmd_servo", timeout=2.0)
        if ack is None:
            print(f"  [servo {index}] TIMEOUT waiting for ack at {pulse} µs")
            return False
        if not ack.get("ok"):
            print(f"  [servo {index}] NACK: {ack.get('error')} at {pulse} µs")
            return False
        actual = ack.get("actual_us", "?")
        written = ack.get("written", None)
        written_str = "" if written is None else f"  written={'YES' if written else 'NO — LEDC dropped!'}"
        print(f"  [servo {index}] {pulse} µs → actual {actual} µs  ok{written_str}")
        await asyncio.sleep(STEP_SLEEP_S)
    return True


async def main(port: str) -> int:
    global boot_event
    boot_event = asyncio.Event()

    print(f"[bringup] connecting to {port}")
    dog = Dog(port=port)
    dog.set_telem_callback(on_telem)

    await dog.open()

    # Wait for boot frame
    print("[bringup] waiting for boot frame (3 s)...")
    try:
        await asyncio.wait_for(boot_event.wait(), timeout=3.0)
        print(f"[bringup] firmware: {dog.firmware_info}")
    except asyncio.TimeoutError:
        print("[bringup] no boot frame in 3 s — issuing ping, continuing anyway")
        await dog.send_json({"type": "ping"})
        await asyncio.sleep(0.5)

    # 1 s baseline telemetry stream
    await asyncio.sleep(1.0)

    batt = dog.get_battery_mv()
    print(f"\n[bringup] battery: {batt} mV  cutoff: {dog.get_battery_cutoff()}")
    print(f"[bringup] imu: {dog.get_imu()}")
    print(f"[bringup] engaged: {dog.get_engaged()}")

    if dog.get_battery_cutoff():
        print("[bringup] ABORT: battery cutoff latched — reboot robot to clear")
        await dog.close()
        return 1

    if batt < MIN_BATTERY_MV:
        print(f"[bringup] ABORT: battery {batt} mV < {MIN_BATTERY_MV} mV threshold")
        await dog.close()
        return 1

    # Engage
    print("\n[bringup] engaging...")
    await dog.send_json({"type": "cmd_engage", "enabled": True})
    ack = await dog.recv_ack("cmd_engage", timeout=ENGAGE_TIMEOUT_S)
    if ack is None:
        print("[bringup] ABORT: cmd_engage timed out")
        await dog.close()
        return 1
    if not ack.get("ok"):
        print(f"[bringup] ABORT: cmd_engage NACK — {ack.get('error')}")
        await dog.close()
        return 1

    print("[bringup] engage ack ok — waiting for ramp to complete...")
    if not await wait_for_engaged(dog, ENGAGE_TIMEOUT_S):
        print("[bringup] ABORT: never reached engaged+not-ramping state")
        await dog.close()
        return 1
    print("[bringup] engaged and ready")

    # Sweep all 8 servos
    failures = []
    for i in range(8):
        ok = await sweep_servo(dog, i)
        if not ok:
            failures.append(i)
        await asyncio.sleep(INTER_SERVO_S)

    # Disengage
    print("\n[bringup] disengaging...")
    await dog.send_json({"type": "cmd_engage", "enabled": False})
    ack = await dog.recv_ack("cmd_engage", timeout=3.0)
    if ack is None:
        print("[bringup] WARNING: disengage ack timeout (proceeding to close)")
    else:
        print(f"[bringup] disengage ack: ok={ack.get('ok')} error={ack.get('error')}")

    await asyncio.sleep(1.6)  # wait for 1.5 s PWM detach ramp
    await dog.close()

    if failures:
        print(f"\n[bringup] FAILED servos: {[SERVO_NAMES[i] for i in failures]}")
        return 1

    print("\n[bringup] all 8 servos OK")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Servo bring-up sweep")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("ERROR: no USB serial port found. Pass --port /dev/cu.usbserial-XXXX")
        sys.exit(1)

    sys.exit(asyncio.run(main(port)))
