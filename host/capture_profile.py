#!/usr/bin/env python3
"""
Capture IMU + servo profile from real MechDog during a gait sequence.

Records timestamped telemetry at max rate into a JSON file that can be
analyzed by the C++ profile_analyzer.

Usage:
    python3 host/capture_profile.py --serial /dev/cu.usbserial-210 -o profile.json
    python3 host/capture_profile.py --wifi 192.168.1.163 -o profile.json
"""

import argparse
import asyncio
import json
import logging
import sys
import time

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger(__name__)


async def capture(transport_type, address, output_file, duration, gait_sequence):
    """Connect to dog, run gait sequence, capture telemetry."""

    # Create transport
    if transport_type == "serial":
        from repl_transport import ReplTransport
        transport = ReplTransport(port=address)
    elif transport_type == "wifi":
        from webrepl_transport import WebReplTransport
        transport = WebReplTransport(host=address)
    else:
        print("Unknown transport type")
        return

    from comms import DogComms

    dog = DogComms(transport)
    await dog.connect()
    logger.info("Connected to dog via %s", transport_type)

    samples = []
    start_time = time.monotonic()

    async def record_sample():
        """Read IMU and record a sample."""
        imu = await dog.read_imu()
        if not imu:
            return
        t = time.monotonic() - start_time
        sample = {
            "t": round(t, 4),
            "pitch": imu.get("pitch", 0),
            "roll": imu.get("roll", 0),
            "gz": 0,  # gyro Z not available via stock REPL
            "angles": [],  # servo angles not available via stock REPL
        }
        samples.append(sample)

    # Pre-capture: record standing baseline (2 seconds)
    logger.info("Recording standing baseline (2s)...")
    baseline_end = time.monotonic() + 2.0
    while time.monotonic() < baseline_end:
        await record_sample()
        await asyncio.sleep(0.05)

    # Execute gait sequence
    for cmd in gait_sequence:
        action = cmd.get("action", "stop")
        cmd_duration = cmd.get("duration", 2.0)
        logger.info("Executing: %s for %.1fs", action, cmd_duration)

        if action == "forward":
            await dog.move_forward()
        elif action == "backward":
            await dog.move_backward()
        elif action == "left":
            await dog.turn_left()
        elif action == "right":
            await dog.turn_right()
        elif action == "stop":
            await dog.stop()
        elif action == "stand":
            await dog.stand()

        # Record during this command
        phase_end = time.monotonic() + cmd_duration
        while time.monotonic() < phase_end:
            await record_sample()
            await asyncio.sleep(0.05)

    # Stop and record settling (2 seconds)
    await dog.stop()
    logger.info("Recording settling (2s)...")
    settle_end = time.monotonic() + 2.0
    while time.monotonic() < settle_end:
        await record_sample()
        await asyncio.sleep(0.05)

    await dog.disconnect()

    # Write output
    logger.info("Captured %d samples over %.1fs", len(samples),
                samples[-1]["t"] if samples else 0)

    with open(output_file, "w") as f:
        for sample in samples:
            f.write(json.dumps(sample) + "\n")

    logger.info("Saved to %s", output_file)


# Default gait sequence for profiling
DEFAULT_SEQUENCE = [
    {"action": "forward", "duration": 3.0},
    {"action": "stop", "duration": 1.0},
    {"action": "backward", "duration": 2.0},
    {"action": "stop", "duration": 1.0},
    {"action": "left", "duration": 2.0},
    {"action": "stop", "duration": 1.0},
    {"action": "right", "duration": 2.0},
    {"action": "stop", "duration": 1.0},
    {"action": "forward", "duration": 3.0},
    {"action": "stop", "duration": 1.0},
]


def main():
    parser = argparse.ArgumentParser(description="Capture MechDog IMU profile")
    parser.add_argument("--serial", help="USB serial port")
    parser.add_argument("--wifi", help="WiFi IP address")
    parser.add_argument("-o", "--output", default="profile.json",
                        help="Output file (default: profile.json)")
    parser.add_argument("-d", "--duration", type=float, default=None,
                        help="Override total duration (uses default sequence if not set)")
    parser.add_argument("--sequence", help="Custom sequence JSON file")
    args = parser.parse_args()

    if not args.serial and not args.wifi:
        print("Specify --serial or --wifi")
        sys.exit(1)

    transport_type = "serial" if args.serial else "wifi"
    address = args.serial or args.wifi

    if args.sequence:
        with open(args.sequence) as f:
            sequence = json.load(f)
    elif args.duration:
        sequence = [
            {"action": "forward", "duration": args.duration},
            {"action": "stop", "duration": 1.0},
        ]
    else:
        sequence = DEFAULT_SEQUENCE

    asyncio.run(capture(transport_type, address, args.output, args.duration, sequence))


if __name__ == "__main__":
    main()
