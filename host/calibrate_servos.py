#!/usr/bin/env python3
"""
Servo calibration test suite for MechDog custom firmware.

Connects via serial, wakes servos, runs per-servo sweeps using the
firmware's cmd_calibrate protocol, collects IMU responses, and reports
which servos are functional.

Usage:
    python3 host/calibrate_servos.py [--port /dev/cu.usbserial-10] [--sweep-range 100]
"""

import argparse
import asyncio
import json
import logging
import sys
import time

logger = logging.getLogger(__name__)

STANDING = [2096, 1621, 2170, 1611, 904, 1389, 1379, 830]
PINS = [14, 13, 16, 17, 18, 8, 5, 7]
LABELS = ['FL_hip', 'FL_knee', 'FR_hip', 'FR_knee',
          'RL_hip', 'RL_knee', 'RR_hip', 'RR_knee']


async def run_calibration(port: str, sweep_range: int = 100, step_us: int = 10,
                          dwell_ms: int = 300, tilt_limit: float = 8.0):
    import serial_asyncio

    reader, writer = await serial_asyncio.open_serial_connection(
        url=port, baudrate=115200
    )

    results = []

    async def read_messages(timeout: float = 1.0):
        """Read all available JSON messages within timeout."""
        msgs = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = await asyncio.wait_for(reader.readline(), timeout=0.3)
                text = line.decode(errors='replace').strip()
                if text.startswith('{'):
                    msgs.append(json.loads(text))
            except asyncio.TimeoutError:
                break
            except json.JSONDecodeError:
                continue
        return msgs

    async def send_json(msg):
        writer.write((json.dumps(msg) + '\n').encode())
        await writer.drain()

    async def wait_for_type(msg_type, timeout=30.0):
        """Wait for a specific message type, collecting all cal data along the way."""
        cal_points = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = await asyncio.wait_for(reader.readline(), timeout=0.5)
                text = line.decode(errors='replace').strip()
                if not text.startswith('{'):
                    continue
                msg = json.loads(text)
                if msg.get('type') == 'telem_cal':
                    cal_points.append(msg)
                if msg.get('type') == msg_type:
                    return msg, cal_points
            except asyncio.TimeoutError:
                continue
            except json.JSONDecodeError:
                continue
        return None, cal_points

    # Step 1: Verify firmware is running
    print("Connecting to firmware...")
    await asyncio.sleep(2)
    await send_json({"type": "ping"})
    msgs = await read_messages(3.0)
    pong = any(m.get('type') == 'pong' for m in msgs)
    if not pong:
        print("ERROR: No pong from firmware. Is custom firmware flashed?")
        writer.close()
        return []
    print("  Firmware responding.")

    # Step 2: Wake servos via stand command
    print("Waking servos (stand)...")
    await send_json({"type": "cmd_stand"})
    await asyncio.sleep(3)

    # Check servos are active
    msgs = await read_messages(2.0)
    status = next((m for m in msgs if m.get('type') == 'telem_status'), None)
    if status and not status.get('servos'):
        print("  WARNING: servos not active, sending init...")
        # Try a servo write to force init
        await send_json({"type": "cmd_servo", "index": 0, "pulse_us": STANDING[0]})
        await asyncio.sleep(1)

    # Step 3: Read baseline IMU
    msgs = await read_messages(2.0)
    imu = next((m for m in msgs if m.get('type') == 'telem_imu'), None)
    if imu:
        print(f"  Baseline IMU: pitch={imu['pitch']:.1f} roll={imu['roll']:.1f}")
    else:
        print("  WARNING: could not read baseline IMU")

    # Step 4: Run calibration sweep for each servo
    print(f"\n{'Servo':8s} {'GPIO':5s} {'Label':10s} {'Points':7s} {'PitchΔ':8s} {'RollΔ':8s} {'Primary':8s} {'Status':10s}")
    print("-" * 68)

    for servo in range(8):
        pos = STANDING[servo]
        from_us = pos - sweep_range
        to_us = pos + sweep_range

        # Send calibration command
        await send_json({
            "type": "cmd_calibrate",
            "action": "sweep",
            "servo": servo,
            "from_us": from_us,
            "to_us": to_us,
            "step_us": step_us,
            "dwell_ms": dwell_ms,
            "tilt_limit": tilt_limit,
        })

        # Wait for cal_done or cal_abort
        expected_steps = (to_us - from_us) // step_us + 1
        timeout = expected_steps * (dwell_ms / 1000.0) + 5.0
        done_msg, cal_points = await wait_for_type('cal_done', timeout=timeout)
        if done_msg is None:
            # Check for abort
            _, abort_points = await wait_for_type('cal_abort', timeout=2.0)
            cal_points.extend(abort_points)

        # Analyze results
        if not cal_points:
            status = "NO DATA"
            best_dp = best_dr = 0.0
            axis = "?"
        else:
            baseline_p = cal_points[0]['pitch']
            baseline_r = cal_points[0]['roll']
            best_dp = max(abs(p['pitch'] - baseline_p) for p in cal_points)
            best_dr = max(abs(p['roll'] - baseline_r) for p in cal_points)
            axis = 'pitch' if best_dp > best_dr else 'roll'
            moved = best_dp > 0.3 or best_dr > 0.3
            aborted = done_msg is None
            status = 'OK' if moved else 'WEAK'
            if aborted:
                status += ' ABORT'

        print(f"{servo:8d} {PINS[servo]:5d} {LABELS[servo]:10s} {len(cal_points):5d}   "
              f"{best_dp:6.2f}°  {best_dr:6.2f}°  {axis:8s} {status}")

        results.append({
            'servo': servo,
            'pin': PINS[servo],
            'label': LABELS[servo],
            'points': len(cal_points),
            'max_pitch_delta': round(best_dp, 3),
            'max_roll_delta': round(best_dr, 3),
            'primary_axis': axis,
            'moved': best_dp > 0.3 or best_dr > 0.3,
            'cal_data': cal_points,
        })

        # Brief settle between servos
        await asyncio.sleep(0.5)

    # Step 5: Return to standing
    await send_json({"type": "cmd_stand"})
    await asyncio.sleep(1)

    # Final IMU check
    msgs = await read_messages(1.0)
    imu = next((m for m in msgs if m.get('type') == 'telem_imu'), None)
    if imu:
        print(f"\nFinal IMU: pitch={imu['pitch']:.1f} roll={imu['roll']:.1f}")

    confirmed = sum(1 for r in results if r['moved'])
    print(f"Servos confirmed: {confirmed}/8")

    # Save results
    output_file = 'calibration_results.json'
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2, default=lambda x: None if isinstance(x, float) and x != x else x)
    print(f"Results saved to {output_file}")

    writer.close()
    return results


def main():
    parser = argparse.ArgumentParser(description="MechDog servo calibration")
    parser.add_argument("--port", default="/dev/cu.usbserial-10")
    parser.add_argument("--sweep-range", type=int, default=100,
                        help="Sweep ± this many microseconds from standing (default: 100)")
    parser.add_argument("--step-us", type=int, default=10)
    parser.add_argument("--dwell-ms", type=int, default=300)
    parser.add_argument("--tilt-limit", type=float, default=8.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    results = asyncio.run(run_calibration(
        args.port, args.sweep_range, args.step_us, args.dwell_ms, args.tilt_limit
    ))

    if not results:
        sys.exit(1)
    if not all(r['moved'] for r in results):
        failed = [r['label'] for r in results if not r['moved']]
        print(f"\nWARNING: {len(failed)} servo(s) not confirmed: {', '.join(failed)}")
        sys.exit(2)


if __name__ == "__main__":
    main()
