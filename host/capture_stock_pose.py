#!/usr/bin/env python3
"""
Servo mapping and standing pose discovery for MechDog.

Works with the custom firmware (JSON protocol). Systematically probes
each servo to determine:
1. Which physical joint each servo index controls
2. The IMU response axis and direction (pitch vs roll, polarity)
3. The correct standing pulse width for level stance

Method:
- Start from center crouch (all 1500μs, body on ground)
- Rise gradually to a test pose
- Probe each servo ±offset from test pose, record IMU deltas
- Classify each servo by primary axis response
- Search for the standing pose that minimizes total tilt

Usage:
    python3 capture_stock_pose.py                # full mapping
    python3 capture_stock_pose.py --servo 6      # single servo
    python3 capture_stock_pose.py --find-level    # find level standing pose
"""

import argparse
import asyncio
import json
import os
import signal
import sys
import time

import serial_asyncio

SERVO_NAMES = [
    "FL_hip", "FL_knee", "FR_hip", "FR_knee",
    "RL_hip", "RL_knee", "RR_hip", "RR_knee",
]

# Current config.h standing pose (known to be wrong for RR)
STANDING_POSE = [2096, 1621, 2170, 1611, 904, 1379, 1389, 830]


class FirmwareConnection:
    """Low-level JSON serial connection to custom firmware."""

    def __init__(self, port, baudrate=115200):
        self._port = port
        self._baudrate = baudrate
        self._reader = None
        self._writer = None
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._last_ack = None
        self._read_task = None

    async def connect(self):
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )
        # DTR reset
        so = self._writer.transport.serial
        so.dtr = False
        await asyncio.sleep(0.1)
        so.dtr = True
        await asyncio.sleep(3.0)
        self._read_task = asyncio.create_task(self._read_loop())
        # Wait for first IMU reading
        await asyncio.sleep(1.0)

    async def close(self):
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self._writer:
            self._writer.close()

    def send(self, msg):
        self._writer.write((json.dumps(msg) + "\n").encode())

    async def read_imu(self, settle_ms=100):
        """Read IMU after settling."""
        await asyncio.sleep(settle_ms / 1000.0)
        return dict(self._imu)

    async def set_servo(self, index, pulse_us):
        self._last_ack = None
        self.send({"type": "cmd_servo", "index": index, "pulse_us": pulse_us})
        # Wait for ack briefly
        for _ in range(20):
            if self._last_ack:
                return self._last_ack
            await asyncio.sleep(0.01)
        return None

    async def set_all_servos(self, values, reps=5, delay=0.02):
        """Set all 8 servos, repeating to work with frail slew rate."""
        for _ in range(reps):
            for i, us in enumerate(values):
                self.send({"type": "cmd_servo", "index": i, "pulse_us": us})
            await asyncio.sleep(delay)

    async def enter_test_mode(self, frail=True):
        self.send({"type": "cmd_test_mode", "enable": True, "frail": frail})
        await asyncio.sleep(0.5)

    async def exit_test_mode(self):
        self.send({"type": "cmd_test_mode", "enable": False})
        await asyncio.sleep(1.0)

    def ping(self):
        self.send({"type": "ping"})

    async def _read_loop(self):
        try:
            while True:
                line = await self._reader.readline()
                text = line.decode(errors="replace").strip()
                if not text.startswith("{"):
                    continue
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    continue
                if msg.get("type") == "telem_imu":
                    self._imu["pitch"] = msg.get("pitch", 0)
                    self._imu["roll"] = msg.get("roll", 0)
                    self._imu["yaw"] = msg.get("yaw", 0)
                elif msg.get("type") == "ack":
                    self._last_ack = msg
        except asyncio.CancelledError:
            pass


async def gradual_move(conn, start, target, steps=50, settle_ms=100):
    """Move from start pose to target pose gradually. Returns final IMU."""
    for step in range(1, steps + 1):
        t = step / steps
        pose = [int(s + (e - s) * t) for s, e in zip(start, target)]
        await conn.set_all_servos(pose, reps=3, delay=0.02)
        await asyncio.sleep(0.02)
        if step % 10 == 0:
            conn.ping()
    await asyncio.sleep(settle_ms / 1000.0)
    return await conn.read_imu(settle_ms=50)


async def probe_servo(conn, base_pose, servo_idx, offsets, dwell_ms=100):
    """Probe one servo at various offsets from base_pose. Returns results."""
    results = []
    standing = base_pose[servo_idx]

    for offset in offsets:
        us = standing + offset
        if us < 500 or us > 2500:
            continue

        pose = list(base_pose)
        pose[servo_idx] = us
        await conn.set_all_servos(pose, reps=8, delay=0.02)
        conn.ping()
        imu = await conn.read_imu(settle_ms=dwell_ms)

        results.append({
            "servo": servo_idx,
            "name": SERVO_NAMES[servo_idx],
            "pulse_us": us,
            "offset": offset,
            "pitch": imu["pitch"],
            "roll": imu["roll"],
        })

    return results


async def map_all_servos(conn, args):
    """Full servo mapping: classify each servo by IMU response."""
    print("=== Servo Mapping ===\n")

    # Enter test mode
    await conn.enter_test_mode(frail=True)

    # Start from center crouch
    center = [1500] * 8
    print("Moving to center crouch...")
    await conn.set_all_servos(center, reps=60, delay=0.05)
    await asyncio.sleep(0.5)
    center_imu = await conn.read_imu()
    print(f"Center IMU: pitch={center_imu['pitch']:.1f}  roll={center_imu['roll']:.1f}\n")

    # Probe offsets (relative to center)
    offsets = list(range(-150, 151, 25))

    all_results = {}

    for idx in range(8):
        print(f"--- Servo {idx} ({SERVO_NAMES[idx]}) ---")
        results = await probe_servo(conn, center, idx, offsets, dwell_ms=args.dwell)

        if not results:
            print("  No results\n")
            continue

        # Find response range
        pitches = [r["pitch"] for r in results]
        rolls = [r["roll"] for r in results]
        p_range = max(pitches) - min(pitches)
        r_range = max(rolls) - min(rolls)

        # Determine primary axis
        primary = "pitch" if p_range > r_range else "roll"
        response = p_range if primary == "pitch" else r_range
        moved = response > 1.0

        # Determine polarity: does increasing μs increase or decrease primary axis?
        neg_vals = [r[primary] for r in results if r["offset"] < 0]
        pos_vals = [r[primary] for r in results if r["offset"] > 0]
        if neg_vals and pos_vals:
            polarity = "+" if sum(pos_vals) / len(pos_vals) > sum(neg_vals) / len(neg_vals) else "-"
        else:
            polarity = "?"

        all_results[idx] = {
            "name": SERVO_NAMES[idx],
            "primary_axis": primary,
            "polarity": polarity,
            "pitch_range": round(p_range, 2),
            "roll_range": round(r_range, 2),
            "moved": moved,
            "readings": results,
        }

        print(f"  Primary: {primary} ({polarity})  "
              f"Δpitch={p_range:.1f}°  Δroll={r_range:.1f}°  "
              f"{'MOVED' if moved else '--- no response'}")

        # Print condensed readings
        for r in results:
            marker = " *" if abs(r["offset"]) == 0 else ""
            print(f"    {r['pulse_us']:4d}μs ({r['offset']:+4d})  "
                  f"pitch={r['pitch']:6.1f}  roll={r['roll']:6.1f}{marker}")
        print()

        # Return to center between servos
        await conn.set_all_servos(center, reps=15, delay=0.03)
        await asyncio.sleep(0.3)

    # Summary
    print("\n=== Summary ===")
    print(f"{'Idx':>3} {'Label':>10} {'Axis':>6} {'Dir':>4} {'ΔPitch':>7} {'ΔRoll':>7} {'Moved':>6}")
    print("-" * 52)
    for idx in range(8):
        r = all_results.get(idx)
        if r:
            print(f"{idx:3d} {r['name']:>10} {r['primary_axis']:>6} {r['polarity']:>4} "
                  f"{r['pitch_range']:7.1f} {r['roll_range']:7.1f} {'YES' if r['moved'] else 'no':>6}")

    # Save results
    out_path = "servo_mapping.json"
    # Strip readings for compact output
    summary = {}
    for idx, r in all_results.items():
        summary[str(idx)] = {k: v for k, v in r.items() if k != "readings"}
    with open(out_path, "w") as f:
        json.dump({"mapping": summary, "center_imu": center_imu,
                    "standing_pose_tested": STANDING_POSE}, f, indent=2)
    print(f"\nSaved to {out_path}")

    await conn.exit_test_mode()
    return all_results


async def find_level(conn, args):
    """Search for the standing pose that minimizes tilt.

    Starts from the current STANDING_POSE, then iteratively adjusts
    each servo to reduce pitch and roll toward zero.
    """
    print("=== Find Level Standing Pose ===\n")
    await conn.enter_test_mode(frail=True)

    # Start from center, rise to current pose
    center = [1500] * 8
    current = list(STANDING_POSE)

    print("Rising from center to current standing pose...")
    await conn.set_all_servos(center, reps=60, delay=0.05)
    await asyncio.sleep(0.5)

    final_imu = await gradual_move(conn, center, current, steps=50, settle_ms=200)
    print(f"At standing: pitch={final_imu['pitch']:.1f}  roll={final_imu['roll']:.1f}")

    tilt = (final_imu["pitch"] ** 2 + final_imu["roll"] ** 2) ** 0.5
    print(f"Total tilt: {tilt:.1f}°\n")

    if tilt > 10:
        print("WARNING: tilt > 10° — probably on 3 legs. Aborting.")
        print("Fix the standing pose first (use --map to identify issues).")
        await conn.exit_test_mode()
        return

    # Iterative adjustment: for each servo, try ±step and keep the better value
    step_sizes = [50, 25, 10]
    best_pose = list(current)

    for step in step_sizes:
        print(f"--- Step size: ±{step}μs ---")
        improved = True
        passes = 0
        while improved and passes < 3:
            improved = False
            passes += 1
            for idx in range(8):
                imu_at = await conn.read_imu(settle_ms=100)
                current_tilt = (imu_at["pitch"] ** 2 + imu_at["roll"] ** 2) ** 0.5

                best_tilt = current_tilt
                best_val = best_pose[idx]

                for delta in [-step, +step]:
                    test_val = best_pose[idx] + delta
                    if test_val < 500 or test_val > 2500:
                        continue
                    test_pose = list(best_pose)
                    test_pose[idx] = test_val
                    await conn.set_all_servos(test_pose, reps=8, delay=0.02)
                    conn.ping()
                    test_imu = await conn.read_imu(settle_ms=args.dwell)
                    test_tilt = (test_imu["pitch"] ** 2 + test_imu["roll"] ** 2) ** 0.5

                    if test_tilt < best_tilt - 0.3:  # require meaningful improvement
                        best_tilt = test_tilt
                        best_val = test_val

                if best_val != best_pose[idx]:
                    old = best_pose[idx]
                    best_pose[idx] = best_val
                    await conn.set_all_servos(best_pose, reps=8, delay=0.02)
                    print(f"  {SERVO_NAMES[idx]:>10}: {old} → {best_val}  tilt: {best_tilt:.1f}°")
                    improved = True

        imu_now = await conn.read_imu(settle_ms=200)
        tilt_now = (imu_now["pitch"] ** 2 + imu_now["roll"] ** 2) ** 0.5
        print(f"  After ±{step}: pitch={imu_now['pitch']:.1f}  "
              f"roll={imu_now['roll']:.1f}  tilt={tilt_now:.1f}°\n")

    # Final result
    print("=== Level Standing Pose ===")
    for i in range(8):
        changed = " (changed)" if best_pose[i] != STANDING_POSE[i] else ""
        print(f"  {SERVO_NAMES[i]:>10}: {best_pose[i]}{changed}")

    final = await conn.read_imu(settle_ms=300)
    final_tilt = (final["pitch"] ** 2 + final["roll"] ** 2) ** 0.5
    print(f"\nFinal IMU: pitch={final['pitch']:.1f}  roll={final['roll']:.1f}  tilt={final_tilt:.1f}°")

    # Save
    out_path = "level_pose.json"
    with open(out_path, "w") as f:
        json.dump({
            "pose": best_pose,
            "labels": SERVO_NAMES,
            "imu": final,
            "tilt": round(final_tilt, 2),
            "original_pose": STANDING_POSE,
        }, f, indent=2)
    print(f"Saved to {out_path}")

    # Print as C array for config.h
    print(f"\nFor config.h:")
    print(f"static const uint16_t STANDING_POSE[8] = {{")
    print(f"    {', '.join(str(v) for v in best_pose)}")
    print(f"    // {', '.join(SERVO_NAMES)}")
    print(f"}};")

    await conn.exit_test_mode()


async def walk_capture(conn, args):
    """Capture IMU during 1-second walks in each direction."""
    print("=== Walk Capture (1s per direction) ===\n")

    directions = [
        ("forward", {"type": "cmd_move", "direction": "forward", "speed": 0.5}),
        ("backward", {"type": "cmd_move", "direction": "backward", "speed": 0.5}),
        ("left", {"type": "cmd_move", "direction": "left", "speed": 0.5}),
        ("right", {"type": "cmd_move", "direction": "right", "speed": 0.5}),
    ]

    for name, cmd in directions:
        print(f"--- {name} ---")
        # Stand first
        conn.send({"type": "cmd_stand"})
        await asyncio.sleep(1.0)

        pre_imu = await conn.read_imu(settle_ms=100)
        print(f"  Before: pitch={pre_imu['pitch']:.1f}  roll={pre_imu['roll']:.1f}")

        # Walk 1 second
        conn.send(cmd)
        samples = []
        for _ in range(20):  # 20 samples over 1s
            await asyncio.sleep(0.05)
            imu = await conn.read_imu(settle_ms=0)
            samples.append(dict(imu))

        # Stop
        conn.send({"type": "cmd_move", "direction": "stop"})
        await asyncio.sleep(0.5)

        post_imu = await conn.read_imu(settle_ms=200)

        pitches = [s["pitch"] for s in samples]
        rolls = [s["roll"] for s in samples]
        print(f"  During: pitch=[{min(pitches):.1f}, {max(pitches):.1f}]  "
              f"roll=[{min(rolls):.1f}, {max(rolls):.1f}]")
        print(f"  After:  pitch={post_imu['pitch']:.1f}  roll={post_imu['roll']:.1f}")
        print()

    conn.send({"type": "cmd_stand"})
    await asyncio.sleep(1.0)


async def run(args):
    port = args.port
    if not port:
        import glob
        candidates = glob.glob("/dev/cu.usbserial-*")
        port = candidates[0] if candidates else None
    if not port:
        print("No USB serial device found")
        sys.exit(1)

    conn = FirmwareConnection(port)
    await conn.connect()

    try:
        if args.find_level:
            await find_level(conn, args)
        elif args.walk:
            await walk_capture(conn, args)
        elif args.servo is not None:
            await conn.enter_test_mode(frail=True)
            center = [1500] * 8
            await conn.set_all_servos(center, reps=60, delay=0.05)
            await asyncio.sleep(0.5)
            offsets = list(range(-150, 151, 25))
            results = await probe_servo(conn, center, args.servo, offsets, dwell_ms=args.dwell)
            for r in results:
                print(f"  {r['pulse_us']:4d}μs ({r['offset']:+4d})  "
                      f"pitch={r['pitch']:6.1f}  roll={r['roll']:6.1f}")
            await conn.exit_test_mode()
        else:
            await map_all_servos(conn, args)
    finally:
        await conn.close()


def main():
    parser = argparse.ArgumentParser(
        description="MechDog servo mapping and pose discovery")
    parser.add_argument("--port", default=None)
    parser.add_argument("--servo", type=int, default=None, metavar="IDX",
                        help="Probe a single servo (0-7)")
    parser.add_argument("--find-level", action="store_true",
                        help="Search for level standing pose")
    parser.add_argument("--walk", action="store_true",
                        help="Capture IMU during 1s walks in each direction")
    parser.add_argument("--dwell", type=int, default=100,
                        help="Dwell time in ms per step (default: 100)")
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
