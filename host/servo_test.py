#!/usr/bin/env python3
"""
Servo testing tool for MechDog.

Enters test mode, keeps a heartbeat alive, and provides a clean interface
for probing individual servos with IMU feedback.

Features:
- Auto-wakes servos on every command (no more idle timeout issues)
- Logs every command + IMU response to NDJSON for analysis
- Returns to standing on exit or Ctrl+C

Usage:
    python3 servo_test.py                         # interactive probe
    python3 servo_test.py --probe 6               # probe RR_hip around standing
    python3 servo_test.py --probe 6 --range 150   # ±150μs sweep
    python3 servo_test.py --sweep 6               # full calibration sweep
    python3 servo_test.py --all                   # probe all 8 servos
    python3 servo_test.py --standing              # just stand and report IMU
    python3 servo_test.py --host 192.168.1.163    # connect over WiFi TCP
"""

import argparse
import asyncio
import json
import logging
import os
import signal
import sys
import time

# Ensure host/ is on path
sys.path.insert(0, os.path.dirname(__file__))

from firmware_transport import FirmwareTransport

logging.basicConfig(level=logging.INFO, format="%(name)s %(message)s")
logger = logging.getLogger("servo_test")

# Servo labels
SERVO_NAMES = [
    "FL_hip", "FL_knee", "FR_hip", "FR_knee",
    "RL_hip", "RL_knee", "RR_hip", "RR_knee",
]

# Standing pose from config.h
STANDING_POSE = [2096, 1621, 2170, 1611, 904, 1379, 1389, 830]


class ServoTester:
    """Manages test mode session using FirmwareTransport (serial or WiFi)."""

    def __init__(self, port: str = None, host: str = None):
        self._transport = FirmwareTransport(
            port=port, host=host, dtr_reset=(port is not None)
        )
        self._heartbeat_task = None
        self._log_file = None
        self._log_path = None

    async def connect(self):
        """Open transport and start heartbeat."""
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = f"servo_test_{ts}.ndjson"
        self._log_file = open(self._log_path, "w")
        logger.info("Logging to %s", self._log_path)

        await self._transport.open()
        logger.info("Connected")

        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    async def disconnect(self):
        """Clean up and close transport."""
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        await self._transport.close()

        if self._log_file:
            self._log_file.close()
            logger.info("Log saved: %s", self._log_path)

    async def read_imu(self) -> dict:
        """Get current IMU reading from the transport's telemetry cache."""
        return self._transport.get_imu()

    async def set_servo(self, index: int, pulse_us: int) -> dict:
        """Set a servo and return the ack with readback."""
        self._log({"event": "send", "type": "cmd_servo", "index": index, "pulse_us": pulse_us})
        await self._transport.send_json({"type": "cmd_servo", "index": index, "pulse_us": pulse_us})
        ack = await self._transport.recv_ack("cmd_servo", timeout=2.0)
        if ack is None:
            return {"ok": False, "error": "timeout"}
        return ack

    async def stand_up(self, level_limit: float = 15.0):
        """Send cmd_stand, wait for dog to rise, verify IMU is near-level.

        Aborts with SystemExit if IMU reads too tilted after standing — indicates
        the dog didn't actually stand up (e.g. a leg failed).
        """
        logger.info("Sending cmd_stand — watch the dog rise...")
        await self._transport.send_json({"type": "cmd_stand"})
        await asyncio.sleep(3.0)

        imu = await self.read_imu()
        pitch, roll = imu.get("pitch", 0.0), imu.get("roll", 0.0)
        logger.info("Post-stand IMU: pitch=%.1f  roll=%.1f", pitch, roll)
        if abs(pitch) > level_limit or abs(roll) > level_limit:
            logger.error("ABORT — dog is not level after cmd_stand "
                         "(pitch=%.1f roll=%.1f, limit=±%.1f°). "
                         "Check legs before proceeding.", pitch, roll, level_limit)
            raise SystemExit(1)
        logger.info("Stand OK — proceeding.")

    async def probe_servo(self, index: int, range_us: int = 100,
                          step_us: int = 10, dwell_ms: int = 500,
                          tilt_limit: float = 6.0,
                          jerk_limit: float = 2.0) -> list[dict]:
        """Sweep a single servo around its standing pose, recording IMU at each step.

        Aborts early if:
        - Total tilt delta from baseline exceeds tilt_limit
        - A single step causes a jerk > jerk_limit (sudden stall or impact)

        Returns list of {pulse_us, pitch, roll, actual_us} dicts.
        """
        standing = STANDING_POSE[index]
        results = []

        # Return to standing first
        await self.set_servo(index, standing)
        await asyncio.sleep(0.5)

        values = list(range(standing - range_us, standing + range_us + 1, step_us))
        logger.info("Probing %s (servo %d): %d→%d μs, %d steps, %dms dwell  "
                    "[tilt_limit=%.1f°  jerk_limit=%.1f°]",
                    SERVO_NAMES[index], index, values[0], values[-1],
                    len(values), dwell_ms, tilt_limit, jerk_limit)

        baseline = await self.read_imu()
        logger.info("Baseline IMU: pitch=%.1f roll=%.1f", baseline["pitch"], baseline["roll"])

        prev_pitch = baseline["pitch"]
        prev_roll = baseline["roll"]

        for us in values:
            ack = await self.set_servo(index, us)
            await asyncio.sleep(dwell_ms / 1000.0)
            imu = await self.read_imu()

            actual = ack.get("actual_us", us)
            d_pitch = round(imu["pitch"] - baseline["pitch"], 2)
            d_roll = round(imu["roll"] - baseline["roll"], 2)
            jerk_p = abs(imu["pitch"] - prev_pitch)
            jerk_r = abs(imu["roll"] - prev_roll)

            result = {
                "servo": index,
                "name": SERVO_NAMES[index],
                "pulse_us": us,
                "actual_us": actual,
                "pitch": imu["pitch"],
                "roll": imu["roll"],
                "d_pitch": d_pitch,
                "d_roll": d_roll,
            }
            results.append(result)
            self._log({"event": "probe", **result})

            logger.info("  %4dμs (actual=%4d)  pitch=%6.1f (%+.1f)  roll=%6.1f (%+.1f)",
                         us, actual, imu["pitch"], d_pitch, imu["roll"], d_roll)

            # Jerk check — sudden large change between steps
            if jerk_p > jerk_limit or jerk_r > jerk_limit:
                logger.warning("ABORT — jerk detected at %dμs: Δpitch=%.1f Δroll=%.1f "
                                "(limit=%.1f°) — possible stall or impact",
                                us, jerk_p, jerk_r, jerk_limit)
                await self.set_servo(index, standing)
                await asyncio.sleep(0.5)
                return results

            # Tilt accumulation check
            if abs(d_pitch) > tilt_limit or abs(d_roll) > tilt_limit:
                logger.warning("ABORT — tilt limit at %dμs: d_pitch=%.1f d_roll=%.1f "
                                "(limit=%.1f°)", us, d_pitch, d_roll, tilt_limit)
                await self.set_servo(index, standing)
                await asyncio.sleep(0.5)
                return results

            prev_pitch = imu["pitch"]
            prev_roll = imu["roll"]

        # Return to standing
        await self.set_servo(index, standing)
        await asyncio.sleep(0.3)

        # Summarize
        if results:
            pitches = [r["d_pitch"] for r in results]
            rolls = [r["d_roll"] for r in results]
            max_dp = max(pitches, key=abs)
            max_dr = max(rolls, key=abs)
            moved = abs(max_dp) > 1.0 or abs(max_dr) > 1.0
            axis = "pitch" if abs(max_dp) > abs(max_dr) else "roll"
            logger.info("  Summary: max Δpitch=%.1f  max Δroll=%.1f  axis=%s  moved=%s",
                         max_dp, max_dr, axis, moved)

        return results

    async def standing_report(self):
        """Just stand and report IMU for a few seconds."""
        logger.info("Standing pose — reading IMU...")
        for _ in range(10):
            await asyncio.sleep(0.5)
            imu = await self.read_imu()
            logger.info("  pitch=%6.1f  roll=%6.1f  yaw=%6.1f",
                         imu["pitch"], imu["roll"], imu.get("yaw", 0))

    # --- Internal ---

    async def _heartbeat_loop(self):
        """Send pings to keep test mode alive."""
        try:
            while True:
                await asyncio.sleep(3.0)
                await self._transport.send_json({"type": "ping"})
        except asyncio.CancelledError:
            pass

    def _log(self, entry: dict):
        """Write an entry to the NDJSON log."""
        if self._log_file:
            entry["t"] = round(time.time(), 3)
            self._log_file.write(json.dumps(entry) + "\n")
            self._log_file.flush()


def find_serial_port() -> str | None:
    """Find USB serial port."""
    import glob
    candidates = glob.glob("/dev/cu.usbserial-*")
    return candidates[0] if candidates else None


async def main():
    parser = argparse.ArgumentParser(
        description="MechDog servo testing tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 servo_test.py --standing                  # check standing pose IMU
  python3 servo_test.py --probe 6                   # probe RR_hip ±100μs
  python3 servo_test.py --probe 6 --range 150       # wider sweep
  python3 servo_test.py --probe 6 --step 5          # finer resolution
  python3 servo_test.py --all                       # probe all servos
  python3 servo_test.py --sweep 6                   # full calibration sweep
  python3 servo_test.py --host 192.168.1.163        # connect over WiFi

Servo map:
  0=FL_hip  1=FL_knee  2=FR_hip  3=FR_knee
  4=RL_hip  5=RL_knee  6=RR_hip  7=RR_knee
        """,
    )
    parser.add_argument("--port", default=None, help="Serial port (auto-detected)")
    parser.add_argument("--host", default=None, help="WiFi host (e.g. 192.168.1.163)")
    parser.add_argument("--probe", type=int, default=None, metavar="IDX",
                        help="Probe a single servo (0-7)")
    parser.add_argument("--sweep", type=int, default=None, metavar="IDX",
                        help="Full calibration sweep of a servo")
    parser.add_argument("--all", action="store_true", help="Probe all 8 servos")
    parser.add_argument("--standing", action="store_true", help="Stand and report IMU")
    parser.add_argument("--range", type=int, default=100,
                        help="Sweep range ±μs from standing (default: 100)")
    parser.add_argument("--step", type=int, default=10,
                        help="Step size in μs (default: 10)")
    parser.add_argument("--dwell", type=int, default=100,
                        help="Dwell time in ms per step (default: 100)")
    parser.add_argument("--tilt-limit", type=float, default=6.0,
                        help="Abort if pitch or roll delta exceeds this (degrees, default: 6.0)")
    parser.add_argument("--jerk-limit", type=float, default=2.0,
                        help="Abort if a single step changes IMU by more than this (degrees, default: 2.0)")
    args = parser.parse_args()

    port = args.port
    host = args.host
    if not port and not host:
        port = find_serial_port()
    if not port and not host:
        print("No USB serial device found and no --host specified")
        sys.exit(1)

    tester = ServoTester(port=port, host=host)

    # Handle Ctrl+C gracefully
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(_shutdown(tester)))

    try:
        await tester.connect()
        await tester.stand_up()

        if args.standing:
            await tester.standing_report()

        elif args.probe is not None:
            await tester.probe_servo(args.probe, range_us=args.range,
                                     step_us=args.step, dwell_ms=args.dwell,
                                     tilt_limit=args.tilt_limit,
                                     jerk_limit=args.jerk_limit)

        elif args.sweep is not None:
            # Full sweep: wider range, finer steps
            await tester.probe_servo(args.sweep, range_us=200,
                                     step_us=5, dwell_ms=300,
                                     tilt_limit=args.tilt_limit,
                                     jerk_limit=args.jerk_limit)

        elif args.all:
            for idx in range(8):
                logger.info("")
                await tester.probe_servo(idx, range_us=args.range,
                                         step_us=args.step, dwell_ms=args.dwell,
                                         tilt_limit=args.tilt_limit,
                                         jerk_limit=args.jerk_limit)
                # Rest between servos
                await asyncio.sleep(1.0)

        else:
            # Default: standing report
            await tester.standing_report()

    finally:
        await tester.disconnect()


async def _shutdown(tester: ServoTester):
    """Graceful shutdown on signal."""
    logger.info("Shutting down — returning to standing...")
    await tester.disconnect()
    sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
