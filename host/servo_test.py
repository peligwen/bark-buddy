#!/usr/bin/env python3
"""
Servo testing tool for MechDog.

Enters test mode (frail by default), keeps a heartbeat alive, and provides
a clean interface for probing individual servos with IMU feedback.

Features:
- Auto-wakes servos on every command (no more idle timeout issues)
- Frail mode: range-clamped, slew-rate limited, duty-cycle protected
- Logs every command + IMU response to NDJSON for analysis
- Returns to standing on exit or Ctrl+C

Usage:
    python3 servo_test.py                         # interactive probe
    python3 servo_test.py --probe 6               # probe RR_hip around standing
    python3 servo_test.py --probe 6 --range 150   # ±150μs sweep
    python3 servo_test.py --sweep 6               # full calibration sweep
    python3 servo_test.py --all                   # probe all 8 servos
    python3 servo_test.py --no-frail              # disable frail mode (careful!)
    python3 servo_test.py --standing              # just stand and report IMU
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
    """Manages test mode session over serial JSON protocol."""

    def __init__(self, port: str, baudrate: int = 115200, frail: bool = True):
        self._port = port
        self._baudrate = baudrate
        self._frail = frail
        self._reader = None
        self._writer = None
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._last_ack = None
        self._reader_task = None
        self._heartbeat_task = None
        self._log_file = None
        self._log_path = None

    async def connect(self):
        """Open serial and enter test mode."""
        import serial_asyncio
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._port, baudrate=self._baudrate
        )

        # DTR reset to get clean state
        transport = self._writer.transport
        serial_obj = transport.serial if hasattr(transport, 'serial') else None
        if serial_obj and hasattr(serial_obj, 'dtr'):
            serial_obj.dtr = False
            await asyncio.sleep(0.1)
            serial_obj.dtr = True
            await asyncio.sleep(3.0)  # wait for boot

        # Drain boot output
        await self._drain()

        # Open log file
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._log_path = f"servo_test_{ts}.ndjson"
        self._log_file = open(self._log_path, "w")
        logger.info("Logging to %s", self._log_path)

        # Start background reader
        self._reader_task = asyncio.create_task(self._read_loop())

        # Enter test mode
        await self._send({"type": "cmd_test_mode", "enable": True, "frail": self._frail})
        await asyncio.sleep(0.5)
        logger.info("Test mode active (frail=%s)", self._frail)

        # Start heartbeat
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    async def disconnect(self):
        """Exit test mode and clean up."""
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        # Return to standing
        try:
            await self._send({"type": "cmd_test_mode", "enable": False})
            await asyncio.sleep(1.0)
        except Exception:
            pass

        if self._reader_task:
            self._reader_task.cancel()
            try:
                await self._reader_task
            except asyncio.CancelledError:
                pass

        if self._writer:
            self._writer.close()

        if self._log_file:
            self._log_file.close()
            logger.info("Log saved: %s", self._log_path)

    async def read_imu(self) -> dict:
        """Get current IMU reading (from background reader cache)."""
        return dict(self._imu)

    async def set_servo(self, index: int, pulse_us: int) -> dict:
        """Set a servo and return the ack with readback."""
        self._last_ack = None
        await self._send({"type": "cmd_servo", "index": index, "pulse_us": pulse_us})

        # Wait for ack
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self._last_ack and self._last_ack.get("ref_type") == "cmd_servo":
                return self._last_ack
            await asyncio.sleep(0.02)

        return {"ok": False, "error": "timeout"}

    async def probe_servo(self, index: int, range_us: int = 100,
                          step_us: int = 10, dwell_ms: int = 500) -> list[dict]:
        """Sweep a single servo around its standing pose, recording IMU at each step.

        Returns list of {pulse_us, pitch, roll, actual_us} dicts.
        """
        standing = STANDING_POSE[index]
        results = []

        # Return to standing first
        await self.set_servo(index, standing)
        await asyncio.sleep(0.5)

        values = list(range(standing - range_us, standing + range_us + 1, step_us))
        logger.info("Probing %s (servo %d): %d→%d μs, %d steps, %dms dwell",
                     SERVO_NAMES[index], index, values[0], values[-1],
                     len(values), dwell_ms)

        baseline = await self.read_imu()
        logger.info("Baseline IMU: pitch=%.1f roll=%.1f", baseline["pitch"], baseline["roll"])

        for us in values:
            ack = await self.set_servo(index, us)
            await asyncio.sleep(dwell_ms / 1000.0)
            imu = await self.read_imu()

            actual = ack.get("actual_us", us)
            result = {
                "servo": index,
                "name": SERVO_NAMES[index],
                "pulse_us": us,
                "actual_us": actual,
                "pitch": imu["pitch"],
                "roll": imu["roll"],
                "d_pitch": round(imu["pitch"] - baseline["pitch"], 2),
                "d_roll": round(imu["roll"] - baseline["roll"], 2),
            }
            results.append(result)
            self._log({"event": "probe", **result})

            logger.info("  %4dμs (actual=%4d)  pitch=%6.1f (%+.1f)  roll=%6.1f (%+.1f)",
                         us, actual, imu["pitch"], result["d_pitch"],
                         imu["roll"], result["d_roll"])

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

    async def _send(self, msg: dict):
        """Send a JSON message."""
        line = json.dumps(msg) + "\n"
        self._writer.write(line.encode())
        await self._writer.drain()
        self._log({"event": "send", **msg})

    async def _read_loop(self):
        """Background reader — updates IMU cache, captures acks."""
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

                msg_type = msg.get("type", "")

                if msg_type == "telem_imu":
                    self._imu["pitch"] = msg.get("pitch", 0)
                    self._imu["roll"] = msg.get("roll", 0)
                    self._imu["yaw"] = msg.get("yaw", 0)

                elif msg_type == "ack":
                    self._last_ack = msg
                    ref = msg.get("ref_type", "")
                    if not msg.get("ok"):
                        logger.warning("NACK: %s — %s", ref, msg.get("error", "?"))

        except asyncio.CancelledError:
            pass

    async def _heartbeat_loop(self):
        """Send pings to keep test mode alive."""
        try:
            while True:
                await asyncio.sleep(3.0)
                await self._send({"type": "ping"})
        except asyncio.CancelledError:
            pass

    async def _drain(self):
        """Read and discard any buffered serial data."""
        try:
            while True:
                data = await asyncio.wait_for(self._reader.read(1024), timeout=0.2)
                if not data:
                    break
        except asyncio.TimeoutError:
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
  python3 servo_test.py --no-frail --probe 0        # disable frail (careful!)

Servo map:
  0=FL_hip  1=FL_knee  2=FR_hip  3=FR_knee
  4=RL_hip  5=RL_knee  6=RR_hip  7=RR_knee
        """,
    )
    parser.add_argument("--port", default=None, help="Serial port (auto-detected)")
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
    parser.add_argument("--no-frail", action="store_true",
                        help="Disable frail mode (full servo authority)")

    args = parser.parse_args()

    port = args.port or find_serial_port()
    if not port:
        print("No USB serial device found")
        sys.exit(1)

    tester = ServoTester(port=port, frail=not args.no_frail)

    # Handle Ctrl+C gracefully
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(_shutdown(tester)))

    try:
        await tester.connect()

        if args.standing:
            await tester.standing_report()

        elif args.probe is not None:
            await tester.probe_servo(args.probe, range_us=args.range,
                                     step_us=args.step, dwell_ms=args.dwell)

        elif args.sweep is not None:
            # Full sweep: wider range, finer steps
            await tester.probe_servo(args.sweep, range_us=200,
                                     step_us=5, dwell_ms=300)

        elif args.all:
            for idx in range(8):
                logger.info("")
                await tester.probe_servo(idx, range_us=args.range,
                                         step_us=args.step, dwell_ms=args.dwell)
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
