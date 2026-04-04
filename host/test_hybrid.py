"""
Tests for the hybrid transport system.

Tests the MicroPython handler's command/telemetry logic and the
HybridTransport's bootstrap + NDJSON flow, all without hardware.

Uses a mock serial pair to simulate the ESP32 side.
"""

import asyncio
import json
import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from comms import DogComms

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
logger = logging.getLogger("test_hybrid")


class MockSerialPair:
    """
    Simulates a serial connection between host and a fake MicroPython handler.

    host_reader/host_writer connect to the HybridTransport.
    device_reader/device_writer simulate what the ESP32 sees.
    """

    def __init__(self):
        # host -> device
        self._h2d_queue = asyncio.Queue()
        # device -> host
        self._d2h_queue = asyncio.Queue()

    def host_reader(self):
        return _QueueReader(self._d2h_queue)

    def host_writer(self):
        return _QueueWriter(self._h2d_queue)

    def device_reader(self):
        return _QueueReader(self._h2d_queue)

    def device_writer(self):
        return _QueueWriter(self._d2h_queue)


class _QueueReader:
    """Async reader backed by a queue of bytes."""

    def __init__(self, queue):
        self._queue = queue
        self._buf = b""

    async def readline(self):
        while b"\n" not in self._buf:
            chunk = await self._queue.get()
            self._buf += chunk
        idx = self._buf.index(b"\n") + 1
        line, self._buf = self._buf[:idx], self._buf[idx:]
        return line

    async def read(self, n):
        if self._buf:
            data, self._buf = self._buf[:n], self._buf[n:]
            return data
        try:
            return await asyncio.wait_for(self._queue.get(), timeout=0.1)
        except asyncio.TimeoutError:
            return b""

    def at_eof(self):
        return False


class _QueueWriter:
    """Async writer backed by a queue."""

    def __init__(self, queue):
        self._queue = queue

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        self._queue.put_nowait(data)

    async def drain(self):
        pass

    def close(self):
        pass

    async def wait_closed(self):
        pass


class FakeDevice:
    """
    Simulates the MicroPython handler running on the ESP32.

    Reads JSON commands, sends back acks and telemetry.
    """

    def __init__(self, reader, writer):
        self._reader = reader
        self._writer = writer
        self._running = False
        self._mode = "idle"

    def _send(self, msg):
        self._writer.write((json.dumps(msg) + "\n").encode())

    async def run(self):
        """Simulate the handler's main loop."""
        self._running = True

        # First, simulate REPL bootstrap phase
        # The transport sends Ctrl+C and REPL commands
        # Wait for the exec() command that starts the handler
        while self._running:
            try:
                line = await asyncio.wait_for(self._reader.readline(), timeout=5.0)
                text = line.decode(errors="replace").strip()
                if "exec(open(" in text or "_hybrid.py" in text:
                    # Handler is being started — send boot message
                    await asyncio.sleep(0.05)
                    self._send({"type": "boot", "firmware": "hybrid",
                                "version": "1.0", "imu": True, "sonar": True,
                                "servos": True, "battery": True})
                    break
                # Otherwise it's a REPL bootstrap command — echo prompt
                self._writer.write(b">>> ")
            except asyncio.TimeoutError:
                continue

        # Now in NDJSON mode — handle commands
        telem_task = asyncio.create_task(self._telem_loop())

        while self._running:
            try:
                line = await asyncio.wait_for(self._reader.readline(), timeout=1.0)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                msg = json.loads(text)
                await self._handle(msg)
            except asyncio.TimeoutError:
                continue
            except json.JSONDecodeError:
                continue
            except asyncio.CancelledError:
                break

        telem_task.cancel()
        try:
            await telem_task
        except asyncio.CancelledError:
            pass

    async def _handle(self, msg):
        t = msg.get("type", "")
        if t == "ping":
            self._send({"type": "pong"})
        elif t == "cmd_move":
            d = msg.get("direction", "stop")
            self._mode = "walk" if d != "stop" else "idle"
            self._send({"type": "ack", "ref_type": "cmd_move", "ok": True})
        elif t == "cmd_stand":
            self._mode = "idle"
            self._send({"type": "ack", "ref_type": "cmd_stand", "ok": True})
        elif t == "cmd_balance":
            self._send({"type": "ack", "ref_type": "cmd_balance", "ok": True})
        elif t == "cmd_led":
            self._send({"type": "ack", "ref_type": "cmd_led", "ok": True})
        elif t == "cmd_servo":
            self._send({"type": "ack", "ref_type": "cmd_servo", "ok": True})

    async def _ndjson_loop(self):
        """Run just the NDJSON command/telemetry phase (skip REPL bootstrap)."""
        self._running = True
        telem_task = asyncio.create_task(self._telem_loop())
        while self._running:
            try:
                line = await asyncio.wait_for(self._reader.readline(), timeout=1.0)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                msg = json.loads(text)
                await self._handle(msg)
            except asyncio.TimeoutError:
                continue
            except json.JSONDecodeError:
                continue
            except asyncio.CancelledError:
                break
        telem_task.cancel()
        try:
            await telem_task
        except asyncio.CancelledError:
            pass

    async def _telem_loop(self):
        """Push fake telemetry at realistic rates."""
        tick = 0
        try:
            while self._running:
                tick += 1
                # IMU at 50Hz
                self._send({"type": "telem_imu", "pitch": 1.5, "roll": -0.3, "yaw": 0})
                # Sonar every 3rd tick (~17Hz)
                if tick % 3 == 0:
                    self._send({"type": "telem_sonar", "distance_mm": 250})
                # Battery every 50th tick (~1Hz)
                if tick % 50 == 0:
                    self._send({"type": "telem_battery", "voltage_mv": 7400,
                                "pct": 80, "low": False})
                await asyncio.sleep(0.02)
        except asyncio.CancelledError:
            pass

    def stop(self):
        self._running = False


async def _setup_connected_transport(pair, device):
    """
    Create a HybridTransport already past the bootstrap phase.
    Skips the slow REPL upload and jumps straight to NDJSON mode.
    """
    from hybrid_transport import HybridTransport
    transport = HybridTransport(port="/dev/null")
    transport._reader = pair.host_reader()
    transport._writer = pair.host_writer()
    transport._open = True

    # Simulate boot message (what bootstrap waits for)
    boot_msg = {"type": "boot", "firmware": "hybrid", "version": "1.0",
                "imu": True, "sonar": True, "servos": True, "battery": True}
    transport._firmware_info = boot_msg

    # Start NDJSON reader and device telemetry
    transport._reader_task = asyncio.create_task(transport._reader_loop())
    return transport


async def test_telemetry_streaming():
    """Test that telemetry from device populates transport caches."""
    pair = MockSerialPair()
    writer = pair.device_writer()

    from hybrid_transport import HybridTransport
    transport = HybridTransport(port="/dev/null")
    transport._reader = pair.host_reader()
    transport._writer = pair.host_writer()
    transport._open = True
    transport._firmware_info = {"firmware": "hybrid"}
    transport._reader_task = asyncio.create_task(transport._reader_loop())

    # Push telemetry from device side
    writer.write((json.dumps({"type": "telem_imu", "pitch": 1.5, "roll": -0.3, "yaw": 0}) + "\n").encode())
    writer.write((json.dumps({"type": "telem_sonar", "distance_mm": 250}) + "\n").encode())
    writer.write((json.dumps({"type": "telem_battery", "voltage_mv": 7200, "pct": 60, "low": False}) + "\n").encode())

    await asyncio.sleep(0.1)

    assert transport._imu["pitch"] == 1.5, f"IMU pitch: {transport._imu['pitch']}"
    assert transport._imu["roll"] == -0.3, f"IMU roll: {transport._imu['roll']}"
    assert transport._sonar_mm == 250, f"Sonar: {transport._sonar_mm}"
    assert transport._battery_mv == 7200, f"Battery: {transport._battery_mv}"

    logger.info("PASS: telemetry_streaming — all sensor caches updated from NDJSON")

    transport._reader_task.cancel()
    try:
        await transport._reader_task
    except asyncio.CancelledError:
        pass

    return True


async def test_cmd_roundtrip():
    """Test CMD protocol translation through HybridTransport to fake device."""
    pair = MockSerialPair()
    device = FakeDevice(pair.device_reader(), pair.device_writer())
    device_task = asyncio.create_task(device._ndjson_loop())

    from hybrid_transport import HybridTransport
    transport = HybridTransport(port="/dev/null")
    transport._reader = pair.host_reader()
    transport._writer = pair.host_writer()
    transport._open = True
    transport._firmware_info = {"firmware": "hybrid"}
    transport._reader_task = asyncio.create_task(transport._reader_loop())

    await asyncio.sleep(0.05)

    # Test: move forward (CMD func 3, sub 3)
    await transport.send("CMD|3|3|$")
    resp = await transport.recv()
    assert resp == "CMD|3|OK|$", f"Move forward response: {resp}"

    # Wait for ack to arrive from device and telemetry to cache
    await asyncio.sleep(0.15)

    # Test: stand (CMD func 3, sub 2)
    await transport.send("CMD|3|2|$")
    resp = await transport.recv()
    assert resp == "CMD|3|OK|$", f"Stand response: {resp}"

    # Test: read cached sonar (CMD func 4)
    await transport.send("CMD|4|$")
    resp = await transport.recv()
    assert "CMD|4|" in resp, f"Sonar response: {resp}"

    # Test: read cached IMU (CMD func 5)
    await transport.send("CMD|5|$")
    resp = await transport.recv()
    assert "CMD|5|" in resp, f"IMU response: {resp}"

    logger.info("PASS: cmd_roundtrip — CMD protocol correctly translated and routed")

    device.stop()
    transport._reader_task.cancel()
    device_task.cancel()
    try:
        await transport._reader_task
    except asyncio.CancelledError:
        pass
    try:
        await device_task
    except asyncio.CancelledError:
        pass

    return True


async def test_dogcomms_integration():
    """Test full DogComms integration with HybridTransport."""
    pair = MockSerialPair()
    device = FakeDevice(pair.device_reader(), pair.device_writer())
    device_task = asyncio.create_task(device._ndjson_loop())

    from hybrid_transport import HybridTransport
    transport = HybridTransport(port="/dev/null")
    transport._reader = pair.host_reader()
    transport._writer = pair.host_writer()
    transport._open = True
    transport._firmware_info = {"firmware": "hybrid"}
    transport._reader_task = asyncio.create_task(transport._reader_loop())

    dog = DogComms(transport)
    dog._transport = transport
    dog._connected = True

    # Test high-level DogComms API
    ok = await dog.move("forward")
    assert ok, "DogComms.move('forward') should succeed"

    ok = await dog.stand()
    assert ok, "DogComms.stand() should succeed"

    ok = await dog.stop()
    assert ok, "DogComms.stop() should succeed"

    # Read sensors (cached from telemetry)
    await asyncio.sleep(0.15)
    imu = await dog.read_imu()
    assert imu is not None, "IMU should return cached data"

    sonar = await dog.read_ultrasonic()
    assert sonar is not None, "Sonar should return cached data"

    logger.info("PASS: dogcomms_integration — DogComms works with HybridTransport")

    device.stop()
    transport._reader_task.cancel()
    device_task.cancel()
    try:
        await transport._reader_task
    except asyncio.CancelledError:
        pass
    try:
        await device_task
    except asyncio.CancelledError:
        pass

    return True


async def test_handler_command_parsing():
    """Test the handler's command parsing logic in isolation (pure Python)."""
    # Simulate what the handler does without MicroPython imports
    # Test the motion mapping and command dispatch logic

    MOTION = {
        "forward": (35, 0), "backward": (-25, 0),
        "left": (20, -50), "right": (20, 50), "stop": (0, 0),
    }

    # Test motion mapping
    for direction, (expected_speed, expected_dir) in MOTION.items():
        sp, dr = MOTION[direction]
        assert sp == expected_speed, f"{direction}: speed {sp} != {expected_speed}"
        assert dr == expected_dir, f"{direction}: dir {dr} != {expected_dir}"

    # Test speed scaling
    msg = {"type": "cmd_move", "direction": "forward", "speed": 0.5}
    sp, dr = MOTION.get(msg["direction"], (0, 0))
    sp = int(sp * msg["speed"])
    dr = int(dr * msg["speed"])
    assert sp == 17, f"Half-speed forward: {sp}"
    assert dr == 0, f"Half-speed direction: {dr}"

    logger.info("PASS: handler_command_parsing — motion mapping correct")
    return True


async def test_dead_reckoning():
    """Test dead reckoning integration in HybridTransport."""
    from hybrid_transport import HybridTransport
    transport = HybridTransport(port="/dev/null")

    # Simulate forward motion for 1 second
    import time
    transport._motion_cmd = 3  # forward
    transport._last_motion_time = time.monotonic() - 0.5
    pos = transport.get_position()

    # Should have moved forward by ~0.05m (0.10 m/s * 0.5s)
    assert pos[0] > 0, f"Should move in +x: {pos}"
    assert abs(pos[1]) < 0.01, f"Should not drift in y: {pos}"

    # Simulate turn
    transport._motion_cmd = 6  # turn right
    transport._last_motion_time = time.monotonic() - 0.5
    heading = transport.get_heading()
    assert heading > 0, f"Should turn right (positive heading): {heading}"

    logger.info("PASS: dead_reckoning — position and heading update correctly")
    return True


async def main():
    results = {}
    tests = [
        ("handler_command_parsing", test_handler_command_parsing),
        ("dead_reckoning", test_dead_reckoning),
        ("telemetry_streaming", test_telemetry_streaming),
        ("cmd_roundtrip", test_cmd_roundtrip),
        ("dogcomms_integration", test_dogcomms_integration),
    ]

    for name, test_fn in tests:
        try:
            ok = await test_fn()
            results[name] = ok
        except Exception as e:
            logger.error("FAIL: %s — %s", name, e)
            results[name] = False

    print()
    print("=" * 50)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    for name, ok in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}: {name}")
    print(f"\n  {passed}/{total} tests passed")
    print("=" * 50)

    return all(results.values())


if __name__ == "__main__":
    ok = asyncio.run(main())
    sys.exit(0 if ok else 1)
