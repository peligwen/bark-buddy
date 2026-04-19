"""Auto-detect MechDog devices: USB serial glob and serial probe."""

import asyncio
import glob
import logging

logger = logging.getLogger(__name__)


def find_serial_port() -> str | None:
    """Return the first matching USB serial port, or None."""
    ports = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyUSB*")
    return ports[0] if ports else None


def _probe_serial_sync(port: str) -> bool:
    """Blocking probe for custom firmware — runs in a thread via asyncio.to_thread.

    Returns True if JSON pong detected (custom firmware).
    """
    import serial
    import time
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(3.0)
        resp = ""
        for _ in range(3):
            ser.write(b'{"type":"ping"}\n')
            time.sleep(0.5)
            resp += ser.read(ser.in_waiting).decode(errors="replace")
            if '"pong"' in resp:
                break
        ser.close()
        return '"pong"' in resp
    except Exception as e:
        logger.warning("Serial probe failed on %s: %s", port, e)
        return False


async def detect_serial_dog(port: str):
    """Probe a serial port for custom firmware.

    Returns (Dog, label) on success. Raises ConnectionError if not found.
    """
    from .dog import Dog
    logger.info("Probing %s for custom firmware...", port)
    found = await asyncio.to_thread(_probe_serial_sync, port)
    if not found:
        raise ConnectionError(f"No custom firmware response on {port}")
    dog = Dog(port=port)
    label = f"fw:{port.split('/')[-1]}"
    logger.info("Detected custom firmware on %s", port)
    return dog, label
