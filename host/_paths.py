"""
Path / hash helpers shared across host modules.

Centralises three things that previously appeared in two places each:
- firmware project root and built-binary path
- streaming SHA-256 of a file
- "host[:port]" parsing for --fw-tcp and friends
"""

from __future__ import annotations

import hashlib
import os


_HOST_DIR = os.path.dirname(os.path.abspath(__file__))


def firmware_dir() -> str:
    """Absolute path to the firmware/ directory."""
    return os.path.abspath(os.path.join(_HOST_DIR, "..", "firmware"))


def firmware_binary_path() -> str:
    """Absolute path to the .pio-built firmware binary."""
    return os.path.join(firmware_dir(), ".pio", "build", "mechdog", "firmware.bin")


def compute_sha256(path: str) -> str:
    """Streaming SHA-256 hex digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_fw_tcp(fw_tcp: str, default_port: int = 9000) -> tuple[str, int]:
    """Parse a "host[:port]" string from --fw-tcp.

    Raises ValueError if the port portion is non-numeric so callers can
    surface a clear error to the user.
    """
    if ":" in fw_tcp:
        host, port_str = fw_tcp.rsplit(":", 1)
        return host, int(port_str)
    return fw_tcp, default_port
