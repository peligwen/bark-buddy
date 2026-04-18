"""
Transport ABC for Bark-Buddy.

Subclasses implement the connection (USB serial, WiFi TCP, mock).
All communication uses JSON directly — no CMD text protocol.
"""

import asyncio
from abc import ABC, abstractmethod
from typing import Optional

SERIAL_BAUD = 115200
READ_TIMEOUT = 0.1

# Valid direction strings accepted by cmd_move
DIRECTIONS = frozenset({"forward", "backward", "left", "right", "stop"})


class Transport(ABC):

    @abstractmethod
    async def open(self) -> None: ...

    @abstractmethod
    async def close(self) -> None: ...

    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    async def send_json(self, msg: dict) -> None: ...

    # Telemetry accessors — return cached values from the streaming reader

    def get_imu(self) -> dict:
        return {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}

    def get_sonar_mm(self) -> int:
        return 0

    def get_battery_mv(self) -> int:
        return 7400

    def get_fw_version(self) -> str:
        return ""

    @property
    def firmware_info(self) -> dict:
        return {}

    def get_position(self) -> tuple:
        return (0.0, 0.0)

    def get_heading(self) -> float:
        return 0.0

    def get_joint_states(self) -> list:
        return [1500.0] * 8

    async def recv_ack(self, ref_type: str, timeout: float = 2.0) -> Optional[dict]:
        return None

    def set_ack_callback(self, cb) -> None:
        pass

    def set_telem_callback(self, cb) -> None:
        pass
