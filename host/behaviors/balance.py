"""
Balance monitoring layer.

Reads IMU from transport cache; detects falls; emits events.
Balance enable/disable is forwarded to firmware via cmd_balance — firmware owns the servo loop.
"""

import logging
import time

logger = logging.getLogger(__name__)

FALL_PITCH_DEG = 35.0
FALL_ROLL_DEG = 35.0
RECOVER_PITCH_DEG = 20.0
RECOVER_ROLL_DEG = 20.0


class BalanceLayer:

    def __init__(self, transport):
        self._transport = transport
        self._balance_enabled = False
        self._fallen = False
        self._fall_time: float = 0.0

    @property
    def enabled(self) -> bool:
        return self._balance_enabled

    async def set_enabled(self, enabled: bool) -> None:
        self._balance_enabled = enabled
        await self._transport.send_json({"type": "cmd_balance", "enabled": enabled})

    def check(self) -> dict:
        """
        Read cached IMU, detect fall state changes.
        Returns event dict with keys: fallen (bool), recovered (bool), imu (dict).
        Call from server telemetry loop.
        """
        imu = self._transport.get_imu()
        pitch = abs(imu.get("pitch", 0))
        roll = abs(imu.get("roll", 0))
        event = {"fallen": False, "recovered": False, "imu": imu}

        if not self._fallen:
            if pitch > FALL_PITCH_DEG or roll > FALL_ROLL_DEG:
                self._fallen = True
                self._fall_time = time.monotonic()
                event["fallen"] = True
                logger.warning("BalanceLayer: fall detected pitch=%.1f roll=%.1f", pitch, roll)
        else:
            if pitch < RECOVER_PITCH_DEG and roll < RECOVER_ROLL_DEG:
                self._fallen = False
                event["recovered"] = True
                logger.info("BalanceLayer: recovered after %.1fs", time.monotonic() - self._fall_time)

        return event

    @property
    def is_fallen(self) -> bool:
        return self._fallen
