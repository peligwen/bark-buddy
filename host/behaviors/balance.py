"""
Balance behavior layer.

Enables stock self-balance on connect and monitors IMU tilt for
fall detection. Runs as an always-on composable layer beneath
remote control or patrol.
"""

import asyncio
import logging
from typing import Callable, Optional

from comms import DogComms

logger = logging.getLogger(__name__)

# Tilt thresholds (degrees)
FALL_THRESHOLD = 35.0       # tilt beyond this = fallen
WARN_THRESHOLD = 20.0       # tilt beyond this = warning
RECOVERY_THRESHOLD = 10.0   # tilt below this after fall = recovered
FALL_DEBOUNCE = 0.5         # seconds of sustained tilt before declaring fall


class BalanceLayer:
    """
    Always-on balance layer.

    - Enables stock self-balance (CMD|1|3|1) on start
    - Polls IMU and tracks tilt state
    - Detects falls and triggers recovery (stand command)
    - Calls event callbacks for UI notifications
    """

    def __init__(self, dog: DogComms):
        self._dog = dog
        self._enabled = False
        self._fallen = False
        self._fall_start: float = 0.0
        self._on_fall: Optional[Callable] = None
        self._on_recovered: Optional[Callable] = None
        self._on_tilt_warn: Optional[Callable] = None
        self._last_imu: Optional[dict] = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def fallen(self) -> bool:
        return self._fallen

    @property
    def last_imu(self) -> Optional[dict]:
        return self._last_imu

    def on_fall(self, callback: Callable) -> None:
        self._on_fall = callback

    def on_recovered(self, callback: Callable) -> None:
        self._on_recovered = callback

    def on_tilt_warn(self, callback: Callable) -> None:
        self._on_tilt_warn = callback

    async def start(self) -> None:
        """Enable stock self-balance."""
        ok = await self._dog.enable_balance()
        self._enabled = ok
        if ok:
            logger.info("Balance layer enabled")
        else:
            logger.warning("Failed to enable balance")

    async def stop(self) -> None:
        """Disable stock self-balance."""
        await self._dog.disable_balance()
        self._enabled = False
        logger.info("Balance layer disabled")

    async def update(self) -> Optional[dict]:
        """
        Poll IMU once and check tilt state.
        Returns the IMU reading dict or None.
        """
        imu = await self._dog.read_imu()
        if imu is None:
            return None

        self._last_imu = imu
        pitch = abs(imu["pitch"])
        roll = abs(imu["roll"])
        max_tilt = max(pitch, roll)

        now = asyncio.get_running_loop().time()

        if self._fallen:
            # Check for recovery
            if max_tilt < RECOVERY_THRESHOLD:
                self._fallen = False
                logger.info("Recovered from fall (tilt=%.1f)", max_tilt)
                if self._on_recovered:
                    await self._fire(self._on_recovered, imu)
        else:
            # Check for fall
            if max_tilt >= FALL_THRESHOLD:
                if self._fall_start == 0:
                    self._fall_start = now
                elif now - self._fall_start >= FALL_DEBOUNCE:
                    self._fallen = True
                    self._fall_start = 0
                    logger.warning("Fall detected! pitch=%.1f roll=%.1f", imu["pitch"], imu["roll"])
                    if self._on_fall:
                        await self._fire(self._on_fall, imu)
                    # Attempt recovery: stand up
                    await self._dog.stand()
            else:
                self._fall_start = 0

            # Warn on moderate tilt
            if max_tilt >= WARN_THRESHOLD and not self._fallen:
                if self._on_tilt_warn:
                    await self._fire(self._on_tilt_warn, imu)

        return imu

    @staticmethod
    async def _fire(callback: Callable, data: dict) -> None:
        result = callback(data)
        if asyncio.iscoroutine(result):
            await result
