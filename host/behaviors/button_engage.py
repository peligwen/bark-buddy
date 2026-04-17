"""
Physical button engage/disengage behavior.

Listens for telem_button events from the firmware and toggles engage state.
RELEASE → toggle engage (short press complete), only when no remote operator holds the lock
LONG_PRESS → always disengage (safety e-stop, unconditional)
PRESS → no action (arm signal only; action fires on release)

Uses the ButtonEvent values that match firmware's telem_button payload:
  {"type": "telem_button", "event": "press" | "release" | "long_press"}
"""

import asyncio
import logging

logger = logging.getLogger(__name__)


class ButtonEngageBehavior:
    def __init__(self, transport, get_engaged_fn, set_engaged_fn, get_lock_holder_fn):
        """
        transport:           FirmwareTransport instance
        get_engaged_fn:      zero-arg callable returning current engaged state (bool)
        set_engaged_fn:      one-arg callable that sets engaged state (bool) — used for optimistic updates
        get_lock_holder_fn:  zero-arg callable returning current lock holder (None or truthy WebSocketResponse)
        """
        self._transport = transport
        self._get_engaged = get_engaged_fn
        self._set_engaged = set_engaged_fn
        self._get_lock_holder = get_lock_holder_fn

    async def on_button_event(self, event: str) -> None:
        if event == "long_press":
            # Safety e-stop — always fires regardless of lock
            await self._transport.send_json({"type": "cmd_engage", "enabled": False})
            self._set_engaged(False)
            logger.info("ButtonEngage: long_press → disengage (unconditional e-stop)")
        elif event == "release":
            # Short-press toggle — only when no remote operator holds the lock
            if self._get_lock_holder() is not None:
                logger.debug("ButtonEngage: release ignored — lock held by %s", self._get_lock_holder())
                return
            engaged = self._get_engaged()
            await self._transport.send_json({"type": "cmd_engage", "enabled": not engaged})
            self._set_engaged(not engaged)  # optimistic update
            logger.info("ButtonEngage: release → %s", "disengage" if engaged else "engage")
        # 'press' intentionally ignored — action fires on release
