"""
Physical button engage/disengage behavior.

Listens for telem_button events from the firmware and toggles engage state.
PRESS → if disengaged: send cmd_engage(enabled=True)
PRESS → if engaged: send cmd_engage(enabled=False)
LONG_PRESS → always disengage (safety fallback)

Uses the ButtonEvent values that match firmware's telem_button payload:
  {"type": "telem_button", "event": "press" | "release" | "long_press"}
"""

import asyncio
import logging

logger = logging.getLogger(__name__)


class ButtonEngageBehavior:
    def __init__(self, transport, get_engaged_fn):
        """
        transport: FirmwareTransport instance
        get_engaged_fn: zero-arg callable returning current engaged state (bool)
        """
        self._transport = transport
        self._get_engaged = get_engaged_fn
        self._running = False

    async def on_button_event(self, event: str) -> None:
        if event == "long_press":
            await self._transport.send_json({"type": "cmd_engage", "enabled": False})
            logger.info("ButtonEngage: long_press → disengage")
        elif event == "press":
            engaged = self._get_engaged()
            await self._transport.send_json({"type": "cmd_engage", "enabled": not engaged})
            logger.info("ButtonEngage: press → %s", "disengage" if engaged else "engage")
