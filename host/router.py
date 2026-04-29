"""
CommandRouter — dispatches WebSocket messages from browser clients.

Three categories of messages:

1. **Lock control** (`cmd_identify`, `cmd_lock`, `cmd_unlock`, `cmd_lock_yield`)
   — manipulate the control lock; always allowed.
2. **Lock-gated motion** (`cmd_move`, `cmd_stand`, `cmd_balance`, `cmd_engage`)
   — must hold the control lock (auto-acquires if no one holds it). Refresh
   lock timeout on each accepted command.
3. **Host-only** (`cmd_reset`, `cmd_restart_server`) — server-side actions.
4. **Firmware passthrough** — every other `cmd_*` that targets the firmware
   is sent verbatim through the transport. Adding a new firmware command is
   a one-line edit to `FIRMWARE_PASSTHROUGH`.

The router itself does not own state. It takes a small set of injected
collaborators and pure callables, so it can be exercised in tests without
spinning up the full server.
"""

import asyncio
import json
import logging
from typing import Awaitable, Callable

from aiohttp import web

from dog import DIRECTIONS

logger = logging.getLogger(__name__)


class CommandRouter:
    LOCK_GATED = frozenset({"cmd_move", "cmd_stand", "cmd_balance", "cmd_engage"})
    LOCK_CONTROL = frozenset({"cmd_identify", "cmd_lock", "cmd_unlock", "cmd_lock_yield"})
    HOST_ONLY = frozenset({"cmd_reset", "cmd_restart_server"})
    FIRMWARE_PASSTHROUGH = frozenset({
        "cmd_engage", "cmd_ota_update", "cmd_servo", "cmd_transform",
        "cmd_gait_params", "cmd_probe_pin", "cmd_balance_config",
        "cmd_offset", "cmd_servo_pin", "cmd_buzzer", "cmd_led",
        "cmd_gpio", "cmd_i2c", "cmd_yaw_trim",
    })

    def __init__(
        self,
        *,
        lock,
        client_names: dict,
        relay,
        get_transport: Callable[[], object],
        balance,
        set_motion: Callable[[str], None],
        on_reset: Callable[[], Awaitable[None]],
        on_restart_server: Callable[[], None],
    ):
        self._lock = lock
        self._client_names = client_names
        self._relay = relay
        self._get_transport = get_transport
        self._balance = balance
        self._set_motion = set_motion
        self._on_reset = on_reset
        self._on_restart_server = on_restart_server

    def set_balance(self, balance) -> None:
        """Replace the balance reference after a transport swap."""
        self._balance = balance

    async def dispatch(self, data: str, ws: web.WebSocketResponse) -> None:
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON from browser: %s", data[:200])
            return

        msg_type = msg.get("type")
        if not msg_type:
            return

        # Identity is always allowed — pre-lock.
        if msg_type == "cmd_identify":
            self._client_names[ws] = msg.get("name", "Operator")
            return

        if msg_type in self.LOCK_CONTROL:
            await self._handle_lock_command(msg, ws)
            return

        if msg_type in self.LOCK_GATED:
            if not await self._gate_lock(ws):
                return  # denied; lock_denied already sent

        if msg_type in self.HOST_ONLY:
            await self._handle_host_command(msg)
            return

        # Firmware passthrough — covers the remaining LOCK_GATED set too
        # (cmd_move/stand/balance/engage), since they're motion commands.
        if msg_type == "cmd_move":
            await self._handle_cmd_move(msg)
        elif msg_type == "cmd_stand":
            await self._send_to_firmware({"type": "cmd_stand"})
            self._set_motion("stand")
        elif msg_type == "cmd_balance":
            enabled = msg.get("enabled", True)
            await self._balance.set_enabled(enabled)
            await self._relay.broadcast({"type": "balance_state",
                                         "enabled": self._balance.enabled})
        elif msg_type in self.FIRMWARE_PASSTHROUGH:
            await self._send_to_firmware(msg)
        else:
            logger.warning("Unknown WS message type: %s", msg_type)

    # --- Lock command handlers ---

    async def _handle_lock_command(self, msg: dict, ws: web.WebSocketResponse) -> None:
        msg_type = msg["type"]
        if msg_type == "cmd_lock":
            name = msg.get("name", "Anonymous")
            if self._lock.can_control(ws):
                self._lock.acquire(ws, name)
                await self.broadcast_lock_status()
            elif self._lock.holder is not None:
                await self._relay.send_to(self._lock.holder, {
                    "type": "lock_challenge", "challenger": name,
                })
                await self._relay.send_to(ws, {
                    "type": "lock_denied", "holder": self._lock.holder_name,
                })
        elif msg_type in ("cmd_unlock", "cmd_lock_yield"):
            if self._lock.release_if_holder(ws):
                await self.broadcast_lock_status()

    async def _gate_lock(self, ws: web.WebSocketResponse) -> bool:
        if self._lock.is_timed_out():
            self._lock.release()
        if not self._lock.can_control(ws):
            await self._relay.send_to(ws, {
                "type": "lock_denied", "holder": self._lock.holder_name,
            })
            return False
        if self._lock.holder is None:
            self._lock.acquire(ws, self._client_names.get(ws, "Operator"))
            await self.broadcast_lock_status()
        if self._lock.is_locked_by(ws):
            self._lock.touch()
        return True

    async def broadcast_lock_status(self) -> None:
        if self._lock.is_timed_out():
            self._lock.release()
        msg = self._lock.status_msg()
        for ws in self._relay.clients():
            m = dict(msg)
            m["is_you"] = self._lock.is_locked_by(ws)
            await self._relay.send_to(ws, m)

    # --- Host commands ---

    async def _handle_host_command(self, msg: dict) -> None:
        msg_type = msg["type"]
        if msg_type == "cmd_reset":
            await self._on_reset()
        elif msg_type == "cmd_restart_server":
            logger.warning("=== SERVER RESTART REQUESTED ===")
            try:
                await self._relay.broadcast({"type": "server_restarting"})
            except Exception:
                pass
            self._on_restart_server()

    # --- Firmware-direct commands ---

    async def _handle_cmd_move(self, msg: dict) -> None:
        direction = msg.get("direction", "stop")
        if direction not in DIRECTIONS:
            logger.warning("Unknown direction: %s", direction)
            return
        speed = msg.get("speed", 1.0)
        await self._send_to_firmware({"type": "cmd_move", "direction": direction, "speed": speed})
        self._set_motion(direction)

    async def _send_to_firmware(self, msg: dict) -> None:
        transport = self._get_transport()
        if not transport:
            return
        try:
            await transport.send_json(msg)
        except Exception as e:
            logger.warning("Passthrough send failed (%s): %s", msg.get("type"), e)
