"""
TelemetryRelay — fan-out from host to all WS clients.

Owns the set of WebSocket clients and a single broadcaster task. Synchronous
producers (firmware telem callbacks running in the asyncio loop but outside a
coroutine) call `enqueue(msg)`; the broadcaster task drains the queue and
sends each message to all clients in arrival order.

Why a queue with a single consumer:
- Eliminates orphan `asyncio.ensure_future(...)` tasks (no
  "Task exception was never retrieved" warnings, no shutdown-time
  RuntimeError("no running event loop")).
- Serialises ordering — two messages from the firmware cannot interleave
  their `await ws.send_str(...)`.
- Centralises dead-client GC so each broadcast path doesn't re-implement it.
"""

import asyncio
import json
import logging
from typing import Iterable

from aiohttp import web

logger = logging.getLogger(__name__)


class TelemetryRelay:
    def __init__(self, *, queue_max: int = 4096):
        self._clients: set[web.WebSocketResponse] = set()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=queue_max)
        self._task: asyncio.Task | None = None

    # --- Client set ---

    def add_client(self, ws: web.WebSocketResponse) -> None:
        self._clients.add(ws)

    def remove_client(self, ws: web.WebSocketResponse) -> None:
        self._clients.discard(ws)

    def has_clients(self) -> bool:
        return bool(self._clients)

    def clients(self) -> Iterable[web.WebSocketResponse]:
        # Snapshot so callers can iterate while we mutate on dead-client GC.
        return list(self._clients)

    def client_count(self) -> int:
        return len(self._clients)

    # --- Producer API (sync-safe; called from telem callbacks) ---

    def enqueue(self, msg: dict) -> None:
        try:
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            logger.warning("Relay queue full — dropping %s", msg.get("type"))

    # --- Direct send (caller awaits delivery) ---

    async def broadcast(self, msg: dict) -> None:
        data = json.dumps(msg)
        dead = set()
        for ws in self._clients:
            try:
                await ws.send_str(data)
            except (ConnectionError, ConnectionResetError):
                dead.add(ws)
        self._clients -= dead

    async def send_to(self, ws: web.WebSocketResponse, msg: dict) -> bool:
        """Return True if delivered, False if the client looked dead."""
        try:
            await ws.send_str(json.dumps(msg))
            return True
        except (ConnectionError, ConnectionResetError):
            self._clients.discard(ws)
            return False

    # --- Lifecycle ---

    async def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="relay-broadcaster")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None

    async def _run(self) -> None:
        while True:
            try:
                msg = await self._queue.get()
            except asyncio.CancelledError:
                break
            try:
                await self.broadcast(msg)
            except Exception:
                logger.exception("relay: broadcast failed for %s", msg.get("type"))
