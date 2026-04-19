"""WebSocket control-lock state machine."""

import time
from aiohttp import web


class ControlLock:
    """
    Mutual-exclusion lock for WebSocket control sessions.

    One client holds the lock at a time. Lock auto-releases after
    `timeout` seconds of inactivity. Call touch() on each control command.
    """

    def __init__(self, timeout: float = 30.0):
        self._holder: web.WebSocketResponse | None = None
        self._name: str = ""
        self._time: float = 0.0
        self._timeout = timeout

    def is_timed_out(self) -> bool:
        if self._holder and self._time:
            return time.monotonic() - self._time > self._timeout
        return False

    def is_locked_by(self, ws: web.WebSocketResponse) -> bool:
        if self.is_timed_out():
            return False
        return self._holder is ws

    def is_locked(self) -> bool:
        if self.is_timed_out():
            return False
        return self._holder is not None

    def can_control(self, ws: web.WebSocketResponse) -> bool:
        if self.is_timed_out():
            return True
        return self._holder is None or self._holder is ws

    def acquire(self, ws: web.WebSocketResponse, name: str) -> None:
        self._holder = ws
        self._name = name
        self._time = time.monotonic()

    def release(self) -> None:
        self._holder = None
        self._name = ""
        self._time = 0.0

    def touch(self) -> None:
        """Reset timeout — call on each control command from lock holder."""
        if self._holder:
            self._time = time.monotonic()

    def release_if_holder(self, ws: web.WebSocketResponse) -> bool:
        """Release lock if ws holds it. Returns True if released."""
        if self._holder is ws:
            self.release()
            return True
        return False

    @property
    def holder(self) -> web.WebSocketResponse | None:
        return self._holder if not self.is_timed_out() else None

    @property
    def holder_name(self) -> str:
        return self._name if not self.is_timed_out() else ""

    def status_msg(self) -> dict:
        timed_out = self.is_timed_out()
        return {
            "type": "lock_status",
            "locked": self._holder is not None and not timed_out,
            "holder": self._name if self._holder and not timed_out else None,
            "is_you": False,  # caller overrides per-client
        }
