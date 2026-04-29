"""Tests for ControlLock — single-holder mutex with idle timeout."""

import time
import pytest

from lock import ControlLock


# Sentinel objects stand in for WebSocketResponse — ControlLock only does
# identity comparisons, so any hashable distinct objects work.
class _WS:
    def __init__(self, name):
        self.name = name


def test_unlocked_initially():
    lock = ControlLock(timeout=30.0)
    a = _WS("a")
    assert lock.holder is None
    assert lock.holder_name == ""
    assert not lock.is_locked()
    assert not lock.is_locked_by(a)
    assert lock.can_control(a)


def test_acquire_release():
    lock = ControlLock(timeout=30.0)
    a, b = _WS("a"), _WS("b")
    lock.acquire(a, "Alice")
    assert lock.is_locked()
    assert lock.is_locked_by(a)
    assert lock.holder is a
    assert lock.holder_name == "Alice"
    assert lock.can_control(a)
    assert not lock.can_control(b)

    lock.release()
    assert not lock.is_locked()
    assert lock.holder is None
    assert lock.can_control(b)


def test_release_if_holder_returns_true_only_for_holder():
    lock = ControlLock(timeout=30.0)
    a, b = _WS("a"), _WS("b")
    lock.acquire(a, "Alice")
    assert lock.release_if_holder(b) is False
    assert lock.is_locked_by(a)
    assert lock.release_if_holder(a) is True
    assert not lock.is_locked()


def test_timeout_releases_silently():
    lock = ControlLock(timeout=0.05)
    a = _WS("a")
    lock.acquire(a, "Alice")
    assert lock.is_locked()
    time.sleep(0.07)
    # No explicit release call; queries should reflect timeout.
    assert lock.is_timed_out()
    assert not lock.is_locked()
    assert lock.holder is None
    assert lock.can_control(_WS("b"))


def test_touch_extends_timeout():
    lock = ControlLock(timeout=0.10)
    a = _WS("a")
    lock.acquire(a, "Alice")
    time.sleep(0.07)
    lock.touch()
    time.sleep(0.05)
    # Still within touch+timeout window
    assert lock.is_locked()


def test_touch_no_holder_is_noop():
    lock = ControlLock(timeout=30.0)
    lock.touch()  # must not raise
    assert lock.holder is None


def test_status_msg_shape_when_locked():
    lock = ControlLock(timeout=30.0)
    a = _WS("a")
    lock.acquire(a, "Alice")
    msg = lock.status_msg()
    assert msg["type"] == "lock_status"
    assert msg["locked"] is True
    assert msg["holder"] == "Alice"
    assert msg["is_you"] is False  # caller overrides per-client


def test_status_msg_shape_when_unlocked():
    lock = ControlLock(timeout=30.0)
    msg = lock.status_msg()
    assert msg["type"] == "lock_status"
    assert msg["locked"] is False
    assert msg["holder"] is None


def test_status_msg_after_timeout_reports_unlocked():
    lock = ControlLock(timeout=0.05)
    a = _WS("a")
    lock.acquire(a, "Alice")
    time.sleep(0.07)
    msg = lock.status_msg()
    # Timed-out lock should not surface as locked to clients.
    assert msg["locked"] is False
    assert msg["holder"] is None
