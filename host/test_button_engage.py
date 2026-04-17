#!/usr/bin/env python3
"""
ButtonEngageBehavior unit tests.

Verifies:
  - F6: optimistic engaged-state update prevents stale double-tap
  - F7: short-press (release) is gated by lock holder; long-press is not

Usage:
    python3 host/test_button_engage.py
"""

import asyncio
import sys

sys.path.insert(0, "host")

from behaviors.button_engage import ButtonEngageBehavior

# ============================================================
# Mock helpers
# ============================================================

class MockTransport:
    def __init__(self):
        self.calls = []

    async def send_json(self, msg):
        self.calls.append(msg)


class _MockLockHolder:
    """Truthy placeholder for a WebSocket lock holder."""
    def __repr__(self):
        return "<MockLockHolder>"


LOCK_HOLDER = _MockLockHolder()

# ============================================================
# Test infrastructure
# ============================================================

passed = 0
failed = 0


def _check(name: str, condition: bool, detail: str = ""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  PASS  {name}")
    else:
        failed += 1
        msg = f"  FAIL  {name}"
        if detail:
            msg += f"  [{detail}]"
        print(msg)


# ============================================================
# Tests
# ============================================================

async def test_release_no_lock_sends_engage():
    """No lock holder + _engaged=False → release sends enabled=True and calls setter."""
    transport = MockTransport()
    engaged_state = [False]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: None,
    )
    await behavior.on_button_event("release")
    _check("test_release_no_lock_sends_engage — send_json called",
           len(transport.calls) == 1)
    _check("test_release_no_lock_sends_engage — payload correct",
           transport.calls[0] == {"type": "cmd_engage", "enabled": True})
    _check("test_release_no_lock_sends_engage — setter called with True",
           engaged_state[0] is True)


async def test_release_with_lock_is_noop():
    """Lock holder present → release does NOT call send_json."""
    transport = MockTransport()
    engaged_state = [False]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: LOCK_HOLDER,
    )
    await behavior.on_button_event("release")
    _check("test_release_with_lock_is_noop — send_json not called",
           len(transport.calls) == 0)
    _check("test_release_with_lock_is_noop — engaged_state unchanged",
           engaged_state[0] is False)


async def test_long_press_always_disengages():
    """Long-press fires even when lock is held — unconditional safety e-stop."""
    transport = MockTransport()
    engaged_state = [True]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: LOCK_HOLDER,  # lock is held
    )
    await behavior.on_button_event("long_press")
    _check("test_long_press_always_disengages — send_json called despite lock",
           len(transport.calls) == 1)
    _check("test_long_press_always_disengages — payload is enabled=False",
           transport.calls[0] == {"type": "cmd_engage", "enabled": False})
    _check("test_long_press_always_disengages — setter called with False",
           engaged_state[0] is False)


async def test_optimistic_update_on_fast_double_tap():
    """
    No lock. First release when _engaged=False → sends True, setter updates state.
    Second release → reads updated True → sends False.
    Without optimistic update the second release would also send True.
    """
    transport = MockTransport()
    engaged_state = [False]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: None,
    )
    # First press
    await behavior.on_button_event("release")
    # Second press immediately (no telem_status has arrived yet)
    await behavior.on_button_event("release")

    _check("test_optimistic_update_on_fast_double_tap — two sends",
           len(transport.calls) == 2)
    _check("test_optimistic_update_on_fast_double_tap — first send enabled=True",
           transport.calls[0] == {"type": "cmd_engage", "enabled": True})
    _check("test_optimistic_update_on_fast_double_tap — second send enabled=False",
           transport.calls[1] == {"type": "cmd_engage", "enabled": False})
    _check("test_optimistic_update_on_fast_double_tap — state ends at False",
           engaged_state[0] is False)


async def test_press_is_ignored():
    """'press' event → no send_json call."""
    transport = MockTransport()
    engaged_state = [False]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: None,
    )
    await behavior.on_button_event("press")
    _check("test_press_is_ignored — send_json not called",
           len(transport.calls) == 0)


async def test_long_press_optimistic_update():
    """After long_press, setter is called with False (optimistic update)."""
    transport = MockTransport()
    engaged_state = [True]
    behavior = ButtonEngageBehavior(
        transport,
        get_engaged_fn=lambda: engaged_state[0],
        set_engaged_fn=lambda v: engaged_state.__setitem__(0, v),
        get_lock_holder_fn=lambda: None,
    )
    await behavior.on_button_event("long_press")
    _check("test_long_press_optimistic_update — setter called with False",
           engaged_state[0] is False)
    _check("test_long_press_optimistic_update — send_json called",
           len(transport.calls) == 1)


# ============================================================
# Runner
# ============================================================

async def run_all():
    tests = [
        test_release_no_lock_sends_engage,
        test_release_with_lock_is_noop,
        test_long_press_always_disengages,
        test_optimistic_update_on_fast_double_tap,
        test_press_is_ignored,
        test_long_press_optimistic_update,
    ]
    for test_fn in tests:
        name = test_fn.__name__
        print(f"\n{name}")
        try:
            await test_fn()
        except Exception as exc:
            global failed
            failed += 1
            print(f"  FAIL  {name} — raised {type(exc).__name__}: {exc}")


if __name__ == "__main__":
    asyncio.run(run_all())
    print(f"\n{'='*50}")
    print(f"Results: {passed} passed, {failed} failed")
    if failed:
        sys.exit(1)
    print("All tests passed.")
