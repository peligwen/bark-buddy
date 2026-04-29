"""Tests for BalanceLayer — fall/recover hysteresis (pure synchronous check)."""

import pytest

from behaviors.balance import (
    BalanceLayer, FALL_PITCH_DEG, FALL_ROLL_DEG,
    RECOVER_PITCH_DEG, RECOVER_ROLL_DEG,
)


class _FakeTransport:
    """Minimal transport stub for BalanceLayer.check — returns the supplied IMU dict."""
    def __init__(self):
        self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self.sent: list[dict] = []

    def get_imu(self):
        return dict(self._imu)

    def set_imu(self, pitch=0.0, roll=0.0, yaw=0.0):
        self._imu = {"pitch": pitch, "roll": roll, "yaw": yaw}

    async def send_json(self, msg):
        self.sent.append(msg)


def test_no_fall_when_level():
    t = _FakeTransport()
    bl = BalanceLayer(t)
    t.set_imu(pitch=0.0, roll=0.0)
    e = bl.check()
    assert e["fallen"] is False
    assert e["recovered"] is False
    assert bl.is_fallen is False


def test_fall_on_pitch_above_threshold():
    t = _FakeTransport()
    bl = BalanceLayer(t)
    t.set_imu(pitch=FALL_PITCH_DEG + 1.0)
    e = bl.check()
    assert e["fallen"] is True
    assert bl.is_fallen is True
    # Same overage on next tick — should not re-fire fallen edge.
    e2 = bl.check()
    assert e2["fallen"] is False
    assert e2["recovered"] is False
    assert bl.is_fallen is True


def test_fall_on_roll_above_threshold():
    t = _FakeTransport()
    bl = BalanceLayer(t)
    t.set_imu(roll=-(FALL_ROLL_DEG + 1.0))  # negative roll: abs() must trigger
    assert bl.check()["fallen"] is True


def test_hysteresis_band_does_not_recover():
    """Between RECOVER and FALL thresholds, a fallen dog stays fallen."""
    t = _FakeTransport()
    bl = BalanceLayer(t)
    t.set_imu(pitch=FALL_PITCH_DEG + 1.0)
    bl.check()
    assert bl.is_fallen
    # Drop into the hysteresis band — above RECOVER but below FALL.
    t.set_imu(pitch=(FALL_PITCH_DEG + RECOVER_PITCH_DEG) / 2)
    e = bl.check()
    assert e["fallen"] is False
    assert e["recovered"] is False
    assert bl.is_fallen


def test_recover_below_threshold():
    t = _FakeTransport()
    bl = BalanceLayer(t)
    t.set_imu(pitch=FALL_PITCH_DEG + 1.0)
    bl.check()
    t.set_imu(pitch=RECOVER_PITCH_DEG - 1.0, roll=RECOVER_ROLL_DEG - 1.0)
    e = bl.check()
    assert e["recovered"] is True
    assert bl.is_fallen is False
    # Subsequent ticks while still level emit no event.
    e2 = bl.check()
    assert e2["recovered"] is False
    assert e2["fallen"] is False


@pytest.mark.asyncio
async def test_set_enabled_forwards_cmd_balance():
    t = _FakeTransport()
    bl = BalanceLayer(t)
    await bl.set_enabled(True)
    assert bl.enabled
    assert t.sent == [{"type": "cmd_balance", "enabled": True}]
    await bl.set_enabled(False)
    assert bl.enabled is False
    assert t.sent[-1] == {"type": "cmd_balance", "enabled": False}
