"""Integration test: the mock firmware walks and streams joint angles to the host."""

import asyncio
import socket
import subprocess
from pathlib import Path

import pytest
from dog import Dog

MOCK_BIN = Path(__file__).resolve().parents[2] / "firmware" / "test" / "bark-mock"


@pytest.fixture
def mock_port(tmp_path):
    if not MOCK_BIN.exists():
        pytest.skip("mock firmware not built (cd firmware/test && make bark-mock)")
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    # The mock persists NVS state under $HOME; a private one keeps saved
    # offsets from a developer's own sessions out of the standing pose.
    proc = subprocess.Popen([str(MOCK_BIN), "--tcp-port", str(port)],
                            env={"HOME": str(tmp_path)},
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    yield port
    proc.terminate()
    proc.wait(timeout=5)


@pytest.fixture
async def mock_dog(mock_port):
    dog = Dog(host="127.0.0.1", tcp_port=mock_port)
    for _ in range(50):
        try:
            await dog.open()
            break
        except OSError:
            await asyncio.sleep(0.1)
    else:
        pytest.fail("mock firmware did not accept a connection")
    yield dog
    await dog.close()


async def _collect(messages: list, seconds: float) -> list:
    start = len(messages)
    await asyncio.sleep(seconds)
    return messages[start:]


def _hip_swing(joints: list[dict]) -> float:
    """Largest hip-angle range over the sample, across legs (radians)."""
    return max(max(j[leg]["h"] for j in joints) - min(j[leg]["h"] for j in joints)
               for leg in ("fl", "fr", "rl", "rr"))


async def test_mock_walks_and_reports_joint_angles(mock_dog):
    joints: list[dict] = []
    events: list[str] = []

    def on_telem(msg):
        if msg.get("type") == "telem_joints":
            joints.append(msg)
        elif msg.get("type") == "telem_event":
            events.append(msg.get("event"))

    mock_dog.set_telem_callback(on_telem)

    await mock_dog.send_json({"type": "cmd_engage", "enabled": True})
    for _ in range(50):
        if "engage_complete" in events:
            break
        await asyncio.sleep(0.1)
    assert "engage_complete" in events

    standing = await _collect(joints, 0.4)
    assert len(standing) >= 4, "no joint telemetry while standing"
    assert _hip_swing(standing) < 1e-3

    await mock_dog.send_json({"type": "cmd_move", "direction": "forward", "speed": 1.0})
    await asyncio.sleep(0.4)  # speed ramps 0→1 in ~330 ms
    walking = await _collect(joints, 0.8)
    assert len(walking) >= 8, "joint telemetry stalled while walking"
    assert _hip_swing(walking) > 0.1, "legs are not moving"

    await mock_dog.send_json({"type": "cmd_move", "direction": "stop"})
    await asyncio.sleep(1.0)  # 600 ms stand-return taper
    stopped = await _collect(joints, 0.3)
    assert _hip_swing(stopped) < 1e-3
    assert stopped[-1]["fl"] == standing[-1]["fl"]
