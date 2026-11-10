"""Tests for CaptureRecorder — keyframes only when the dog is standing still."""

import asyncio
import json

import pytest
from behaviors.capture import SETTLE_S, CaptureRecorder
from camera import CameraError


class _Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class _FakeTransport:
    def __init__(self):
        self.engaged = True
        self.ramping = False
        self.buzzes: list[tuple[int, int]] = []

    def is_open(self):
        return True

    def get_engaged(self):
        return self.engaged

    def get_ramping(self):
        return self.ramping

    def get_fw_version(self):
        return "0.4.0"

    async def cmd_buzzer(self, freq_hz, duration_ms):
        self.buzzes.append((freq_hz, duration_ms))


class _FakeCamera:
    url = "http://cam.test/capture"

    def __init__(self):
        self.grabs = 0
        self.error: CameraError | None = None
        self.on_grab = None

    async def open(self):
        pass

    async def close(self):
        pass

    async def grab(self):
        if self.on_grab:
            self.on_grab()
        if self.error:
            raise self.error
        self.grabs += 1
        return b"\xff\xd8frame" + str(self.grabs).encode() + b"\xff\xd9"


class _Rig:
    def __init__(self, tmp_path):
        self.clock = _Clock()
        self.transport = _FakeTransport()
        self.camera = _FakeCamera()
        self.rec = CaptureRecorder(lambda: self.transport, self.camera, tmp_path,
                                   clock=self.clock)
        self.rec._open_run()  # files without the background loop; tests drive step()

    async def settle_and_step(self):
        self.clock.advance(SETTLE_S)
        await self.rec.step()

    def records(self):
        path = self.rec.run_dir / "frames.jsonl"
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text().splitlines()]


@pytest.fixture
def rig(tmp_path):
    return _Rig(tmp_path)


@pytest.mark.asyncio
async def test_first_keyframe_waits_for_settle(rig):
    await rig.rec.step()
    assert rig.rec.frame_count == 0

    await rig.settle_and_step()
    assert rig.rec.frame_count == 1
    [record] = rig.records()
    assert record["idx"] == 0
    assert record["moves"] == []
    assert (rig.rec.run_dir / record["file"]).read_bytes().startswith(b"\xff\xd8")
    assert len(rig.transport.buzzes) == 1


@pytest.mark.asyncio
async def test_no_keyframe_until_engaged_and_settled(rig):
    rig.transport.engaged = False
    rig.clock.advance(10 * SETTLE_S)
    await rig.rec.step()
    assert rig.rec.frame_count == 0

    # Engage ramp: still not standing.
    rig.transport.engaged = True
    rig.transport.ramping = True
    await rig.settle_and_step()
    assert rig.rec.frame_count == 0

    # Ramp done — the settle period starts from the last non-standing poll.
    rig.transport.ramping = False
    await rig.rec.step()
    assert rig.rec.frame_count == 0
    await rig.settle_and_step()
    assert rig.rec.frame_count == 1


@pytest.mark.asyncio
async def test_keyframe_after_stop_records_moves_since_last(rig):
    await rig.settle_and_step()

    rig.rec.on_motion("forward", 1.0)
    rig.clock.advance(1.5)
    rig.rec.on_motion("forward", 1.0)  # key repeat: same segment
    rig.clock.advance(0.5)
    rig.rec.on_motion("rotate_left", 0.5)
    rig.clock.advance(1.0)
    await rig.rec.step()  # still moving
    assert rig.rec.frame_count == 1

    rig.rec.on_motion("stop")
    await rig.rec.step()  # stopped but not settled
    assert rig.rec.frame_count == 1

    await rig.settle_and_step()
    assert rig.rec.frame_count == 2
    assert rig.records()[1]["moves"] == [
        {"direction": "forward", "speed": 1.0, "duration_s": 2.0},
        {"direction": "rotate_left", "speed": 0.5, "duration_s": 1.0},
    ]
    assert len(rig.transport.buzzes) == 2


@pytest.mark.asyncio
async def test_no_second_keyframe_without_motion(rig):
    await rig.settle_and_step()
    rig.rec.on_motion("stand")
    rig.clock.advance(10 * SETTLE_S)
    await rig.rec.step()
    assert rig.rec.frame_count == 1


@pytest.mark.asyncio
async def test_frame_discarded_when_told_to_move_during_grab(rig):
    rig.camera.on_grab = lambda: rig.rec.on_motion("forward", 1.0)
    await rig.settle_and_step()
    assert rig.rec.frame_count == 0
    assert rig.transport.buzzes == []

    # The next stop retakes it, and the interrupted move is on its record.
    rig.camera.on_grab = None
    rig.clock.advance(0.4)
    rig.rec.on_motion("stop")
    await rig.settle_and_step()
    assert rig.rec.frame_count == 1
    assert rig.records()[0]["moves"] == [
        {"direction": "forward", "speed": 1.0, "duration_s": 0.4},
    ]


@pytest.mark.asyncio
async def test_camera_error_retries_after_another_settle(rig):
    rig.camera.error = CameraError("unreachable")
    await rig.settle_and_step()
    assert rig.rec.frame_count == 0

    rig.camera.error = None
    await rig.rec.step()
    assert rig.rec.frame_count == 0
    await rig.settle_and_step()
    assert rig.rec.frame_count == 1


@pytest.mark.asyncio
async def test_record_and_log_carry_telemetry(rig):
    rig.rec.on_telem({"type": "telem_imu", "pitch": 1.5, "roll": -0.5, "yaw": 42.0, "gz": 0.1})
    rig.rec.on_telem({"type": "telem_sonar", "distance_mm": 310})
    rig.rec.on_telem({"type": "telem_joints", "fl": {"h": 0.3, "k": -0.6}})
    rig.rec.on_telem({"type": "ack", "ref_type": "cmd_move", "ok": True})
    rig.rec.on_motion("forward", 1.0)
    rig.rec.on_motion("stop")
    await rig.settle_and_step()

    [record] = rig.records()
    assert record["imu"] == {"pitch": 1.5, "roll": -0.5, "yaw": 42.0, "gz": 0.1}
    assert record["sonar_mm"] == 310
    assert record["joints"] == {"fl": {"h": 0.3, "k": -0.6}}

    log = [json.loads(line)
           for line in (rig.rec.run_dir / "telemetry.ndjson").read_text().splitlines()]
    assert [m["type"] for m in log] == [
        "telem_imu", "telem_sonar", "telem_joints", "host_motion", "host_motion",
    ]
    assert all("host_t" in m for m in log)
    assert log[3]["direction"] == "forward" and log[4]["direction"] == "stop"


@pytest.mark.asyncio
async def test_start_runs_the_loop_and_stop_closes_the_run(tmp_path):
    clock = _Clock()
    transport = _FakeTransport()
    rec = CaptureRecorder(lambda: transport, _FakeCamera(), tmp_path, clock=clock)
    await rec.start()
    clock.advance(SETTLE_S)
    for _ in range(50):
        if rec.frame_count:
            break
        await asyncio.sleep(0.02)
    await rec.stop()

    assert rec.frame_count == 1
    meta = json.loads((rec.run_dir / "meta.json").read_text())
    assert meta["camera_url"] == "http://cam.test/capture"
    assert meta["fw_version"] == "0.4.0"
    assert rec.run_dir.parent == tmp_path
    assert (rec.run_dir / "frames" / "000000.jpg").exists()
