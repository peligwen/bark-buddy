"""
Stop-and-snap capture.

Saves a keyframe each time the dog comes to rest: one camera still plus the
telemetry that says where and how it was taken. A standing dog gives sharp
frames over a slow WiFi camera link, and the reconstruction models register
widely spaced stills, so no video stream is needed.

Run directory (one per server start, under the capture root):
    meta.json          run parameters
    frames/NNNNNN.jpg  keyframes
    frames.jsonl       one record per keyframe
    telemetry.ndjson   every firmware telemetry message and accepted motion
                       command, stamped with host time
"""

import asyncio
import json
import logging
import time
from datetime import datetime
from pathlib import Path

from camera import CameraError

logger = logging.getLogger(__name__)

# Firmware tapers back to the standing pose over 600 ms (STAND_RETURN_MS);
# the remainder lets the body stop rocking before the exposure.
SETTLE_S = 1.0
POLL_S = 0.1

_AT_REST = frozenset({"stop", "stand"})
_LOGGED_PREFIXES = ("telem_", "boot")
_SAVED_CHIRP = (2400, 60)  # Hz, ms — tells the operator it is safe to move on


class CaptureRecorder:

    def __init__(self, get_transport, camera, root, clock=time.monotonic):
        """
        get_transport:  zero-arg callable returning the current Dog (survives transport swaps)
        camera:         Camera instance
        root:           directory under which a timestamped run directory is created
        """
        self._get_transport = get_transport
        self._camera = camera
        self._root = Path(root)
        self._clock = clock

        self._run_dir: Path | None = None
        self._telemetry = None
        self._task: asyncio.Task | None = None

        self._index = 0
        self._direction: str | None = None  # None while at rest
        self._speed = 0.0
        self._since = clock()
        self._motion_seq = 0
        self._not_before = self._since + SETTLE_S
        self._moves: list[dict] = []
        # A keyframe is owed whenever the view may have changed; true at start
        # so the first standing pose is recorded before any motion.
        self._pending = True

        self._imu: dict = {}
        self._sonar_mm: int | None = None
        self._joints: dict | None = None

    @property
    def run_dir(self) -> Path | None:
        return self._run_dir

    @property
    def frame_count(self) -> int:
        return self._index

    # --- Lifecycle ---

    async def start(self) -> None:
        self._open_run()
        await self._camera.open()
        self._task = asyncio.create_task(self._run(), name="capture")
        logger.info("Capture: recording keyframes to %s", self._run_dir)

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        await self._camera.close()
        if self._telemetry is not None:
            self._telemetry.close()
            self._telemetry = None
            logger.info("Capture: %d keyframes in %s", self._index, self._run_dir)

    # --- Inputs from the server ---

    def on_motion(self, direction: str, speed: float = 0.0) -> None:
        """An accepted motion command: a cmd_move direction, or "stand"."""
        moving = None if direction in _AT_REST else direction
        if moving == self._direction and (moving is None or speed == self._speed):
            return  # key repeat or redundant stop
        now = self._clock()
        if self._direction is not None:
            self._moves.append({
                "direction": self._direction,
                "speed": self._speed,
                "duration_s": round(now - self._since, 3),
            })
        self._direction, self._speed, self._since = moving, speed, now
        self._motion_seq += 1
        if moving is None:
            self._not_before = now + SETTLE_S
        else:
            self._pending = True
        self._log({"type": "host_motion", "direction": direction, "speed": speed})

    def on_telem(self, msg: dict) -> None:
        msg_type = msg.get("type", "")
        if msg_type == "telem_imu":
            self._imu = msg
        elif msg_type == "telem_sonar":
            self._sonar_mm = msg.get("distance_mm")
        elif msg_type == "telem_joints":
            self._joints = msg
        if msg_type.startswith(_LOGGED_PREFIXES):
            self._log(msg)

    # --- Keyframe loop ---

    async def _run(self) -> None:
        while True:
            try:
                await self.step()
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.exception("Capture step failed")
            await asyncio.sleep(POLL_S)

    async def step(self) -> None:
        """Save a keyframe if the dog has been standing still long enough."""
        if not self._due():
            return
        seq = self._motion_seq
        record = self._keyframe_record()
        try:
            jpeg = await self._camera.grab()
        except CameraError as e:
            logger.warning("Capture: no frame (%s)", e)
            self._not_before = self._clock() + SETTLE_S
            return
        if self._motion_seq != seq:
            return  # told to move mid-exposure; the next stop will retake it
        self._save_keyframe(jpeg, record)
        await self._get_transport().cmd_buzzer(*_SAVED_CHIRP)

    def _due(self) -> bool:
        now = self._clock()
        transport = self._get_transport()
        standing = (transport is not None and transport.is_open()
                    and transport.get_engaged() and not transport.get_ramping())
        if not standing:
            # Engaging changes the pose without a motion command; settle after it too.
            self._not_before = now + SETTLE_S
            return False
        return self._pending and self._direction is None and now >= self._not_before

    def _keyframe_record(self) -> dict:
        return {
            "idx": self._index,
            "file": f"frames/{self._index:06d}.jpg",
            "t": round(time.time(), 3),
            "imu": _without_type(self._imu),
            "sonar_mm": self._sonar_mm,
            "joints": _without_type(self._joints) if self._joints else None,
            "moves": list(self._moves),
        }

    # --- Files ---

    def _open_run(self) -> None:
        self._run_dir = self._root / datetime.now().strftime("%Y-%m-%d_%H%M%S")
        (self._run_dir / "frames").mkdir(parents=True)
        transport = self._get_transport()
        meta = {
            "started": datetime.now().astimezone().isoformat(timespec="seconds"),
            "camera_url": self._camera.url,
            "settle_s": SETTLE_S,
            "fw_version": transport.get_fw_version() if transport else "",
        }
        (self._run_dir / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        self._telemetry = open(self._run_dir / "telemetry.ndjson", "w")

    def _save_keyframe(self, jpeg: bytes, record: dict) -> None:
        (self._run_dir / record["file"]).write_bytes(jpeg)
        with open(self._run_dir / "frames.jsonl", "a") as f:
            f.write(json.dumps(record) + "\n")
        self._telemetry.flush()
        logger.info("Capture: keyframe %d (%d bytes, %d moves since last)",
                    record["idx"], len(jpeg), len(record["moves"]))
        self._index += 1
        self._moves = []
        self._pending = False

    def _log(self, msg: dict) -> None:
        if self._telemetry is not None:
            self._telemetry.write(
                json.dumps({"host_t": round(time.time(), 4), **msg}, separators=(",", ":")) + "\n")


def _without_type(msg: dict) -> dict:
    return {k: v for k, v in msg.items() if k != "type"}
