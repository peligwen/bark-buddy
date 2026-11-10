"""Tests for the reconstruction geometry, on a synthetic room with known ground truth."""

import json

import numpy as np
import pytest
from perception import geometry, reconstruct
from perception.da3 import Prediction

CAM_HEIGHT_M = 0.11
PITCH_DOWN_DEG = 4.0          # camera mount tilt the solver is not told about
MODEL_SCALE = 2.5             # metres per model unit
FORWARD_MM_S, TURN_DEG_S = 40.0, 20.0
W, H, FOCAL = 160, 120, 133.0

ROOM_LO, ROOM_HI = np.array([-1.0, -1.5, 0.0]), np.array([3.0, 1.5, 2.4])
BOX_LO, BOX_HI = np.array([1.4, -0.9, 0.0]), np.array([1.9, -0.4, 0.15])

# (x, y, heading°) of each keyframe: a loop of straight runs and turns in place.
PATH = [(0.0, 0.0, 0), (0.3, 0.0, 0), (0.6, 0.0, 0), (0.9, 0.0, 0),
        (0.9, 0.0, 30), (0.9, 0.0, 60), (0.9, 0.0, 90),
        (0.9, 0.3, 90), (0.9, 0.6, 90), (0.9, 0.9, 90),
        (0.9, 0.9, 135), (0.9, 0.9, 180),
        (0.6, 0.9, 180), (0.3, 0.9, 180), (0.0, 0.9, 180),
        (0.0, 0.9, 225), (0.0, 0.9, 270),
        (0.0, 0.6, 270), (0.0, 0.3, 270)]


def _camera(x, y, yaw_deg):
    """Metric world-to-camera (R, t) for an upright camera pitched slightly down."""
    yaw, pitch = np.radians(yaw_deg), np.radians(PITCH_DOWN_DEG)
    forward0 = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    right = np.array([np.sin(yaw), -np.cos(yaw), 0.0])
    down0 = np.array([0.0, 0.0, -1.0])
    forward = np.cos(pitch) * forward0 + np.sin(pitch) * down0
    down = np.cos(pitch) * down0 - np.sin(pitch) * forward0
    R = np.stack([right, down, forward])
    return R, -R @ np.array([x, y, CAM_HEIGHT_M])


def _render_depth(R, t):
    """Z-depth image of the room and box by ray casting."""
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - W / 2) / FOCAL, (v - H / 2) / FOCAL, np.ones_like(u, float)], -1) @ R
    rays = rays + 1e-12
    origin = -R.T @ t
    room = np.minimum.reduce(np.maximum((ROOM_LO - origin) / rays, (ROOM_HI - origin) / rays), axis=-1)
    near = np.maximum.reduce(np.minimum((BOX_LO - origin) / rays, (BOX_HI - origin) / rays), axis=-1)
    far = np.minimum.reduce(np.maximum((BOX_LO - origin) / rays, (BOX_HI - origin) / rays), axis=-1)
    box = np.where((near < far) & (near > 0), near, np.inf)
    return np.minimum(room, box)


def _similarity():
    """Arbitrary rotation and offset standing in for the model's own world frame."""
    a, b = np.radians(20.0), np.radians(35.0)
    rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    rz = np.array([[np.cos(b), -np.sin(b), 0], [np.sin(b), np.cos(b), 0], [0, 0, 1]])
    return rz @ rx, np.array([0.3, -0.2, 0.1])


@pytest.fixture(scope="module")
def scene():
    rng = np.random.default_rng(7)
    Q, offset = _similarity()
    depth, w2c, records = [], [], []
    for i, (x, y, yaw) in enumerate(PATH):
        R, t = _camera(x, y, yaw)
        metric_depth = _render_depth(R, t)
        depth.append(metric_depth / MODEL_SCALE * (1 + 0.005 * rng.standard_normal(metric_depth.shape)))
        Rm = R @ Q.T
        w2c.append(np.column_stack([Rm, t / MODEL_SCALE - Rm @ offset]))

        moves = []
        if i:
            px, py, pyaw = PATH[i - 1]
            dist, turn = np.hypot(x - px, y - py), yaw - pyaw
            if dist:
                moves.append({"direction": "forward", "speed": 1.0,
                              "duration_s": 1000 * dist / FORWARD_MM_S})
            if turn:
                moves.append({"direction": "rotate_left", "speed": 1.0,
                              "duration_s": turn / TURN_DEG_S})
        records.append({
            "idx": i, "file": f"frames/{i:06d}.jpg",
            "imu": {"pitch": 0.0, "roll": 0.0, "yaw": -yaw + 0.2 * i},  # opposite sign, slow drift
            "sonar_mm": int(1000 * np.median(metric_depth[H // 2 - 9:H // 2 + 9,
                                                          W // 2 - 12:W // 2 + 12])),
            "moves": moves,
        })
    K = np.array([[FOCAL, 0, W / 2], [0, FOCAL, H / 2], [0, 0, 1.0]])
    pred = Prediction(
        depth=np.array(depth, dtype=np.float32), w2c=np.array(w2c),
        K=np.broadcast_to(K, (len(PATH), 3, 3)), conf=None,
        images=np.full((len(PATH), H, W, 3), 128, dtype=np.uint8))
    return pred, records


@pytest.fixture(scope="module")
def recon(scene):
    pred, records = scene
    return reconstruct.solve(pred, records, CAM_HEIGHT_M, stride=1)


def test_metric_scale_from_camera_height(recon):
    assert recon.report["scale_m_per_model_unit"] == pytest.approx(MODEL_SCALE, rel=0.01)
    assert recon.report["camera_height_spread"] < 0.01
    assert recon.report["floor_roughness_mm"] < 5.0


def test_poses_recovered_in_floor_frame(recon):
    truth = np.array(PATH, dtype=float)
    assert np.abs(recon.poses[:, :2] - truth[:, :2]).max() < 0.01
    assert np.abs(recon.poses[:, 2] - CAM_HEIGHT_M).max() < 0.003
    heading_error = (np.degrees(recon.poses[:, 3]) - truth[:, 2] + 180) % 360 - 180
    assert np.abs(heading_error).max() < 0.5


def test_sonar_agrees_with_camera_height_scale(recon):
    assert recon.report["sonar"]["vs_camera_height"] == pytest.approx(1.0, abs=0.02)


def test_heading_matches_gyro_with_fitted_sign(recon):
    yaw = recon.report["yaw"]
    assert yaw["imu_sign"] == -1
    assert yaw["rms_deg"] < 1.0


def test_motion_calibration(recon):
    motion = recon.report["motion"]
    assert motion["forward_mm_per_s"]["median"] == pytest.approx(FORWARD_MM_S, abs=1.0)
    assert motion["turn_deg_per_s"]["median"] == pytest.approx(TURN_DEG_S, abs=0.5)


def test_occupancy_map(recon):
    grid = recon.grid

    def state(x, y):
        row, col = grid.index(x, y)
        inside = 0 <= row < grid.cells.shape[0] and 0 <= col < grid.cells.shape[1]
        return grid.cells[row, col] if inside else geometry.UNKNOWN

    assert state(0.45, 0.1) == geometry.FREE             # floor just ahead of the start
    assert state(1.5, 0.0) == geometry.FREE              # far floor, seen at a grazing angle
    # The box is taller than the camera, so only its near faces are ever seen.
    assert any(state(x, -0.65) == geometry.OCCUPIED for x in (1.38, 1.42))
    assert any(state(1.65, y) == geometry.OCCUPIED for y in (-0.42, -0.38))
    assert state(1.65, -0.65) == geometry.UNKNOWN        # its interior
    assert state(2.2, -0.87) == geometry.UNKNOWN         # floor hidden in the box's shadow
    assert any(state(x, 0.0) == geometry.OCCUPIED for x in (2.95, 3.0, 3.05))   # far wall
    assert all(state(x, 0.0) != geometry.OCCUPIED for x in np.arange(1.0, 2.8, 0.1))
    assert state(3.6, 0.0) == geometry.UNKNOWN           # beyond the wall


def test_floor_fit_needs_something_below_the_cameras():
    points = np.array([[0.0, 0.0, 1.0]] * 50)
    with pytest.raises(ValueError):
        geometry.fit_floor(points, np.zeros_like(points), down_hint=np.array([0.0, 0.0, -1.0]))


def test_load_run_thins_frames_and_carries_moves(tmp_path):
    lines = [json.dumps({"idx": i, "file": f"frames/{i:06d}.jpg", "imu": {}, "sonar_mm": 0,
                         "moves": [{"direction": "forward", "speed": 1.0, "duration_s": float(i)}]})
             for i in range(10)]
    (tmp_path / "frames.jsonl").write_text("\n".join(lines) + "\n")

    assert len(reconstruct.load_run(tmp_path)) == 10
    kept = reconstruct.load_run(tmp_path, max_frames=4)
    assert [r["idx"] for r in kept] == [0, 3, 6, 9]
    assert [m["duration_s"] for m in kept[1]["moves"]] == [1.0, 2.0, 3.0]
    assert [m["duration_s"] for m in kept[3]["moves"]] == [7.0, 8.0, 9.0]


def test_write_outputs(recon, tmp_path):
    pytest.importorskip("PIL")
    from PIL import Image

    reconstruct.write_outputs(tmp_path, recon)
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["frames"] == len(PATH)
    assert len(report["poses"]) == len(PATH)

    ply = (tmp_path / "cloud.ply").read_bytes()
    header, body = ply.split(b"end_header\n", 1)
    vertices = int(header.split(b"element vertex ")[1].split(b"\n")[0])
    assert vertices == len(recon.cloud) and len(body) == vertices * 15

    image = Image.open(tmp_path / "map.png")
    assert image.size == (recon.grid.cells.shape[1] * 8, recon.grid.cells.shape[0] * 8)
    assert "m per model unit" in reconstruct.summary(report)
