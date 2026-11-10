"""
Offline reconstruction of a capture run.

Turns the keyframes saved by `behaviors/capture.py` into a metric,
floor-aligned reconstruction and a report that says whether the camera
pipeline is good enough to navigate on:

    recon/report.json   scale, consistency checks, motion calibration, poses
    recon/map.png       top-down occupancy map with the keyframe path
    recon/cloud.ply     coloured point cloud, metres, floor at z = 0
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from perception import checks, geometry
from perception.da3 import DEFAULT_MODEL, Prediction

_CONF_PERCENTILE = 40   # drop each frame's least confident pixels (depth edges, glare)
_MAX_CLOUD_POINTS = 2_000_000
_MAP_PX_PER_CELL = 8


@dataclass(frozen=True)
class Reconstruction:
    poses: np.ndarray       # (N, 4) x, y, z metres and heading radians, floor frame
    grid: geometry.Grid
    cloud: np.ndarray       # (M, 3) metres, floor frame
    colors: np.ndarray      # (M, 3) uint8
    report: dict


def load_run(run_dir: Path, max_frames: int | None = None) -> list[dict]:
    """Keyframe records, evenly thinned to `max_frames`, moves of dropped frames carried forward."""
    records = [json.loads(line) for line in (run_dir / "frames.jsonl").read_text().splitlines()]
    if not max_frames or len(records) <= max_frames:
        return records
    keep = set(np.linspace(0, len(records) - 1, max_frames).round().astype(int).tolist())
    kept, carried = [], []
    for i, record in enumerate(records):
        carried += record["moves"]
        if i in keep:
            kept.append({**record, "moves": carried})
            carried = []
    return kept


def solve(pred: Prediction, records: list[dict], cam_height_m: float, stride: int = 2) -> Reconstruction:
    """Model output + keyframe telemetry → metric reconstruction and report."""
    centers = geometry.camera_centers(pred.w2c)
    points, origins, colors = [], [], []
    for i in range(len(pred.depth)):
        depth = pred.depth[i]
        keep = np.isfinite(depth) & (depth > 0)
        if pred.conf is not None:
            keep &= pred.conf[i] >= np.percentile(pred.conf[i], _CONF_PERCENTILE)
        keep = keep[::stride, ::stride].ravel()
        pts = geometry.backproject(depth, pred.K[i], pred.w2c[i], stride)[keep]
        points.append(pts)
        origins.append(np.broadcast_to(centers[i], pts.shape))
        colors.append(pred.images[i, ::stride, ::stride].reshape(-1, 3)[keep])
    points, origins, colors = np.concatenate(points), np.concatenate(origins), np.concatenate(colors)

    # The camera is mounted upright on a standing dog, so its y axis is roughly down.
    floor = geometry.fit_floor(points, origins, down_hint=pred.w2c[:, 1, :3].mean(axis=0))
    frame = geometry.floor_frame(floor, pred.w2c, cam_height_m)
    poses = geometry.planar_poses(frame, pred.w2c)
    cloud = frame.apply(points)
    grid = geometry.occupancy_grid(cloud, frame.apply(origins))

    sonar = checks.sonar_scale(pred.depth, [r.get("sonar_mm") for r in records])
    if sonar:
        sonar["vs_camera_height"] = sonar["scale"] / frame.scale
    near_floor = cloud[np.abs(cloud[:, 2]) < 0.05, 2]
    report = {
        "frames": len(records),
        "camera_height_m": cam_height_m,
        "scale_m_per_model_unit": frame.scale,
        "camera_height_spread": checks.spread(floor.height(centers)),
        "floor_roughness_mm": float(1000 * np.median(np.abs(near_floor))),
        "sonar": sonar,
        "yaw": checks.yaw_agreement(poses[:, 3], [r["imu"].get("yaw") for r in records]),
        "motion": checks.motion_calibration(poses, [r["moves"] for r in records]),
        "path_length_m": float(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1).sum()),
        "map": {
            "cell_m": grid.cell_m,
            "free_m2": float((grid.cells == geometry.FREE).sum() * grid.cell_m ** 2),
            "occupied_cells": int((grid.cells == geometry.OCCUPIED).sum()),
        },
        "poses": [
            {"idx": r["idx"], "x": round(float(p[0]), 4), "y": round(float(p[1]), 4),
             "z": round(float(p[2]), 4), "yaw_deg": round(float(np.degrees(p[3])), 2)}
            for r, p in zip(records, poses)
        ],
    }
    return Reconstruction(poses=poses, grid=grid, cloud=cloud, colors=colors, report=report)


def write_outputs(out_dir: Path, recon: Reconstruction) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "report.json").write_text(json.dumps(recon.report, indent=2) + "\n")
    _write_ply(out_dir / "cloud.ply", recon.cloud, recon.colors)
    _draw_map(recon.grid, recon.poses).save(out_dir / "map.png")


def summary(report: dict) -> str:
    def pct(value):
        return f"{100 * value:.1f}%"

    lines = [
        f"Keyframes:            {report['frames']}  (path {report['path_length_m']:.2f} m)",
        f"Metric scale:         {report['scale_m_per_model_unit']:.4f} m per model unit",
        f"Camera height spread: {pct(report['camera_height_spread'])}"
        "   (every keyframe should sit at the same height)",
        f"Floor roughness:      {report['floor_roughness_mm']:.1f} mm",
    ]
    sonar = report["sonar"]
    lines.append("Sonar vs camera:      " + (
        f"{sonar['vs_camera_height']:.2f}x   (1.00 = both scale anchors agree; "
        f"{sonar['frames']} frames, spread {pct(sonar['spread'])})"
        if sonar else "needs 3+ keyframes with a sonar echo"))
    yaw = report["yaw"]
    lines.append("Heading vs gyro:      " + (
        f"{yaw['rms_deg']:.1f} deg RMS over {yaw['pairs']} moves (gyro sign {yaw['imu_sign']:+d})"
        if yaw else "needs 3+ consecutive keyframes with gyro yaw"))
    for label, key, unit in (("Forward speed:        ", "forward_mm_per_s", "mm/s"),
                             ("Turn rate:            ", "turn_deg_per_s", "deg/s")):
        value = report["motion"][key]
        lines.append(label + (
            f"{value['median']:.1f} {unit} at speed 1.0 "
            f"(spread {pct(value['spread'])}, {value['moves']} moves)"
            if value else "needs 3+ keyframe gaps made of only that move"))
    lines.append(f"Map:                  {report['map']['free_m2']:.2f} m2 free floor, "
                 f"{report['map']['occupied_cells']} occupied cells")
    return "\n".join(lines)


def run(run_dir: Path, cam_height_m: float, model_id: str | None = None, device: str = "cuda",
        max_frames: int | None = 60, process_res: int = 504) -> Reconstruction:
    from perception import da3

    records = load_run(run_dir, max_frames)
    if len(records) < 3:
        raise SystemExit(f"{run_dir}: need at least 3 keyframes, found {len(records)}")
    pred = da3.infer([run_dir / r["file"] for r in records], model_id or DEFAULT_MODEL,
                     device, process_res)
    recon = solve(pred, records, cam_height_m)
    write_outputs(run_dir / "recon", recon)
    return recon


def _write_ply(path: Path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    if len(xyz) > _MAX_CLOUD_POINTS:
        pick = np.random.default_rng(0).choice(len(xyz), _MAX_CLOUD_POINTS, replace=False)
        xyz, rgb = xyz[pick], rgb[pick]
    vertices = np.empty(len(xyz), dtype=[("xyz", "<f4", 3), ("rgb", "u1", 3)])
    vertices["xyz"], vertices["rgb"] = xyz, rgb
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(xyz)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(vertices.tobytes())


def _draw_map(grid: geometry.Grid, poses: np.ndarray):
    """Occupancy map, +x right and +y up: white free, black occupied, grey unknown."""
    from PIL import Image, ImageDraw

    shade = np.full(grid.cells.shape, 110, dtype=np.uint8)
    shade[grid.cells == geometry.FREE] = 240
    shade[grid.cells == geometry.OCCUPIED] = 15
    px = _MAP_PX_PER_CELL
    image = Image.fromarray(np.flipud(shade)).convert("RGB").resize(
        (shade.shape[1] * px, shade.shape[0] * px), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def to_px(x, y):
        return ((x - grid.origin[0]) / grid.cell_m * px,
                image.height - (y - grid.origin[1]) / grid.cell_m * px)

    path = [to_px(x, y) for x, y in poses[:, :2]]
    draw.line(path, fill=(0, 110, 255), width=2)
    for i, ((cx, cy), yaw) in enumerate(zip(path, poses[:, 3])):
        color = (0, 170, 60) if i == 0 else (0, 110, 255)
        draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=color)
        draw.line([cx, cy, cx + 10 * np.cos(yaw), cy - 10 * np.sin(yaw)], fill=(230, 60, 40), width=2)
    return image
