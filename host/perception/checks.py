"""
Consistency checks between a visual reconstruction and what the dog measured.

Each check compares the reconstruction against an independent sensor, so
together they say whether the camera pipeline can be trusted for navigation.
"""

import numpy as np

_FORWARD = frozenset({"forward"})
_TURNS = frozenset({"rotate_left", "rotate_right"})


def spread(values: np.ndarray) -> float:
    """Robust relative spread: median absolute deviation over |median|."""
    median = np.median(values)
    return float(np.median(np.abs(values - median)) / abs(median))


def sonar_scale(depth: np.ndarray, sonar_mm: list[int | None], window: float = 0.15) -> dict | None:
    """
    Metres per model unit implied by the sonar.

    The sonar looks roughly along the optical axis, so its range should match
    the model depth at the image centre. This is a second metric anchor,
    independent of the camera-height one. A sonar reading of 0 means no echo.
    """
    n, h, w = depth.shape
    dh, dw = max(1, int(h * window / 2)), max(1, int(w * window / 2))
    centre = depth[:, h // 2 - dh:h // 2 + dh, w // 2 - dw:w // 2 + dw].reshape(n, -1)
    ratios = np.array([mm / 1000.0 / np.median(centre[i])
                       for i, mm in enumerate(sonar_mm) if mm])
    if len(ratios) < 3:
        return None
    return {"scale": float(np.median(ratios)), "spread": spread(ratios), "frames": len(ratios)}


def yaw_agreement(visual_yaw_rad: np.ndarray, imu_yaw_deg: list[float | None]) -> dict | None:
    """
    RMS difference between visual and gyro heading changes across consecutive
    keyframes. Gyro yaw drifts over minutes but is good over one move, and its
    sign convention is not fixed by the docs, so the sign is fitted.
    """
    pairs = [(i, i + 1) for i in range(len(imu_yaw_deg) - 1)
             if imu_yaw_deg[i] is not None and imu_yaw_deg[i + 1] is not None]
    if len(pairs) < 3:
        return None
    a, b = np.array(pairs).T
    visual = _wrap_deg(np.degrees(visual_yaw_rad[b] - visual_yaw_rad[a]))
    imu = np.array(imu_yaw_deg, dtype=float)
    gyro = _wrap_deg(imu[b] - imu[a])
    rms = {sign: float(np.sqrt(np.mean(_wrap_deg(visual - sign * gyro) ** 2))) for sign in (1, -1)}
    sign = min(rms, key=rms.get)
    return {"rms_deg": rms[sign], "imu_sign": sign, "pairs": len(pairs)}


def motion_calibration(poses: np.ndarray, moves: list[list[dict]]) -> dict:
    """
    What one second of a motion command actually does, measured visually.

    poses: (N, 4) x, y, z, yaw from `geometry.planar_poses`
    moves: per keyframe, the motion commands executed since the previous one

    Only keyframe gaps made of a single kind of command are used. These are the
    constants dead reckoning needs between keyframes.
    """
    forward_mm_s, turn_deg_s = [], []
    for i in range(1, len(poses)):
        directions = {m["direction"] for m in moves[i]}
        seconds = sum(m["duration_s"] * m["speed"] for m in moves[i])
        if not directions or seconds <= 0:
            continue
        if directions <= _FORWARD:
            forward_mm_s.append(1000.0 * np.linalg.norm(poses[i, :2] - poses[i - 1, :2]) / seconds)
        elif directions <= _TURNS and len(directions) == 1:
            turned = abs(_wrap_deg(np.degrees(poses[i, 3] - poses[i - 1, 3])))
            turn_deg_s.append(turned / seconds)
    return {"forward_mm_per_s": _summary(forward_mm_s), "turn_deg_per_s": _summary(turn_deg_s)}


def _summary(values: list[float]) -> dict | None:
    if len(values) < 3:
        return None
    arr = np.array(values)
    return {"median": float(np.median(arr)), "spread": spread(arr), "moves": len(arr)}


def _wrap_deg(angle):
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0
