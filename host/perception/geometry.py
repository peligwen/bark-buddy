"""
Geometry on reconstruction-model output.

The model returns per-frame depth and world-to-camera poses in its own
arbitrary frame and scale. These functions find the floor, turn the known
camera height into a metric scale, and re-express everything in a
floor-aligned frame: origin on the floor under the first camera, x along its
heading, y to its left, z up, metres.

Camera convention is OpenCV (x right, y down, z forward). `w2c` is (N, 3, 4)
and maps world points to camera points as X_c = R·X_w + t.
"""

from dataclasses import dataclass

import numpy as np

FREE, OCCUPIED, UNKNOWN = 0, 1, -1


def camera_centers(w2c: np.ndarray) -> np.ndarray:
    """(N, 3) camera positions in world coordinates."""
    return -np.einsum("nij,ni->nj", w2c[:, :, :3], w2c[:, :, 3])


def backproject(depth: np.ndarray, K: np.ndarray, w2c: np.ndarray, stride: int = 1) -> np.ndarray:
    """(M, 3) world points for every `stride`-th pixel of one frame, row-major."""
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    x = (u - K[0, 2]) / K[0, 0] * z
    y = (v - K[1, 2]) / K[1, 1] * z
    in_camera = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    return (in_camera - w2c[:, 3]) @ w2c[:, :3]


@dataclass(frozen=True)
class Floor:
    normal: np.ndarray  # unit vector pointing down, from the cameras toward the floor
    offset: float       # normal · X == offset for points on the floor

    def height(self, points: np.ndarray) -> np.ndarray:
        """Height above the floor, in the units of `points`."""
        return self.offset - points @ self.normal


def fit_floor(points: np.ndarray, origins: np.ndarray, down_hint: np.ndarray) -> Floor:
    """
    Fit the floor plane.

    points:     (M, 3) world points
    origins:    (M, 3) centre of the camera that saw each point
    down_hint:  approximate down direction (the cameras' mean y axis)

    Every camera rides at the same height, so floor points all sit the same
    distance below the camera that saw them, while walls and furniture are
    spread over all distances. The floor is the peak of that distribution.
    """
    down_hint = down_hint / np.linalg.norm(down_hint)
    drop = np.einsum("ij,j->i", points - origins, down_hint)
    below = drop > 0
    if below.sum() < 100:
        raise ValueError("no surface below the cameras; cannot find the floor")

    counts, edges = np.histogram(drop[below], bins=100,
                                 range=(0.0, float(np.percentile(drop[below], 99))))
    peak = int(np.argmax(counts))
    drop0 = 0.5 * (edges[peak] + edges[peak + 1])

    normal, centroid = _plane(points[below & (np.abs(drop - drop0) < 0.25 * drop0)], down_hint)
    # Second pass against the fitted normal, so a tilted camera mount (down_hint
    # off by the mount pitch) does not bias which points count as floor.
    drop = np.einsum("ij,j->i", points - origins, normal)
    distance = (points - centroid) @ normal
    normal, centroid = _plane(points[(drop > 0) & (np.abs(distance) < 0.1 * drop0)], normal)
    return Floor(normal=normal, offset=float(normal @ centroid))


def _plane(points: np.ndarray, toward: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares plane through `points`; normal oriented along `toward`."""
    centroid = points.mean(axis=0)
    centred = points - centroid
    _, vectors = np.linalg.eigh(centred.T @ centred)
    normal = vectors[:, 0]
    if normal @ toward < 0:
        normal = -normal
    return normal, centroid


@dataclass(frozen=True)
class FloorFrame:
    rotation: np.ndarray  # (3, 3); rows are the frame's x, y, z axes in model coordinates
    origin: np.ndarray    # (3,) model coordinates of the frame origin
    scale: float          # metres per model unit

    def apply(self, points: np.ndarray) -> np.ndarray:
        return self.scale * (points - self.origin) @ self.rotation.T


def floor_frame(floor: Floor, w2c: np.ndarray, cam_height_m: float) -> FloorFrame:
    """Metric, floor-aligned frame anchored at the first camera."""
    centers = camera_centers(w2c)
    heights = floor.height(centers)
    z = -floor.normal
    forward = w2c[0, 2, :3]
    x = forward - (forward @ z) * z
    x /= np.linalg.norm(x)
    return FloorFrame(
        rotation=np.stack([x, np.cross(z, x), z]),
        origin=centers[0] - heights[0] * z,
        scale=cam_height_m / float(np.median(heights)),
    )


def planar_poses(frame: FloorFrame, w2c: np.ndarray) -> np.ndarray:
    """(N, 4) camera x, y, z in metres and heading in radians, in the floor frame."""
    position = frame.apply(camera_centers(w2c))
    forward = w2c[:, 2, :3] @ frame.rotation.T
    return np.column_stack([position, np.arctan2(forward[:, 1], forward[:, 0])])


@dataclass(frozen=True)
class Grid:
    cells: np.ndarray            # (rows, cols) int8 of FREE / OCCUPIED / UNKNOWN; row ↔ y, col ↔ x
    origin: tuple[float, float]  # floor-frame (x, y) of the corner of cells[0, 0]
    cell_m: float

    def index(self, x: float, y: float) -> tuple[int, int]:
        return (int((y - self.origin[1]) // self.cell_m),
                int((x - self.origin[0]) // self.cell_m))


def occupancy_grid(points: np.ndarray, origins: np.ndarray, cell_m: float = 0.05,
                   floor_tol_m: float = 0.02, obstacle_m: tuple[float, float] = (0.03, 0.25),
                   min_points: int = 3) -> Grid:
    """
    Top-down map from floor-frame points (metres) and the camera each was seen from.

    A cell is OCCUPIED when enough points fall in the height band the dog's
    body sweeps (taller than a step it cannot climb, lower than its back).
    A cell is FREE when enough sight lines to floor points cross it: from a
    camera a hand's width off the ground the floor is seen at a grazing angle,
    so the points themselves land in sparse stripes and would leave gaps.
    Everything else is UNKNOWN, including floor hidden behind obstacles.
    """
    z = points[:, 2]
    on_floor = np.abs(z) <= floor_tol_m
    blocking = (z >= obstacle_m[0]) & (z <= obstacle_m[1])
    used = points[on_floor | blocking, :2]
    lo = np.percentile(used, 0.5, axis=0) - 2 * cell_m
    hi = np.percentile(used, 99.5, axis=0) + 2 * cell_m
    cols, rows = np.ceil((hi - lo) / cell_m).astype(int)

    def count(xy: np.ndarray) -> np.ndarray:
        ij = np.floor((xy - lo) / cell_m).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < cols) & (ij[:, 1] >= 0) & (ij[:, 1] < rows)
        flat = ij[ok, 1] * cols + ij[ok, 0]
        return np.bincount(flat, minlength=rows * cols).reshape(rows, cols)

    cells = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    cells[_sight_line_counts(points[on_floor, :2], origins[on_floor, :2], cell_m, count)
          >= min_points] = FREE
    cells[count(points[blocking, :2]) >= min_points] = OCCUPIED
    return Grid(cells=cells, origin=(float(lo[0]), float(lo[1])), cell_m=cell_m)


def _sight_line_counts(hits: np.ndarray, origins: np.ndarray, cell_m: float, count,
                       max_rays: int = 100_000, chunk: int = 10_000) -> np.ndarray:
    """Per-cell number of samples along the camera→floor-hit segments."""
    if len(hits) > max_rays:
        pick = np.random.default_rng(0).choice(len(hits), max_rays, replace=False)
        hits, origins = hits[pick], origins[pick]
    longest = float(np.linalg.norm(hits - origins, axis=1).max())
    steps = np.linspace(0.0, 1.0, int(np.ceil(longest / cell_m)) + 1)[:, None, None]
    total = 0
    for i in range(0, len(hits), chunk):
        a, b = origins[i:i + chunk], hits[i:i + chunk]
        total = total + count((a + steps * (b - a)).reshape(-1, 2))
    return total
