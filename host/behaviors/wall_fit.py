"""
Wall fitting from point cloud data.

Clusters 2D points using grid-based DBSCAN, fits lines to elongated
clusters via PCA, and returns wall segments for 3D rendering.
No external dependencies (no numpy/scipy).
"""

import math
from dataclasses import dataclass


@dataclass
class WallSegment:
    x1: float
    y1: float
    x2: float
    y2: float
    height: float
    confidence: float


def fit_walls(points: list[dict], eps: float = 0.45, min_samples: int = 3,
              min_confidence: float = 0.15, wall_height: float = 0.2) -> list[WallSegment]:
    """
    Cluster 2D points and fit wall segments.

    points: list of {"x", "y", "confidence"} dicts
    eps: DBSCAN neighborhood radius (meters)
    min_samples: minimum points to form a cluster
    min_confidence: filter points below this
    wall_height: height of wall segments (meters)
    """
    # Filter by confidence
    pts = [(p["x"], p["y"], p.get("confidence", 1.0))
           for p in points if p.get("confidence", 1.0) >= min_confidence]

    if len(pts) < min_samples:
        return []

    # DBSCAN via spatial hash grid
    clusters = _dbscan(pts, eps, min_samples)

    walls = []
    for cluster in clusters:
        if len(cluster) < min_samples:
            continue

        # PCA on 2D points
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        confs = [p[2] for p in cluster]

        result = _fit_line(xs, ys)
        if result is None:
            continue

        x1, y1, x2, y2, linearity = result

        # Only produce walls from sufficiently linear clusters
        if linearity < 4.0:
            continue

        avg_conf = sum(confs) / len(confs)
        walls.append(WallSegment(
            x1=round(x1, 3), y1=round(y1, 3),
            x2=round(x2, 3), y2=round(y2, 3),
            height=wall_height,
            confidence=round(avg_conf, 2),
        ))

    return walls


def _dbscan(points: list[tuple], eps: float, min_samples: int) -> list[list[tuple]]:
    """Grid-based DBSCAN. Returns list of clusters (each cluster is a list of points)."""
    # Build spatial hash grid
    cell_size = eps
    grid: dict[tuple, list[int]] = {}
    for i, (x, y, c) in enumerate(points):
        key = (int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))
        grid.setdefault(key, []).append(i)

    def neighbors(idx):
        x, y, _ = points[idx]
        cx = int(math.floor(x / cell_size))
        cy = int(math.floor(y / cell_size))
        result = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((cx + dx, cy + dy), []):
                    if j != idx:
                        jx, jy, _ = points[j]
                        dist = math.sqrt((x - jx) ** 2 + (y - jy) ** 2)
                        if dist <= eps:
                            result.append(j)
        return result

    labels = [-1] * len(points)  # -1 = unvisited
    cluster_id = 0
    clusters = []

    for i in range(len(points)):
        if labels[i] != -1:
            continue
        nbrs = neighbors(i)
        if len(nbrs) < min_samples - 1:
            labels[i] = -2  # noise
            continue

        # Expand cluster
        labels[i] = cluster_id
        cluster = [points[i]]
        queue = list(nbrs)
        visited = {i}

        while queue:
            j = queue.pop(0)
            if j in visited:
                continue
            visited.add(j)

            if labels[j] == -2:
                labels[j] = cluster_id
                cluster.append(points[j])
                continue

            if labels[j] != -1:
                continue

            labels[j] = cluster_id
            cluster.append(points[j])

            j_nbrs = neighbors(j)
            if len(j_nbrs) >= min_samples - 1:
                queue.extend(j_nbrs)

        clusters.append(cluster)
        cluster_id += 1

    return clusters


def _fit_line(xs: list[float], ys: list[float]) -> tuple[float, float, float, float, float] | None:
    """
    PCA-based line fit. Returns (x1, y1, x2, y2, linearity_ratio) or None.
    linearity_ratio = eigenvalue1 / eigenvalue2 (high = very linear).
    """
    n = len(xs)
    if n < 2:
        return None

    # Mean
    mx = sum(xs) / n
    my = sum(ys) / n

    # Covariance matrix elements
    cxx = sum((x - mx) ** 2 for x in xs) / n
    cyy = sum((y - my) ** 2 for y in ys) / n
    cxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / n

    # Eigenvalues of 2x2 symmetric matrix via quadratic formula
    trace = cxx + cyy
    det = cxx * cyy - cxy * cxy
    disc = trace * trace - 4 * det
    if disc < 0:
        disc = 0
    sqrt_disc = math.sqrt(disc)

    e1 = (trace + sqrt_disc) / 2  # larger eigenvalue
    e2 = (trace - sqrt_disc) / 2  # smaller eigenvalue

    if e1 < 1e-10:
        return None

    linearity = e1 / max(e2, 1e-10)

    # Principal eigenvector (for larger eigenvalue)
    if abs(cxy) > 1e-10:
        vx = e1 - cyy
        vy = cxy
    elif cxx >= cyy:
        vx, vy = 1.0, 0.0
    else:
        vx, vy = 0.0, 1.0

    # Normalize
    vlen = math.sqrt(vx * vx + vy * vy)
    if vlen < 1e-10:
        return None
    vx /= vlen
    vy /= vlen

    # Project points onto principal axis to find endpoints
    projections = [(xs[i] - mx) * vx + (ys[i] - my) * vy for i in range(n)]
    t_min = min(projections)
    t_max = max(projections)

    x1 = mx + t_min * vx
    y1 = my + t_min * vy
    x2 = mx + t_max * vx
    y2 = my + t_max * vy

    return (x1, y1, x2, y2, linearity)
