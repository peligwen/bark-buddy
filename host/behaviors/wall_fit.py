"""
Wall fitting from point cloud data.

Clusters 2D points using grid-based DBSCAN, fits lines to elongated
clusters via PCA. This is the legacy fallback — the primary wall
rendering now uses wall_mesh.py (point cloud chains).
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


def fit_walls(points: list[dict], eps: float = None, min_samples: int = 2,
              min_confidence: float = 0.2, wall_height: float = 0.2) -> list[WallSegment]:
    """Cluster 2D points and fit wall segments (legacy fallback)."""
    pts = [(p["x"], p["y"], p.get("confidence", 1.0))
           for p in points if p.get("confidence", 1.0) >= min_confidence]

    if len(pts) < min_samples:
        return []

    if eps is None:
        if len(pts) > 150:
            eps = 0.07
        elif len(pts) > 80:
            eps = 0.10
        elif len(pts) > 30:
            eps = 0.15
        else:
            eps = 0.25

    clusters = _dbscan(pts, eps, min_samples)

    walls = []
    for cluster in clusters:
        _extract_walls(cluster, walls, min_samples, wall_height)

    return walls


def _extract_walls(cluster, walls, min_samples, wall_height, depth=0):
    if len(cluster) < min_samples or depth > 6:
        return

    xs = [p[0] for p in cluster]
    ys = [p[1] for p in cluster]
    confs = [p[2] for p in cluster]

    result = _fit_line(xs, ys)
    if result is None:
        return

    x1, y1, x2, y2, linearity = result

    if linearity >= 2.5:
        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if length > 0.05:
            avg_conf = sum(confs) / len(confs)
            walls.append(WallSegment(
                x1=round(x1, 3), y1=round(y1, 3),
                x2=round(x2, 3), y2=round(y2, 3),
                height=wall_height, confidence=round(avg_conf, 2)))
        return

    lx, ly = x2 - x1, y2 - y1
    line_len = math.sqrt(lx * lx + ly * ly)
    if line_len < 1e-6:
        return
    nx, ny = -ly / line_len, lx / line_len

    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    projections = [((xs[i] - mx) * (lx / line_len) + (ys[i] - my) * (ly / line_len))
                   for i in range(len(xs))]
    deviations = [abs((xs[i] - mx) * nx + (ys[i] - my) * ny)
                  for i in range(len(xs))]

    indexed = sorted(range(len(cluster)), key=lambda i: projections[i])
    max_dev_idx = max(range(len(indexed)), key=lambda i: deviations[indexed[i]])

    if max_dev_idx < 2 or max_dev_idx > len(indexed) - 3:
        return

    left = [cluster[indexed[i]] for i in range(max_dev_idx)]
    right = [cluster[indexed[i]] for i in range(max_dev_idx, len(indexed))]

    _extract_walls(left, walls, min_samples, wall_height, depth + 1)
    _extract_walls(right, walls, min_samples, wall_height, depth + 1)


def _dbscan(points, eps, min_samples):
    cell_size = eps
    grid = {}
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
                        dist = math.sqrt((x - points[j][0])**2 + (y - points[j][1])**2)
                        if dist <= eps:
                            result.append(j)
        return result

    labels = [-1] * len(points)
    cluster_id = 0
    clusters = []

    for i in range(len(points)):
        if labels[i] != -1:
            continue
        nbrs = neighbors(i)
        if len(nbrs) < min_samples - 1:
            labels[i] = -2
            continue

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


def _fit_line(xs, ys):
    n = len(xs)
    if n < 2:
        return None

    mx = sum(xs) / n
    my = sum(ys) / n

    cxx = sum((x - mx)**2 for x in xs) / n
    cyy = sum((y - my)**2 for y in ys) / n
    cxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / n

    trace = cxx + cyy
    det = cxx * cyy - cxy * cxy
    disc = max(0, trace * trace - 4 * det)
    sqrt_disc = math.sqrt(disc)

    e1 = (trace + sqrt_disc) / 2
    e2 = (trace - sqrt_disc) / 2

    if e1 < 1e-10:
        return None

    linearity = e1 / max(e2, 1e-10)

    if abs(cxy) > 1e-10:
        vx, vy = e1 - cyy, cxy
    elif cxx >= cyy:
        vx, vy = 1.0, 0.0
    else:
        vx, vy = 0.0, 1.0

    vlen = math.sqrt(vx * vx + vy * vy)
    if vlen < 1e-10:
        return None
    vx /= vlen
    vy /= vlen

    projections = [(xs[i] - mx) * vx + (ys[i] - my) * vy for i in range(n)]
    t_min = min(projections)
    t_max = max(projections)

    return (mx + t_min * vx, my + t_min * vy,
            mx + t_max * vx, my + t_max * vy, linearity)
