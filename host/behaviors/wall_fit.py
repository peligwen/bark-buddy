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


def fit_walls(points: list[dict], eps: float = None, min_samples: int = 2,
              min_confidence: float = 0.2, wall_height: float = 0.2) -> list[WallSegment]:
    """
    Cluster 2D points and fit wall segments.

    points: list of {"x", "y", "confidence"} dicts
    eps: DBSCAN neighborhood radius (meters). If None, auto-scales with density.
    min_samples: minimum points to form a cluster
    min_confidence: filter points below this
    wall_height: height of wall segments (meters)
    """
    # Filter by confidence
    pts = [(p["x"], p["y"], p.get("confidence", 1.0))
           for p in points if p.get("confidence", 1.0) >= min_confidence]

    if len(pts) < min_samples:
        return []

    # Auto-scale eps: tighter with more points to separate walls at corners
    if eps is None:
        if len(pts) > 150:
            eps = 0.07
        elif len(pts) > 80:
            eps = 0.10
        elif len(pts) > 30:
            eps = 0.15
        else:
            eps = 0.25

    # DBSCAN via spatial hash grid
    clusters = _dbscan(pts, eps, min_samples)

    walls = []
    for cluster in clusters:
        _extract_walls(cluster, walls, min_samples, wall_height)

    # Merge collinear nearby wall segments
    walls = _merge_collinear(walls)

    # Refit long walls against the underlying point cloud
    walls = _refit_walls(walls, pts)

    # Snap nearby endpoints to form clean corners
    walls = _snap_corners(walls)

    return walls


def _extract_walls(cluster: list[tuple], walls: list[WallSegment],
                   min_samples: int, wall_height: float, depth: int = 0):
    """Recursively extract wall segments from a cluster.
    Splits non-linear clusters at the point of max deviation from the PCA line."""
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
        # Linear enough — emit as a wall
        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if length > 0.05:
            avg_conf = sum(confs) / len(confs)
            walls.append(WallSegment(
                x1=round(x1, 3), y1=round(y1, 3),
                x2=round(x2, 3), y2=round(y2, 3),
                height=wall_height,
                confidence=round(avg_conf, 2),
            ))
        return

    # Non-linear: split at the point with maximum deviation from the PCA line
    # Line direction
    lx, ly = x2 - x1, y2 - y1
    line_len = math.sqrt(lx * lx + ly * ly)
    if line_len < 1e-6:
        return
    nx, ny = -ly / line_len, lx / line_len  # normal to line

    # Project each point and find max deviation
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    projections = [((xs[i] - mx) * (lx / line_len) + (ys[i] - my) * (ly / line_len))
                   for i in range(len(xs))]
    deviations = [abs((xs[i] - mx) * nx + (ys[i] - my) * ny)
                  for i in range(len(xs))]

    # Sort by projection along the line and split at the point of max deviation
    indexed = sorted(range(len(cluster)), key=lambda i: projections[i])
    max_dev_idx = max(range(len(indexed)), key=lambda i: deviations[indexed[i]])

    # Split into two halves at the deviation point
    if max_dev_idx < 2 or max_dev_idx > len(indexed) - 3:
        # Can't split meaningfully near the edges
        return

    left = [cluster[indexed[i]] for i in range(max_dev_idx)]
    right = [cluster[indexed[i]] for i in range(max_dev_idx, len(indexed))]

    _extract_walls(left, walls, min_samples, wall_height, depth + 1)
    _extract_walls(right, walls, min_samples, wall_height, depth + 1)


def _merge_collinear(walls: list[WallSegment], angle_thresh: float = 0.3,
                     gap_thresh: float = 0.20) -> list[WallSegment]:
    """Merge wall segments that are nearly parallel and close together."""
    if len(walls) < 2:
        return walls

    def wall_angle(w):
        return math.atan2(w.y2 - w.y1, w.x2 - w.x1)

    def wall_length(w):
        return math.sqrt((w.x2 - w.x1)**2 + (w.y2 - w.y1)**2)

    def endpoints_gap(a, b):
        """Min distance between any endpoint pair of two segments."""
        pts_a = [(a.x1, a.y1), (a.x2, a.y2)]
        pts_b = [(b.x1, b.y1), (b.x2, b.y2)]
        return min(math.sqrt((pa[0]-pb[0])**2 + (pa[1]-pb[1])**2)
                   for pa in pts_a for pb in pts_b)

    def merge_two(a, b):
        """Merge two segments into one spanning all 4 endpoints along the average direction."""
        all_pts = [(a.x1, a.y1), (a.x2, a.y2), (b.x1, b.y1), (b.x2, b.y2)]
        # Average direction
        ang = (wall_angle(a) + wall_angle(b)) / 2
        dx, dy = math.cos(ang), math.sin(ang)
        # Project all points onto this direction
        projections = [(p[0]*dx + p[1]*dy, p) for p in all_pts]
        projections.sort()
        p_min = projections[0][1]
        p_max = projections[-1][1]
        avg_conf = (a.confidence * wall_length(a) + b.confidence * wall_length(b)) / max(
            wall_length(a) + wall_length(b), 0.01)
        return WallSegment(
            x1=round(p_min[0], 3), y1=round(p_min[1], 3),
            x2=round(p_max[0], 3), y2=round(p_max[1], 3),
            height=a.height, confidence=round(avg_conf, 2))

    # Greedy merging — keep merging until no more pairs match
    changed = True
    while changed:
        changed = False
        new_walls = []
        used = set()
        for i in range(len(walls)):
            if i in used:
                continue
            merged = False
            for j in range(i + 1, len(walls)):
                if j in used:
                    continue
                # Check angle similarity (handle wrapping)
                da = abs(wall_angle(walls[i]) - wall_angle(walls[j]))
                if da > math.pi:
                    da = 2 * math.pi - da
                # Allow near-parallel (same direction) or anti-parallel
                if da > angle_thresh and abs(da - math.pi) > angle_thresh:
                    continue
                # Check proximity
                if endpoints_gap(walls[i], walls[j]) > gap_thresh:
                    continue
                # Merge
                new_walls.append(merge_two(walls[i], walls[j]))
                used.add(i)
                used.add(j)
                merged = True
                changed = True
                break
            if not merged:
                new_walls.append(walls[i])
        walls = new_walls

    return walls


def _refit_walls(walls: list[WallSegment], pts: list[tuple],
                  proximity: float = 0.10) -> list[WallSegment]:
    """Refit each wall against nearby source points for better accuracy."""
    if not walls or not pts:
        return walls

    result = []
    for w in walls:
        # Find points within proximity of this wall segment
        nearby = []
        for p in pts:
            d = _point_to_segment_dist(p[0], p[1], w.x1, w.y1, w.x2, w.y2)
            if d < proximity:
                nearby.append(p)

        if len(nearby) < 2:
            result.append(w)
            continue

        # Refit line through the nearby points
        xs = [p[0] for p in nearby]
        ys = [p[1] for p in nearby]
        fit = _fit_line(xs, ys)
        if fit and fit[4] >= 2.0:  # still linear after refit
            result.append(WallSegment(
                x1=round(fit[0], 3), y1=round(fit[1], 3),
                x2=round(fit[2], 3), y2=round(fit[3], 3),
                height=w.height,
                confidence=w.confidence,
            ))
        else:
            result.append(w)

    return result


def _snap_corners(walls: list[WallSegment], snap_dist: float = 0.25) -> list[WallSegment]:
    """Extend nearby wall endpoints to meet at their line intersection, forming corners."""
    if len(walls) < 2:
        return walls

    # For each wall, store as mutable list [x1,y1,x2,y2,h,c]
    mwalls = [[w.x1, w.y1, w.x2, w.y2, w.height, w.confidence] for w in walls]
    used_pairs = set()

    # Find pairs of endpoints from different walls that are close
    for i in range(len(mwalls)):
        for j in range(i + 1, len(mwalls)):
            if (i, j) in used_pairs:
                continue

            # Check all 4 endpoint pairs between wall i and wall j
            best_dist = snap_dist
            best_ei = -1  # 0=start, 1=end of wall i
            best_ej = -1

            for ei in (0, 1):
                px = mwalls[i][ei * 2]
                py = mwalls[i][ei * 2 + 1]
                for ej in (0, 1):
                    qx = mwalls[j][ej * 2]
                    qy = mwalls[j][ej * 2 + 1]
                    d = math.sqrt((px - qx)**2 + (py - qy)**2)
                    if d < best_dist:
                        best_dist = d
                        best_ei = ei
                        best_ej = ej

            if best_ei < 0:
                continue

            # Compute intersection of the two wall lines
            ix, iy = _line_intersection(
                mwalls[i][0], mwalls[i][1], mwalls[i][2], mwalls[i][3],
                mwalls[j][0], mwalls[j][1], mwalls[j][2], mwalls[j][3])

            if ix is not None:
                # Check the intersection isn't too far from the endpoints
                ep_i = (mwalls[i][best_ei * 2], mwalls[i][best_ei * 2 + 1])
                ep_j = (mwalls[j][best_ej * 2], mwalls[j][best_ej * 2 + 1])
                di = math.sqrt((ix - ep_i[0])**2 + (iy - ep_i[1])**2)
                dj = math.sqrt((ix - ep_j[0])**2 + (iy - ep_j[1])**2)

                if di < snap_dist * 2 and dj < snap_dist * 2:
                    # Extend both wall endpoints to the intersection
                    mwalls[i][best_ei * 2] = round(ix, 3)
                    mwalls[i][best_ei * 2 + 1] = round(iy, 3)
                    mwalls[j][best_ej * 2] = round(ix, 3)
                    mwalls[j][best_ej * 2 + 1] = round(iy, 3)
                    used_pairs.add((i, j))

    return [WallSegment(x1=w[0], y1=w[1], x2=w[2], y2=w[3],
                         height=w[4], confidence=w[5]) for w in mwalls]


def _line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    """Find intersection point of two infinite lines. Returns (x,y) or (None,None)."""
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3
    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-9:
        return None, None  # parallel
    t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denom
    ix = x1 + t * dx1
    iy = y1 + t * dy1
    return ix, iy


def _point_to_segment_dist(px, py, x1, y1, x2, y2):
    """Distance from point to line segment."""
    dx, dy = x2 - x1, y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-10:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / len_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


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
