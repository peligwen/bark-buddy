"""
Wall mesh from point cloud — connect high-confidence boundary points
into ordered chains, then send as vertex strips for 3D rendering.

Replaces the DBSCAN → PCA → merge → snap → fill pipeline with a
simpler approach: points ARE the mesh vertices.
"""

import math
from typing import Optional


def build_wall_chains(points: list[dict], min_confidence: float = 0.2,
                      connect_dist: float = 0.15,  # optimal from parameter sweep
                      wall_height: float = 0.2) -> list[dict]:
    """
    Build ordered chains of vertices from 2D points.

    Returns list of chains, each chain is:
    {"vertices": [[x,y], [x,y], ...], "confidence": float, "height": float}
    """
    # Filter by confidence
    pts = [(p["x"], p["y"], p.get("confidence", 1.0))
           for p in points if p.get("confidence", 1.0) >= min_confidence]

    if len(pts) < 2:
        return []

    # Build adjacency: connect each point to its nearest neighbors within connect_dist
    # Then extract chains (sequences of connected points)
    n = len(pts)
    used = [False] * n

    # Spatial hash for fast neighbor lookup
    cell = connect_dist
    grid = {}
    for i, (x, y, c) in enumerate(pts):
        key = (int(math.floor(x / cell)), int(math.floor(y / cell)))
        grid.setdefault(key, []).append(i)

    def neighbors(idx):
        x, y, _ = pts[idx]
        cx = int(math.floor(x / cell))
        cy = int(math.floor(y / cell))
        result = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid.get((cx + dx, cy + dy), []):
                    if j != idx and not used[j]:
                        dist = math.sqrt((x - pts[j][0])**2 + (y - pts[j][1])**2)
                        if dist <= connect_dist:
                            result.append((dist, j))
        result.sort()
        return result

    chains = []

    # Greedy chain building: start from unused points, extend in both directions
    for start in range(n):
        if used[start]:
            continue

        chain = [start]
        used[start] = True

        # Extend forward
        current = start
        while True:
            nbrs = neighbors(current)
            if not nbrs:
                break
            _, nxt = nbrs[0]  # nearest unused neighbor
            chain.append(nxt)
            used[nxt] = True
            current = nxt

        # Extend backward from start
        current = start
        while True:
            nbrs = neighbors(current)
            if not nbrs:
                break
            _, nxt = nbrs[0]
            chain.insert(0, nxt)
            used[nxt] = True
            current = nxt

        if len(chain) < 2:
            continue

        # Build vertex list
        vertices = []
        confs = []
        for idx in chain:
            vertices.append([round(pts[idx][0], 3), round(pts[idx][1], 3)])
            confs.append(pts[idx][2])

        # Split chain at sharp turns (>60° angle change = corner)
        sub_chains = _split_at_corners(vertices, confs, max_angle_deg=60)

        for verts, sub_confs in sub_chains:
            if len(verts) < 2:
                continue
            chains.append({
                "vertices": verts,
                "confidence": round(sum(sub_confs) / len(sub_confs), 2),
                "height": wall_height,
            })

    return chains


def _split_at_corners(vertices, confs, max_angle_deg=60):
    """Split a vertex chain at points where the direction changes sharply."""
    if len(vertices) < 3:
        return [(vertices, confs)]

    splits = []
    current_verts = [vertices[0]]
    current_confs = [confs[0]]

    for i in range(1, len(vertices) - 1):
        current_verts.append(vertices[i])
        current_confs.append(confs[i])

        # Compute angle between segments (i-1→i) and (i→i+1)
        dx1 = vertices[i][0] - vertices[i-1][0]
        dy1 = vertices[i][1] - vertices[i-1][1]
        dx2 = vertices[i+1][0] - vertices[i][0]
        dy2 = vertices[i+1][1] - vertices[i][1]

        len1 = math.sqrt(dx1*dx1 + dy1*dy1)
        len2 = math.sqrt(dx2*dx2 + dy2*dy2)
        if len1 < 1e-6 or len2 < 1e-6:
            continue

        # Dot product for angle
        dot = (dx1*dx2 + dy1*dy2) / (len1 * len2)
        dot = max(-1, min(1, dot))
        angle_deg = math.degrees(math.acos(dot))

        if angle_deg > max_angle_deg:
            # Sharp turn — split here (vertex i belongs to both chains)
            splits.append((current_verts[:], current_confs[:]))
            current_verts = [vertices[i]]
            current_confs = [confs[i]]

    # Last segment
    current_verts.append(vertices[-1])
    current_confs.append(confs[-1])
    splits.append((current_verts, current_confs))

    return splits
