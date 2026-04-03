"""
Wall mesh from point cloud — connect high-confidence boundary points
into ordered chains, then send as vertex strips for 3D rendering.

Replaces the DBSCAN → PCA → merge → snap → fill pipeline with a
simpler approach: points ARE the mesh vertices.
"""

import math
from typing import Optional


def build_wall_chains(points: list[dict], min_confidence: float = 0.2,
                      connect_dist: float = 0.15,
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

        # Build vertex list and average confidence
        vertices = []
        total_conf = 0
        for idx in chain:
            vertices.append([round(pts[idx][0], 3), round(pts[idx][1], 3)])
            total_conf += pts[idx][2]

        chains.append({
            "vertices": vertices,
            "confidence": round(total_conf / len(chain), 2),
            "height": wall_height,
        })

    return chains
