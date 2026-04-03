#!/usr/bin/env python3
"""
Parameter sweep for wall mesh chain generation.
Tests merge_radius, consolidate_radius, and connect_dist against
known room geometries to find the most accurate representation.
"""

import sys
import math
sys.path.insert(0, "host")

from behaviors.octree import PointCloud
from behaviors.wall_mesh import build_wall_chains

# ============================================================
# Ground truth rooms
# ============================================================

TRIANGLE_WALLS = [
    (1.5, 0.0, -0.75, 1.3),
    (-0.75, 1.3, -0.75, -1.3),
    (-0.75, -1.3, 1.5, 0.0),
    (-0.3, 0.5, -0.3, -0.5),
]

RECTANGLE_WALLS = [
    (-1, -0.5, 1, -0.5),
    (1, -0.5, 1, 0.5),
    (1, 0.5, -1, 0.5),
    (-1, 0.5, -1, -0.5),
]

LSHAPE_WALLS = [
    (0, 0, 2, 0),
    (2, 0, 2, 0.5),
    (2, 0.5, 1, 0.5),
    (1, 0.5, 1, 1),
    (1, 1, 0, 1),
    (0, 1, 0, 0),
]


def raycast(ox, oy, angle_deg, walls):
    rad = math.radians(angle_deg)
    dx, dy = math.cos(rad), math.sin(rad)
    min_dist = 3.0
    for w in walls:
        sx, sy = w[2] - w[0], w[3] - w[1]
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-9:
            continue
        t = ((w[0] - ox) * sy - (w[1] - oy) * sx) / denom
        s = ((w[0] - ox) * dy - (w[1] - oy) * dx) / denom
        if t > 0 and 0 <= s <= 1 and t < min_dist:
            min_dist = t
    return min_dist


def point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-10:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / len_sq))
    return math.sqrt((px - x1 - t * dx)**2 + (py - y1 - t * dy)**2)


def point_to_room_dist(px, py, walls):
    return min(point_to_segment_dist(px, py, *w) for w in walls)


# ============================================================
# Build point cloud from scan positions
# ============================================================

def build_cloud(walls, positions, merge_radius, consolidate_radius, noise_mm=5):
    import random
    random.seed(42)
    cloud = PointCloud()
    cloud._merge_radius = merge_radius
    cloud._consolidate_radius = consolidate_radius
    cloud._consolidate_interval = 0  # allow immediate

    for px, py in positions:
        for angle in range(0, 360, 5):
            dist = raycast(px, py, angle, walls)
            if dist < 3.0:
                dist_mm = int(dist * 1000) + int(random.gauss(0, noise_mm))
                rad = math.radians(angle)
                x = px + (dist_mm / 1000.0) * math.cos(rad)
                y = py + (dist_mm / 1000.0) * math.sin(rad)
                cloud.add_point(x, y, 0.09, max(10, dist_mm))

    cloud.consolidate()
    return cloud


# ============================================================
# Score a chain set against ground truth
# ============================================================

def score_chains(chains, truth_walls):
    """Score how well chains represent the true room geometry.
    Returns dict with multiple metrics."""
    if not chains:
        return {"coverage": 0, "accuracy": 0, "total_length": 0, "chain_count": 0, "score": 0}

    # Total chain length
    total_length = 0
    all_chain_points = []
    for c in chains:
        verts = c["vertices"]
        for i in range(1, len(verts)):
            dx = verts[i][0] - verts[i-1][0]
            dy = verts[i][1] - verts[i-1][1]
            total_length += math.sqrt(dx*dx + dy*dy)
        all_chain_points.extend(verts)

    # Accuracy: how close are chain points to the true walls?
    if all_chain_points:
        errors = [point_to_room_dist(p[0], p[1], truth_walls) for p in all_chain_points]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
    else:
        avg_error = 1.0
        max_error = 1.0

    # Coverage: sample points along the true walls and check if a chain point is nearby
    truth_length = sum(math.sqrt((w[2]-w[0])**2 + (w[3]-w[1])**2) for w in truth_walls)
    covered = 0
    sample_count = 0
    for w in truth_walls:
        wall_len = math.sqrt((w[2]-w[0])**2 + (w[3]-w[1])**2)
        n_samples = max(2, int(wall_len / 0.05))
        for i in range(n_samples):
            t = i / max(n_samples - 1, 1)
            sx = w[0] + t * (w[2] - w[0])
            sy = w[1] + t * (w[3] - w[1])
            # Is there a chain point within 0.15m?
            min_d = min((math.sqrt((sx-p[0])**2 + (sy-p[1])**2) for p in all_chain_points), default=10)
            if min_d < 0.15:
                covered += 1
            sample_count += 1

    coverage = covered / max(sample_count, 1)

    # Composite score: balance accuracy and coverage
    accuracy_score = max(0, 1.0 - avg_error * 10)  # 0.1m error → 0 score
    score = coverage * 0.5 + accuracy_score * 0.3 + min(total_length / truth_length, 1.0) * 0.2

    return {
        "coverage": round(coverage, 3),
        "accuracy": round(avg_error * 1000, 1),  # mm
        "max_error_mm": round(max_error * 1000, 1),
        "total_length": round(total_length, 2),
        "truth_length": round(truth_length, 2),
        "chain_count": len(chains),
        "vertex_count": len(all_chain_points),
        "score": round(score, 3),
    }


# ============================================================
# Parameter sweep
# ============================================================

def run_sweep():
    rooms = [
        ("triangle", TRIANGLE_WALLS,
         [(0, 0)] + [(0.1*i, 0) for i in range(-3, 6)] + [(0, 0.1*i) for i in range(-3, 4)]),
        ("rectangle", RECTANGLE_WALLS,
         [(0, 0), (0.5, 0), (-0.5, 0), (0, 0.2), (0, -0.2)]),
        ("lshape", LSHAPE_WALLS,
         [(0.5, 0.3), (1.5, 0.3), (0.5, 0.7)]),
    ]

    merge_radii = [0.01, 0.02, 0.03, 0.04, 0.05]
    consolidate_radii = [0.03, 0.04, 0.06, 0.08, 0.10]
    connect_dists = [0.08, 0.10, 0.12, 0.15, 0.20, 0.25]

    print(f"{'merge':>6} {'consol':>6} {'conn':>6} | ", end="")
    print(f"{'tri_score':>9} {'tri_cov':>7} {'tri_acc':>7} | ", end="")
    print(f"{'rect_score':>10} {'rect_cov':>8} | ", end="")
    print(f"{'L_score':>7} {'avg':>6}")
    print("-" * 100)

    best_params = None
    best_avg = -1

    for mr in merge_radii:
        for cr in consolidate_radii:
            if cr <= mr:
                continue  # consolidate must be wider than merge
            for cd in connect_dists:
                scores = []
                details = []
                for name, walls, positions in rooms:
                    cloud = build_cloud(walls, positions, mr, cr)
                    pts = cloud.get_points_2d(min_confidence=0.2)
                    chains = build_wall_chains(pts, connect_dist=cd, wall_height=0.2)
                    s = score_chains(chains, walls)
                    scores.append(s["score"])
                    details.append(s)

                avg = sum(scores) / len(scores)

                print(f"{mr:6.2f} {cr:6.2f} {cd:6.2f} | ", end="")
                print(f"{details[0]['score']:9.3f} {details[0]['coverage']:7.1%} {details[0]['accuracy']:6.1f}mm | ", end="")
                print(f"{details[1]['score']:10.3f} {details[1]['coverage']:8.1%} | ", end="")
                print(f"{details[2]['score']:7.3f} {avg:6.3f}", end="")

                if avg > best_avg:
                    best_avg = avg
                    best_params = (mr, cr, cd)
                    print(" ***", end="")
                print()

    print(f"\nBest: merge={best_params[0]}, consolidate={best_params[1]}, connect={best_params[2]}")
    print(f"Average score: {best_avg:.3f}")

    # Print detailed results for best params
    print(f"\nDetailed results for best params:")
    for name, walls, positions in rooms:
        cloud = build_cloud(walls, positions, best_params[0], best_params[1])
        pts = cloud.get_points_2d(min_confidence=0.2)
        chains = build_wall_chains(pts, connect_dist=best_params[2])
        s = score_chains(chains, walls)
        print(f"  {name}: {s}")


if __name__ == "__main__":
    run_sweep()
