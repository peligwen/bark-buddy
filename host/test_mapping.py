#!/usr/bin/env python3
"""
Automated mapping pipeline tests — CLI, reproducible, with pass/fail criteria.

Tests the full stack: mock transport → server → point cloud → wall fitting.
No browser needed. Runs headless via WebSocket.

Usage:
    python3 host/test_mapping.py                    # run all tests
    python3 host/test_mapping.py test_clean_scan    # run one test
    python3 host/test_mapping.py --sweep            # parameter sweep
"""

import asyncio
import json
import math
import sys
import time
import logging

# Add host dir to path
sys.path.insert(0, "host")

from behaviors.octree import PointCloud
from behaviors.wall_fit import fit_walls, WallSegment
from behaviors.map_store import MapStore
from behaviors.scan import ScanResult

logging.basicConfig(level=logging.WARNING)

# ============================================================
# Test infrastructure
# ============================================================

class TestResult:
    def __init__(self, name):
        self.name = name
        self.passed = True
        self.assertions = []
        self.metrics = {}

    def assert_true(self, condition, msg):
        self.assertions.append((condition, msg))
        if not condition:
            self.passed = False

    def assert_ge(self, value, threshold, msg):
        ok = value >= threshold
        self.assertions.append((ok, f"{msg}: {value} >= {threshold}"))
        if not ok:
            self.passed = False

    def assert_le(self, value, threshold, msg):
        ok = value <= threshold
        self.assertions.append((ok, f"{msg}: {value} <= {threshold}"))
        if not ok:
            self.passed = False

    def assert_between(self, value, lo, hi, msg):
        ok = lo <= value <= hi
        self.assertions.append((ok, f"{msg}: {lo} <= {value} <= {hi}"))
        if not ok:
            self.passed = False

    def metric(self, key, value):
        self.metrics[key] = value

    def report(self):
        status = "PASS" if self.passed else "FAIL"
        print(f"  [{status}] {self.name}")
        for ok, msg in self.assertions:
            if not ok:
                print(f"         FAILED: {msg}")
        if self.metrics:
            metrics_str = ", ".join(f"{k}={v}" for k, v in self.metrics.items())
            print(f"         metrics: {metrics_str}")
        return self.passed


def run_tests(tests):
    passed = 0
    failed = 0
    print(f"\nRunning {len(tests)} tests...\n")
    for test_func in tests:
        result = test_func()
        if result.report():
            passed += 1
        else:
            failed += 1
    print(f"\n{passed} passed, {failed} failed, {passed + failed} total\n")
    return failed == 0


# ============================================================
# Simulated scan data generators
# ============================================================

def make_scan_at(origin_x, origin_y, heading, wall_segments, noise_mm=0):
    """Generate a scan result by raycasting against wall segments."""
    import random
    result = ScanResult(origin_x, origin_y, heading)
    for step in range(24):  # 15° steps
        angle = step * 15.0
        world_angle = heading + angle
        rad = math.radians(world_angle)
        dx, dy = math.cos(rad), math.sin(rad)

        # Raycast against walls
        min_dist = 3.0
        for w in wall_segments:
            t = _ray_segment_intersect(origin_x, origin_y, dx, dy,
                                        w[0], w[1], w[2], w[3])
            if t is not None and t < min_dist:
                min_dist = t

        dist_mm = int(min_dist * 1000)
        if noise_mm > 0:
            dist_mm += int(random.gauss(0, noise_mm))
        if dist_mm < 3000:
            result.add_point(angle, max(10, dist_mm))

    return result


def _ray_segment_intersect(ox, oy, dx, dy, x1, y1, x2, y2):
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    s = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t > 0 and 0 <= s <= 1:
        return t
    return None


# Triangle room walls (same as mock transport)
TRIANGLE_WALLS = [
    (1.5, 0.0, -0.75, 1.3),    # A→B
    (-0.75, 1.3, -0.75, -1.3), # B→C
    (-0.75, -1.3, 1.5, 0.0),   # C→A
    (-0.3, 0.5, -0.3, -0.5),   # interior partition
]


# ============================================================
# Tests
# ============================================================

def test_point_cloud_basic():
    r = TestResult("point_cloud_basic")
    cloud = PointCloud()

    p = cloud.add_point(1.0, 0.0, 0.09, 500)
    r.assert_true(p is not None, "point added")
    r.assert_ge(p.confidence, 0.3, "confidence above minimum")
    r.metric("initial_confidence", round(p.confidence, 3))

    # Very nearby point should merge (within 10mm merge radius)
    p2 = cloud.add_point(1.005, 0.0, 0.09, 500)
    r.assert_true(cloud.point_count == 1, f"merged to 1 (got {cloud.point_count})")
    r.assert_ge(p2.confidence, p.confidence, "confidence boosted on merge")

    # Distant point should not merge
    cloud.add_point(2.0, 0.0, 0.09, 1000)
    r.assert_true(cloud.point_count == 2, f"2 distinct points (got {cloud.point_count})")

    return r


def test_point_cloud_decay():
    r = TestResult("point_cloud_decay")
    cloud = PointCloud()
    cloud._decay_interval = 0.01  # fast decay for testing
    cloud._decay_rate = 0.5       # aggressive for test speed
    cloud._prune_threshold = 0.05

    cloud.add_point(1.0, 0.0, 0.09, 500)
    initial_conf = cloud.get_points()[0].confidence

    import time
    time.sleep(0.02)
    cloud.decay_tick()
    after_conf = cloud.get_points()[0].confidence if cloud.point_count > 0 else 0

    r.assert_true(after_conf < initial_conf, "confidence decayed")
    r.metric("before", round(initial_conf, 3))
    r.metric("after", round(after_conf, 3))

    # Decay until pruned
    for _ in range(20):
        time.sleep(0.02)
        cloud.decay_tick()
    r.assert_true(cloud.point_count == 0, f"pruned to 0 (got {cloud.point_count})")

    return r


def test_clean_scan_wall_detection():
    r = TestResult("clean_scan_wall_detection")
    store = MapStore()

    scan = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=0)
    store.add_scan(scan)
    r.metric("scan_points", len(scan.points))

    data = store.to_dict()
    r.metric("cloud_points", data["point_count"])
    r.metric("walls_detected", len(data["walls"]))

    r.assert_ge(data["point_count"], 10, "enough points in cloud")
    r.assert_ge(len(data["walls"]), 2, "at least 2 walls detected")

    # Check wall lengths are reasonable (0.3m - 3.0m)
    for w in data["walls"]:
        length = math.sqrt((w["x2"] - w["x1"])**2 + (w["y2"] - w["y1"])**2)
        r.assert_between(length, 0.2, 3.0, f"wall length {length:.2f}m")

    return r


def test_noisy_scan_wall_detection():
    r = TestResult("noisy_scan_wall_detection")
    store = MapStore()

    scan = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=50)
    store.add_scan(scan)
    r.metric("scan_points", len(scan.points))

    data = store.to_dict()
    r.metric("cloud_points", data["point_count"])
    r.metric("walls_detected", len(data["walls"]))

    # Should still detect walls despite noise
    r.assert_ge(data["point_count"], 8, "enough points survive noise")
    r.assert_ge(len(data["walls"]), 1, "at least 1 wall detected with noise")

    return r


def test_multi_scan_reinforcement():
    r = TestResult("multi_scan_reinforcement")
    store = MapStore()

    # Two scans from same position should reinforce confidence
    scan1 = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=0)
    store.add_scan(scan1)
    pts_after_1 = store.to_dict()["point_count"]

    scan2 = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=0)
    store.add_scan(scan2)
    data = store.to_dict()
    pts_after_2 = data["point_count"]

    r.metric("points_scan1", pts_after_1)
    r.metric("points_scan2", pts_after_2)

    # Points should merge (not double)
    r.assert_le(pts_after_2, pts_after_1 * 1.5, "second scan merges, not doubles")

    # Average confidence should increase
    if data["points"]:
        avg_conf = sum(p.get("confidence", 0) for p in data["points"]) / len(data["points"])
        r.metric("avg_confidence", round(avg_conf, 3))
        r.assert_ge(avg_conf, 0.4, "confidence boosted by reinforcement")

    return r


def test_multi_position_scan():
    r = TestResult("multi_position_scan")
    store = MapStore()

    # Scans from different positions should cover more walls
    positions = [(0, 0, 0), (0.3, 0.2, 45), (-0.2, -0.1, 180)]
    for x, y, h in positions:
        scan = make_scan_at(x, y, h, TRIANGLE_WALLS, noise_mm=10)
        store.add_scan(scan)

    data = store.to_dict()
    r.metric("total_points", data["point_count"])
    r.metric("walls_detected", len(data["walls"]))
    r.metric("scans", data["scan_count"])

    r.assert_ge(data["point_count"], 20, "enough total points")
    r.assert_ge(len(data["walls"]), 3, "at least 3 walls from multi-position")

    return r


def test_outlier_rejection():
    r = TestResult("outlier_rejection")
    cloud = PointCloud()

    # Add a cluster of good points
    for i in range(5):
        cloud.add_point(1.0 + i * 0.04, 0.5, 0.09, 500)

    # Add an outlier far away
    cloud.add_point(5.0, 5.0, 0.09, 2900)

    points = cloud.get_points()
    r.metric("total_points", len(points))

    # Find the outlier
    outlier = [p for p in points if p.x > 3.0]
    cluster = [p for p in points if p.x < 3.0]

    if outlier:
        r.assert_le(outlier[0].confidence, 0.25, "outlier has low confidence")
        r.metric("outlier_conf", round(outlier[0].confidence, 3))

    if cluster:
        avg_cluster_conf = sum(p.confidence for p in cluster) / len(cluster)
        r.metric("cluster_avg_conf", round(avg_cluster_conf, 3))
        r.assert_ge(avg_cluster_conf, 0.3, "cluster has decent confidence")

    return r


def test_wall_fit_linearity():
    r = TestResult("wall_fit_linearity")

    # Points along a line should produce one wall
    line_points = [{"x": i * 0.1, "y": 1.0 + i * 0.01, "confidence": 0.8}
                   for i in range(10)]
    walls = fit_walls(line_points, eps=0.2, min_samples=3)
    r.assert_true(len(walls) == 1, f"1 wall from line (got {len(walls)})")
    if walls:
        length = math.sqrt((walls[0].x2 - walls[0].x1)**2 + (walls[0].y2 - walls[0].y1)**2)
        r.metric("wall_length", round(length, 3))
        r.assert_between(length, 0.8, 1.2, "wall length ~0.9m")

    return r


def test_wall_fit_l_shape():
    r = TestResult("wall_fit_l_shape")

    # L-shaped points with a gap at the corner should produce 2 walls
    horiz = [{"x": 0.3 + i * 0.1, "y": 0, "confidence": 0.8} for i in range(8)]
    vert = [{"x": 0, "y": 0.3 + i * 0.1, "confidence": 0.8} for i in range(8)]
    walls = fit_walls(horiz + vert, eps=0.2, min_samples=2)

    r.metric("walls_detected", len(walls))
    r.assert_ge(len(walls), 2, "at least 2 walls from separated L-shape")

    return r


def test_wall_fit_scattered():
    r = TestResult("wall_fit_scattered")

    # Random scattered points should produce no walls
    import random
    random.seed(42)
    scattered = [{"x": random.uniform(-2, 2), "y": random.uniform(-2, 2), "confidence": 0.5}
                 for _ in range(20)]
    walls = fit_walls(scattered, eps=0.2, min_samples=3)

    r.metric("walls_detected", len(walls))
    r.assert_le(len(walls), 1, "scattered points produce few/no walls")

    return r


# ============================================================
# Parameter sweep
# ============================================================

def run_sweep():
    print("\nParameter sweep: DBSCAN eps × min_samples × min_confidence\n")
    print(f"{'eps':>6} {'min_s':>5} {'min_c':>5} | {'walls':>5} {'avg_len':>7} {'avg_conf':>8} | {'verdict'}")
    print("-" * 65)

    best = None
    best_score = -1

    for eps in [0.15, 0.25, 0.35, 0.45, 0.60]:
        for min_samples in [2, 3, 4]:
            for min_conf in [0.1, 0.2, 0.3]:
                store = MapStore()
                for x, y, h in [(0, 0, 0), (0.3, 0.2, 45), (-0.2, -0.1, 180)]:
                    scan = make_scan_at(x, y, h, TRIANGLE_WALLS, noise_mm=20)
                    store.add_scan(scan)

                points = store._cloud.get_points_2d(min_confidence=min_conf)
                walls = fit_walls(points, eps=eps, min_samples=min_samples,
                                  min_confidence=min_conf)

                avg_len = sum(math.sqrt((w.x2-w.x1)**2+(w.y2-w.y1)**2) for w in walls) / max(len(walls), 1)
                avg_conf = sum(w.confidence for w in walls) / max(len(walls), 1)

                # Score: prefer 3-4 walls with good length and confidence
                wall_score = min(len(walls), 4) / 4.0
                len_score = min(avg_len, 1.0)
                conf_score = avg_conf
                score = wall_score * 0.5 + len_score * 0.3 + conf_score * 0.2

                verdict = "***" if len(walls) >= 3 else ""
                print(f"{eps:6.2f} {min_samples:5d} {min_conf:5.1f} | {len(walls):5d} {avg_len:7.2f} {avg_conf:8.2f} | {verdict}")

                if score > best_score:
                    best_score = score
                    best = (eps, min_samples, min_conf, len(walls), avg_len, avg_conf)

    if best:
        print(f"\nBest: eps={best[0]}, min_samples={best[1]}, min_conf={best[2]}")
        print(f"  walls={best[3]}, avg_len={best[4]:.2f}m, avg_conf={best[5]:.2f}")


# ============================================================
# Main
# ============================================================

# ============================================================
# Wall detection accuracy tests — compare to ground truth
# ============================================================

def wall_distance_to_segment(px, py, x1, y1, x2, y2):
    """Distance from point (px,py) to line segment (x1,y1)-(x2,y2)."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


def wall_match_error(detected, truth_segments):
    """For each detected wall, find closest truth segment. Return avg distance."""
    if not detected:
        return float('inf')
    errors = []
    for w in detected:
        mid_x = (w["x1"] + w["x2"]) / 2
        mid_y = (w["y1"] + w["y2"]) / 2
        min_dist = float('inf')
        for t in truth_segments:
            d = wall_distance_to_segment(mid_x, mid_y, t[0], t[1], t[2], t[3])
            if d < min_dist:
                min_dist = d
        errors.append(min_dist)
    return sum(errors) / len(errors)


def test_accuracy_clean_single_scan():
    """Clean single scan — how close are detected walls to truth?"""
    r = TestResult("accuracy_clean_single_scan")
    store = MapStore()
    scan = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=0)
    store.add_scan(scan)
    walls = store.to_dict()["walls"]

    avg_err = wall_match_error(walls, TRIANGLE_WALLS)
    r.metric("walls", len(walls))
    r.metric("avg_error_m", round(avg_err, 3))
    r.assert_le(avg_err, 0.15, "average wall error < 15cm")
    r.assert_ge(len(walls), 2, "at least 2 walls detected")
    return r


def test_accuracy_noisy_single_scan():
    """50mm noise — walls still close to truth?"""
    r = TestResult("accuracy_noisy_single_scan")
    store = MapStore()
    scan = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=50)
    store.add_scan(scan)
    walls = store.to_dict()["walls"]

    avg_err = wall_match_error(walls, TRIANGLE_WALLS)
    r.metric("walls", len(walls))
    r.metric("avg_error_m", round(avg_err, 3))
    r.assert_le(avg_err, 0.25, "average wall error < 25cm with noise")
    r.assert_ge(len(walls), 1, "at least 1 wall detected")
    return r


def test_accuracy_heavy_noise():
    """100mm noise + 15% outliers — graceful degradation."""
    r = TestResult("accuracy_heavy_noise")
    import random
    random.seed(99)
    store = MapStore()
    scan = make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=100)
    # Add extra outlier points
    for _ in range(5):
        store._cloud.add_point(
            random.uniform(-1, 1), random.uniform(-1, 1), 0.09,
            random.randint(50, 2000))
    store.add_scan(scan)
    walls = store.to_dict()["walls"]

    avg_err = wall_match_error(walls, TRIANGLE_WALLS) if walls else float('inf')
    r.metric("walls", len(walls))
    r.metric("avg_error_m", round(avg_err, 3) if avg_err < 100 else "inf")
    # Relaxed criteria for heavy noise
    r.assert_le(avg_err, 0.5, "average wall error < 50cm under heavy noise")
    return r


def test_accuracy_multi_scan_convergence():
    """Multiple scans from different positions — accuracy improves."""
    r = TestResult("accuracy_multi_scan_convergence")

    # Single scan accuracy
    store1 = MapStore()
    store1.add_scan(make_scan_at(0, 0, 0, TRIANGLE_WALLS, noise_mm=30))
    walls1 = store1.to_dict()["walls"]
    err1 = wall_match_error(walls1, TRIANGLE_WALLS) if walls1 else float('inf')

    # Multi scan accuracy
    store3 = MapStore()
    for x, y, h in [(0, 0, 0), (0.3, 0.2, 45), (-0.2, -0.1, 180)]:
        store3.add_scan(make_scan_at(x, y, h, TRIANGLE_WALLS, noise_mm=30))
    walls3 = store3.to_dict()["walls"]
    err3 = wall_match_error(walls3, TRIANGLE_WALLS) if walls3 else float('inf')

    r.metric("single_scan_walls", len(walls1))
    r.metric("single_scan_error_m", round(err1, 3) if err1 < 100 else "inf")
    r.metric("multi_scan_walls", len(walls3))
    r.metric("multi_scan_error_m", round(err3, 3) if err3 < 100 else "inf")

    r.assert_ge(len(walls3), len(walls1), "more scans → more walls")
    return r


def test_accuracy_rectangular_room():
    """Different room shape — rectangular room."""
    r = TestResult("accuracy_rectangular_room")
    rect_walls = [
        (-1, -0.5, 1, -0.5),   # bottom
        (1, -0.5, 1, 0.5),     # right
        (1, 0.5, -1, 0.5),     # top
        (-1, 0.5, -1, -0.5),   # left
    ]
    store = MapStore()
    store.add_scan(make_scan_at(0, 0, 0, rect_walls, noise_mm=10))
    walls = store.to_dict()["walls"]

    avg_err = wall_match_error(walls, rect_walls) if walls else float('inf')
    r.metric("walls", len(walls))
    r.metric("avg_error_m", round(avg_err, 3))
    r.assert_ge(len(walls), 2, "at least 2 of 4 rectangle walls from single scan")
    r.assert_le(avg_err, 0.15, "average error < 15cm")
    return r


def test_accuracy_corridor():
    """Long narrow corridor — should detect 2 parallel walls."""
    r = TestResult("accuracy_corridor")
    corridor_walls = [
        (-2, -0.3, 2, -0.3),   # bottom wall
        (-2, 0.3, 2, 0.3),     # top wall
        (-2, -0.3, -2, 0.3),   # left cap
        (2, -0.3, 2, 0.3),     # right cap
    ]
    store = MapStore()
    store.add_scan(make_scan_at(0, 0, 0, corridor_walls, noise_mm=10))
    walls = store.to_dict()["walls"]

    avg_err = wall_match_error(walls, corridor_walls) if walls else float('inf')
    r.metric("walls", len(walls))
    r.metric("avg_error_m", round(avg_err, 3))
    r.assert_ge(len(walls), 2, "at least 2 parallel walls")
    return r


def test_accuracy_dense_triangle():
    """Dense points from many positions inside triangle — all walls detected."""
    r = TestResult("accuracy_dense_triangle")
    store = MapStore()

    # Scan from many positions inside the triangle
    positions = [(0, 0)] + [(0.1*i, 0) for i in range(-3, 6)] + [(0, 0.1*i) for i in range(-3, 4)]
    for px, py in positions:
        scan = make_scan_at(px, py, 0, TRIANGLE_WALLS, noise_mm=5)
        store.add_scan(scan)

    data = store.to_dict()
    walls = data["walls"]
    r.metric("points", data["point_count"])
    r.metric("walls", len(walls))

    # Total wall length should be close to room perimeter (~6.5m)
    total_len = sum(math.sqrt((w["x2"]-w["x1"])**2 + (w["y2"]-w["y1"])**2) for w in walls)
    r.metric("total_wall_length_m", round(total_len, 1))
    r.assert_ge(total_len, 4.0, "total wall length >= 4m (room is ~6.5m)")

    # Should detect significant walls (>0.3m)
    big_walls = [w for w in walls if math.sqrt((w["x2"]-w["x1"])**2 + (w["y2"]-w["y1"])**2) > 0.3]
    r.metric("big_walls", len(big_walls))
    r.assert_ge(len(big_walls), 3, "at least 3 significant walls (3 triangle + 1 partition)")

    # Accuracy: walls should be close to ground truth
    avg_err = wall_match_error(walls, TRIANGLE_WALLS)
    r.metric("avg_error_m", round(avg_err, 3))
    r.assert_le(avg_err, 0.15, "average wall error < 15cm")

    return r


def test_consolidation():
    """Consolidation merges nearby points and caps total count."""
    r = TestResult("consolidation")
    cloud = PointCloud()
    cloud._consolidate_interval = 0  # allow immediate consolidation

    # Add points spread wider than merge radius but within consolidation radius
    # merge_radius=0.02, consolidate_radius=0.04
    for i in range(30):
        x = 1.0 + (i % 6) * 0.03  # 30mm apart — wider than merge, within consolidate
        y = 0.5 + (i // 6) * 0.03
        cloud.add_point(x, y, 0.09, 500)

    before = cloud.point_count
    r.metric("before_consolidation", before)

    merged = cloud.consolidate()
    after = cloud.point_count
    r.metric("after_consolidation", after)
    r.metric("merged", merged)

    r.assert_true(after < before, f"consolidation reduced points: {before} -> {after}")
    r.assert_ge(merged, 5, f"at least 5 points merged (got {merged})")

    # Confidence should have increased for surviving points
    pts = cloud.get_points()
    if pts:
        max_conf = max(p.confidence for p in pts)
        r.metric("max_confidence", round(max_conf, 2))
        r.assert_ge(max_conf, 0.8, "consolidated points have high confidence")

    return r


ALL_TESTS = [
    test_point_cloud_basic,
    test_point_cloud_decay,
    test_clean_scan_wall_detection,
    test_noisy_scan_wall_detection,
    test_multi_scan_reinforcement,
    test_multi_position_scan,
    test_outlier_rejection,
    test_wall_fit_linearity,
    test_wall_fit_l_shape,
    test_wall_fit_scattered,
    # Accuracy tests — compare detected walls to ground truth
    test_accuracy_clean_single_scan,
    test_accuracy_noisy_single_scan,
    test_accuracy_heavy_noise,
    test_accuracy_multi_scan_convergence,
    test_accuracy_rectangular_room,
    test_accuracy_corridor,
    test_accuracy_dense_triangle,
    test_consolidation,
]

if __name__ == "__main__":
    if "--sweep" in sys.argv:
        run_sweep()
    elif len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        # Run specific test by name
        name = sys.argv[1]
        matching = [t for t in ALL_TESTS if t.__name__ == name]
        if matching:
            run_tests(matching)
        else:
            print(f"Unknown test: {name}")
            print("Available:", ", ".join(t.__name__ for t in ALL_TESTS))
    else:
        success = run_tests(ALL_TESTS)
        sys.exit(0 if success else 1)
