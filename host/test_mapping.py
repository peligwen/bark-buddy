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

    # Nearby point should merge
    p2 = cloud.add_point(1.01, 0.0, 0.09, 500)
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

    cloud.add_point(1.0, 0.0, 0.09, 500)
    initial_count = cloud.point_count
    initial_conf = cloud.get_points()[0].confidence

    import time
    time.sleep(0.02)
    cloud.decay_tick()
    after_conf = cloud.get_points()[0].confidence if cloud.point_count > 0 else 0

    r.assert_true(after_conf < initial_conf, "confidence decayed")
    r.metric("before", round(initial_conf, 3))
    r.metric("after", round(after_conf, 3))

    # Decay until pruned
    for _ in range(50):
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
