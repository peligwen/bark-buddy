"""Octree-based spatial index and point cloud for 3D obstacle mapping."""

import math
import time
from dataclasses import dataclass, field


@dataclass
class CloudPoint:
    x: float
    y: float
    z: float
    confidence: float
    source: str
    timestamp: float


class OctreeNode:

    def __init__(
        self,
        cx: float,
        cy: float,
        cz: float,
        half_size: float,
        max_points: int = 16,
        min_half: float = 0.05,
    ) -> None:
        self._cx = cx
        self._cy = cy
        self._cz = cz
        self._half_size = half_size
        self._max_points = max_points
        self._min_half = min_half
        self._points: list[CloudPoint] = []
        self._children: list[OctreeNode] = []

    def _contains(self, p: CloudPoint) -> bool:
        h = self._half_size
        return (
            self._cx - h <= p.x <= self._cx + h
            and self._cy - h <= p.y <= self._cy + h
            and self._cz - h <= p.z <= self._cz + h
        )

    def _octant_index(self, p: CloudPoint) -> int:
        idx = 0
        if p.x >= self._cx:
            idx |= 1
        if p.y >= self._cy:
            idx |= 2
        if p.z >= self._cz:
            idx |= 4
        return idx

    def _subdivide(self) -> None:
        q = self._half_size / 2.0
        for i in range(8):
            nx = self._cx + q * (1 if i & 1 else -1)
            ny = self._cy + q * (1 if i & 2 else -1)
            nz = self._cz + q * (1 if i & 4 else -1)
            self._children.append(
                OctreeNode(nx, ny, nz, q, self._max_points, self._min_half)
            )
        for p in self._points:
            self._children[self._octant_index(p)]._points.append(p)
        self._points = []

    def insert(self, point: CloudPoint) -> bool:
        if not self._contains(point):
            return False

        if self._children:
            return self._children[self._octant_index(point)].insert(point)

        self._points.append(point)

        if len(self._points) > self._max_points and self._half_size > self._min_half:
            self._subdivide()

        return True

    def query_radius(self, x: float, y: float, z: float, radius: float) -> list[CloudPoint]:
        results: list[CloudPoint] = []
        self._query_radius_inner(x, y, z, radius, radius * radius, results)
        return results

    def _query_radius_inner(
        self,
        x: float,
        y: float,
        z: float,
        radius: float,
        radius_sq: float,
        results: list[CloudPoint],
    ) -> None:
        h = self._half_size
        closest_x = max(self._cx - h, min(x, self._cx + h))
        closest_y = max(self._cy - h, min(y, self._cy + h))
        closest_z = max(self._cz - h, min(z, self._cz + h))
        dx = closest_x - x
        dy = closest_y - y
        dz = closest_z - z
        if dx * dx + dy * dy + dz * dz > radius_sq:
            return

        if self._children:
            for child in self._children:
                child._query_radius_inner(x, y, z, radius, radius_sq, results)
        else:
            for p in self._points:
                pdx = p.x - x
                pdy = p.y - y
                pdz = p.z - z
                if pdx * pdx + pdy * pdy + pdz * pdz <= radius_sq:
                    results.append(p)

    def all_points(self) -> list[CloudPoint]:
        results: list[CloudPoint] = []
        self._gather(results)
        return results

    def _gather(self, results: list[CloudPoint]) -> None:
        if self._children:
            for child in self._children:
                child._gather(results)
        else:
            results.extend(self._points)

    def remove_below_confidence(self, threshold: float) -> int:
        if self._children:
            count = 0
            for child in self._children:
                count += child.remove_below_confidence(threshold)
            return count
        else:
            before = len(self._points)
            self._points = [p for p in self._points if p.confidence >= threshold]
            return before - len(self._points)


class PointCloud:

    def __init__(self, bounds: float = 4.0) -> None:
        self._root = OctreeNode(0.0, 0.0, 0.0, bounds)
        self._merge_radius = 0.05
        self._decay_rate = 0.95
        self._decay_interval = 10.0
        self._prune_threshold = 0.1
        self._last_decay = time.monotonic()

    def add_point(
        self, x: float, y: float, z: float, distance_mm: float, source: str = "ultrasonic"
    ) -> CloudPoint:
        confidence = max(0.3, 1.0 - distance_mm / 3000)

        neighbors = self._root.query_radius(x, y, z, self._merge_radius)
        if neighbors:
            nearest = neighbors[0]
            nearest.confidence = min(1.0, nearest.confidence + 0.15)
            total_w = nearest.confidence + confidence
            nearest.x = (nearest.x * nearest.confidence + x * confidence) / total_w
            nearest.y = (nearest.y * nearest.confidence + y * confidence) / total_w
            nearest.z = (nearest.z * nearest.confidence + z * confidence) / total_w
            nearest.timestamp = time.monotonic()
            return nearest

        wider = self._root.query_radius(x, y, z, 0.15)
        if not wider:
            confidence *= 0.5

        point = CloudPoint(
            x=x, y=y, z=z, confidence=confidence, source=source, timestamp=time.monotonic()
        )
        self._root.insert(point)
        return point

    def decay_tick(self) -> None:
        now = time.monotonic()
        if now - self._last_decay < self._decay_interval:
            return
        for p in self._root.all_points():
            p.confidence *= self._decay_rate
        self._root.remove_below_confidence(self._prune_threshold)
        self._last_decay = now

    def get_points(self, min_confidence: float = 0.0) -> list[CloudPoint]:
        return [p for p in self._root.all_points() if p.confidence >= min_confidence]

    def get_points_2d(self, min_confidence: float = 0.0) -> list[dict]:
        results = []
        for p in self._root.all_points():
            if p.confidence >= min_confidence:
                dist_2d = math.sqrt(p.x * p.x + p.y * p.y)
                results.append({
                    "x": p.x,
                    "y": p.y,
                    "z": p.z,
                    "confidence": p.confidence,
                    "distance_mm": int(dist_2d * 1000),
                })
        return results

    def clear(self) -> None:
        half = self._root._half_size
        self._root = OctreeNode(0.0, 0.0, 0.0, half)

    @property
    def point_count(self) -> int:
        return len(self._root.all_points())
