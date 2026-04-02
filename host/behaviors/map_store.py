"""
Map store — accumulates scan data into a 3D point cloud with octree indexing.

Points are scored by confidence, merged when nearby, and decayed over time.
Wall segments are fitted from clustered points for 3D visualization.
"""

import logging
from typing import Optional

from behaviors.octree import PointCloud
from behaviors.scan import ScanResult
from behaviors.wall_fit import fit_walls

logger = logging.getLogger(__name__)

DEFAULT_RANGE_M = 2.0
SENSOR_HEIGHT_M = 0.09  # approximate ultrasonic sensor height


class MapStore:
    """
    Accumulates scan results into a 3D point cloud with spatial indexing.

    Points are confidence-scored and spatially indexed via octree.
    Wall segments are computed from clustered high-confidence points.
    """

    def __init__(self):
        self._scans: list[ScanResult] = []
        self._cloud = PointCloud()

    @property
    def scan_count(self) -> int:
        return len(self._scans)

    @property
    def point_count(self) -> int:
        return self._cloud.point_count

    def add_scan(self, scan: ScanResult) -> int:
        index = len(self._scans)
        self._scans.append(scan)
        for point in scan.points:
            self._cloud.add_point(
                x=point.x, y=point.y, z=SENSOR_HEIGHT_M,
                distance_mm=point.distance_mm, source="ultrasonic",
            )
        logger.info("Added scan %d with %d points (cloud: %d)",
                     index, len(scan.points), self._cloud.point_count)
        return index

    def decay_tick(self) -> None:
        self._cloud.decay_tick()

    def clear(self) -> None:
        self._scans.clear()
        self._cloud.clear()
        logger.info("Map cleared")

    def get_bounds(self) -> dict:
        points = self._cloud.get_points_2d()
        if not points:
            return {"min_x": -DEFAULT_RANGE_M, "max_x": DEFAULT_RANGE_M,
                    "min_y": -DEFAULT_RANGE_M, "max_y": DEFAULT_RANGE_M}
        xs = [p["x"] for p in points]
        ys = [p["y"] for p in points]
        margin = 0.2
        return {
            "min_x": min(xs) - margin, "max_x": max(xs) + margin,
            "min_y": min(ys) - margin, "max_y": max(ys) + margin,
        }

    def get_scan_origins(self) -> list[dict]:
        return [
            {"x": s.origin_x, "y": s.origin_y, "heading": s.origin_heading,
             "point_count": len(s.points)}
            for s in self._scans
        ]

    def get_walls(self) -> list[dict]:
        points = self._cloud.get_points_2d(min_confidence=0.1)
        walls = fit_walls(points, wall_height=0.2)
        return [
            {"x1": w.x1, "y1": w.y1, "x2": w.x2, "y2": w.y2,
             "height": w.height, "confidence": w.confidence}
            for w in walls
        ]

    def to_dict(self) -> dict:
        points = self._cloud.get_points_2d()
        return {
            "bounds": self.get_bounds(),
            "points": points,
            "walls": self.get_walls(),
            "scans": self.get_scan_origins(),
            "scan_count": self.scan_count,
            "point_count": len(points),
        }
