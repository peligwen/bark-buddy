"""
Map store — accumulates ultrasonic scan data into a 2D point map.

Stores scan results and provides a merged point cloud that the
web UI can render as a 2D map visualization.
"""

import logging
import math
from typing import Optional

from behaviors.scan import ScanResult, ScanPoint

logger = logging.getLogger(__name__)

# Map bounds (meters) — auto-expand as needed
DEFAULT_RANGE_M = 2.0


class MapStore:
    """
    Accumulates scan results into a combined point cloud.

    Each scan adds obstacle points relative to the scan's origin.
    The map stores all points in a shared coordinate frame.
    """

    def __init__(self):
        self._scans: list[ScanResult] = []
        self._points: list[dict] = []  # {x, y, distance_mm, scan_index}

    @property
    def scan_count(self) -> int:
        return len(self._scans)

    @property
    def point_count(self) -> int:
        return len(self._points)

    def add_scan(self, scan: ScanResult) -> int:
        """Add a completed scan to the map. Returns the scan index."""
        index = len(self._scans)
        self._scans.append(scan)
        for point in scan.points:
            self._points.append({
                "x": round(point.x, 3),
                "y": round(point.y, 3),
                "distance_mm": point.distance_mm,
                "scan_index": index,
            })
        logger.info("Added scan %d with %d points (total: %d)",
                     index, len(scan.points), len(self._points))
        return index

    def clear(self) -> None:
        """Clear all scans and points."""
        self._scans.clear()
        self._points.clear()
        logger.info("Map cleared")

    def get_bounds(self) -> dict:
        """Get the bounding box of all points."""
        if not self._points:
            return {"min_x": -DEFAULT_RANGE_M, "max_x": DEFAULT_RANGE_M,
                    "min_y": -DEFAULT_RANGE_M, "max_y": DEFAULT_RANGE_M}

        xs = [p["x"] for p in self._points]
        ys = [p["y"] for p in self._points]
        margin = 0.2
        return {
            "min_x": min(xs) - margin,
            "max_x": max(xs) + margin,
            "min_y": min(ys) - margin,
            "max_y": max(ys) + margin,
        }

    def get_scan_origins(self) -> list[dict]:
        """Get the origin positions of all scans."""
        return [
            {"x": s.origin_x, "y": s.origin_y, "heading": s.origin_heading,
             "point_count": len(s.points)}
            for s in self._scans
        ]

    def to_dict(self) -> dict:
        """Serialize the full map for the web UI."""
        bounds = self.get_bounds()
        return {
            "bounds": bounds,
            "points": self._points,
            "scans": self.get_scan_origins(),
            "scan_count": self.scan_count,
            "point_count": self.point_count,
        }
