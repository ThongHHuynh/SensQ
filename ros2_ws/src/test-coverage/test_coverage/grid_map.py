from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from test_coverage.models import Point2D, SweepAxis


@dataclass
class GridMap:
    occupancy: np.ndarray
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    @classmethod
    def from_message(cls, msg) -> "GridMap":
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        orientation = msg.info.origin.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return cls(
            occupancy=data,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=math.atan2(siny_cosp, cosy_cosp),
            frame_id=msg.header.frame_id or "map",
        )

    def grid_to_world(self, point: Point2D) -> Point2D:
        local_x = (point.x + 0.5) * self.resolution
        local_y = (point.y + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return Point2D(
            self.origin_x + local_x * cosine - local_y * sine,
            self.origin_y + local_x * sine + local_y * cosine,
        )

    def world_to_grid(self, point: Point2D) -> Point2D:
        dx = point.x - self.origin_x
        dy = point.y - self.origin_y
        cosine = math.cos(-self.origin_yaw)
        sine = math.sin(-self.origin_yaw)
        return Point2D(
            (dx * cosine - dy * sine) / self.resolution - 0.5,
            (dx * sine + dy * cosine) / self.resolution - 0.5,
        )


def meters_to_pixels(value_m: float, resolution: float, minimum: int = 1) -> int:
    return max(minimum, int(math.ceil(value_m / resolution)))


def build_safe_grid(
    occupancy: np.ndarray,
    occupied_threshold: int,
    unknown_is_obstacle: bool,
    inflation_radius_px: int,
) -> np.ndarray:
    if occupancy.ndim != 2:
        raise ValueError("occupancy grid must be two dimensional")
    blocked = occupancy >= occupied_threshold
    if unknown_is_obstacle:
        blocked = np.logical_or(blocked, occupancy < 0)

    inflated = blocked.copy()
    radius = max(0, int(inflation_radius_px))
    height, width = blocked.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            source_y0 = max(0, -dy)
            source_y1 = min(height, height - dy)
            source_x0 = max(0, -dx)
            source_x1 = min(width, width - dx)
            target_y0 = source_y0 + dy
            target_y1 = source_y1 + dy
            target_x0 = source_x0 + dx
            target_x1 = source_x1 + dx
            inflated[target_y0:target_y1, target_x0:target_x1] |= blocked[
                source_y0:source_y1, source_x0:source_x1
            ]

    if radius:
        inflated[:radius, :] = True
        inflated[-radius:, :] = True
        inflated[:, :radius] = True
        inflated[:, -radius:] = True
    return np.logical_not(inflated)


def nearest_safe_cell(safe: np.ndarray, point: Point2D) -> tuple[int, int]:
    ys, xs = np.nonzero(safe)
    if len(xs) == 0:
        raise ValueError("map contains no traversable cells after inflation")
    distances = (xs - point.x) ** 2 + (ys - point.y) ** 2
    index = int(np.argmin(distances))
    return int(xs[index]), int(ys[index])


def connected_component(
    safe: np.ndarray, start: Point2D
) -> tuple[np.ndarray, tuple[int, int]]:
    start_x, start_y = nearest_safe_cell(safe, start)
    component = np.zeros_like(safe, dtype=bool)
    queue = deque([(start_x, start_y)])
    component[start_y, start_x] = True
    height, width = safe.shape

    while queue:
        x, y = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if (
                0 <= nx < width
                and 0 <= ny < height
                and safe[ny, nx]
                and not component[ny, nx]
            ):
                component[ny, nx] = True
                queue.append((nx, ny))
    return component, (start_x, start_y)


def sweep_to_grid(along: float, row: float, axis: SweepAxis) -> Point2D:
    if axis == SweepAxis.HORIZONTAL:
        return Point2D(along, row)
    return Point2D(row, along)


def grid_to_sweep(point: Point2D, axis: SweepAxis) -> Point2D:
    if axis == SweepAxis.HORIZONTAL:
        return point
    return Point2D(point.y, point.x)
