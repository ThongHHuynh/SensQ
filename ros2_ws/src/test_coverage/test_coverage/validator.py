from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from test_coverage.geometry import line_is_safe, point_is_safe
from test_coverage.models import PathPoint


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    unsafe_points: int
    unsafe_edges: int
    nonfinite_points: int


def validate_path(safe: np.ndarray, points: list[PathPoint]) -> ValidationResult:
    unsafe_points = 0
    nonfinite_points = 0
    for point in points:
        if not all(math.isfinite(value) for value in (point.x, point.y, point.yaw)):
            nonfinite_points += 1
            continue
        if not point_is_safe(safe, point.point):
            unsafe_points += 1

    unsafe_edges = sum(
        not line_is_safe(safe, start.point, end.point)
        for start, end in zip(points, points[1:])
    )
    valid = (
        len(points) >= 2
        and unsafe_points == 0
        and unsafe_edges == 0
        and nonfinite_points == 0
    )
    return ValidationResult(valid, unsafe_points, unsafe_edges, nonfinite_points)


def estimate_coverage(
    safe: np.ndarray, points: list[PathPoint], coverage_radius_px: float
) -> float:
    free_count = int(np.count_nonzero(safe))
    if free_count == 0:
        return 0.0

    covered = np.zeros_like(safe, dtype=bool)
    radius = max(0, int(math.ceil(coverage_radius_px)))
    offsets = [
        (dx, dy)
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
        if dx * dx + dy * dy <= coverage_radius_px * coverage_radius_px
    ]
    height, width = safe.shape
    for point in points:
        center_x = int(round(point.x))
        center_y = int(round(point.y))
        for dx, dy in offsets:
            x, y = center_x + dx, center_y + dy
            if 0 <= x < width and 0 <= y < height and safe[y, x]:
                covered[y, x] = True
    return 100.0 * float(np.count_nonzero(covered)) / free_count


def count_kind_runs(points: list[PathPoint], kind) -> int:
    count = 0
    previous = None
    for point in points:
        if point.kind == kind and previous != kind:
            count += 1
        previous = point.kind
    return count
