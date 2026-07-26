from __future__ import annotations

import math

import numpy as np

from test_coverage.models import PathKind, PathPoint, Point2D


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def point_distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def heading(first: Point2D, second: Point2D, fallback: float = 0.0) -> float:
    if first == second:
        return fallback
    return math.atan2(second.y - first.y, second.x - first.x)


def point_is_safe(safe: np.ndarray, point: Point2D) -> bool:
    x = int(round(point.x))
    y = int(round(point.y))
    return 0 <= y < safe.shape[0] and 0 <= x < safe.shape[1] and bool(safe[y, x])


def line_is_safe(
    safe: np.ndarray, first: Point2D, second: Point2D, sample_step_px: float = 0.5
) -> bool:
    distance = point_distance(first, second)
    sample_count = max(1, int(math.ceil(distance / max(0.1, sample_step_px))))
    for index in range(sample_count + 1):
        ratio = index / sample_count
        point = Point2D(
            first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y),
        )
        if not point_is_safe(safe, point):
            return False
    return True


def densify_line(
    first: Point2D,
    second: Point2D,
    spacing_px: float,
    kind: PathKind,
    cell_id: int,
    start_yaw: float | None = None,
    end_yaw: float | None = None,
) -> list[PathPoint]:
    distance = point_distance(first, second)
    if distance <= 1e-9:
        yaw = end_yaw if end_yaw is not None else (start_yaw or 0.0)
        return [PathPoint(first.x, first.y, yaw, kind, cell_id)]

    yaw = heading(first, second)
    divisions = max(1, int(math.ceil(distance / max(0.1, spacing_px))))
    result = []
    for index in range(divisions + 1):
        ratio = index / divisions
        pose_yaw = yaw
        if index == 0 and start_yaw is not None:
            pose_yaw = start_yaw
        elif index == divisions and end_yaw is not None:
            pose_yaw = end_yaw
        result.append(
            PathPoint(
                first.x + ratio * (second.x - first.x),
                first.y + ratio * (second.y - first.y),
                normalize_angle(pose_yaw),
                kind,
                cell_id,
            )
        )
    return result


def append_path(target: list[PathPoint], addition: list[PathPoint]) -> None:
    if not addition:
        return
    if target and same_position(target[-1], addition[0]) and same_yaw(
        target[-1], addition[0]
    ):
        target.extend(addition[1:])
    else:
        target.extend(addition)


def same_position(first: PathPoint, second: PathPoint, tolerance: float = 1e-6) -> bool:
    return math.hypot(first.x - second.x, first.y - second.y) <= tolerance


def same_yaw(first: PathPoint, second: PathPoint, tolerance: float = 1e-6) -> bool:
    return abs(normalize_angle(first.yaw - second.yaw)) <= tolerance
