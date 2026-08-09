from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from test_coverage.geometry import (
    append_path,
    densify_line,
    heading,
    line_is_safe,
    normalize_angle,
    point_is_safe,
)
from test_coverage.models import PathKind, PathPoint, Point2D, SweepAxis


@dataclass(frozen=True)
class TurnResult:
    points: list[PathPoint]
    kind: PathKind


class HeadlandTurnPlanner:
    def __init__(
        self,
        safe: np.ndarray,
        spacing_px: float,
        minimum_turning_radius_px: float,
        mode: str,
    ):
        self.safe = safe
        self.spacing_px = max(0.25, spacing_px)
        self.minimum_turning_radius_px = max(0.0, minimum_turning_radius_px)
        self.mode = mode

    def create(
        self,
        first: PathPoint,
        second: PathPoint,
        axis: SweepAxis,
        cell_id: int,
        overlap_px: float = 0.0,
    ) -> TurnResult | None:
        if self.mode in {"auto", "curve"}:
            curve = self._curve(first, second, cell_id, overlap_px)
            if curve is not None:
                return TurnResult(curve, PathKind.CURVE_TURN)
        if self.mode in {"auto", "square", "curve"}:
            square = self._square(first, second, axis, cell_id, overlap_px)
            if square is not None:
                return TurnResult(square, PathKind.SQUARE_TURN)
        return None

    def _curve(
        self,
        first: PathPoint,
        second: PathPoint,
        cell_id: int,
        overlap_px: float,
    ) -> list[PathPoint] | None:
        start = first.point
        goal = second.point
        blend = max(0.0, float(overlap_px))
        curve_start = Point2D(
            start.x + blend * math.cos(first.yaw),
            start.y + blend * math.sin(first.yaw),
        )
        curve_goal = Point2D(
            goal.x - blend * math.cos(second.yaw),
            goal.y - blend * math.sin(second.yaw),
        )
        separation = math.hypot(
            curve_goal.x - curve_start.x,
            curve_goal.y - curve_start.y,
        )
        control_distance = max(
            self.minimum_turning_radius_px,
            0.55 * separation,
        )
        first_control = Point2D(
            curve_start.x + control_distance * math.cos(first.yaw),
            curve_start.y + control_distance * math.sin(first.yaw),
        )
        second_control = Point2D(
            curve_goal.x - control_distance * math.cos(second.yaw),
            curve_goal.y - control_distance * math.sin(second.yaw),
        )
        approximate_length = separation + 2.0 * control_distance
        divisions = max(
            6, int(math.ceil(approximate_length / self.spacing_px))
        )
        result: list[PathPoint] = []
        lead_in = densify_line(
            start,
            curve_start,
            self.spacing_px,
            PathKind.CURVE_TURN,
            cell_id,
            start_yaw=first.yaw,
            end_yaw=first.yaw,
        )
        append_path(result, lead_in)

        for index in range(divisions + 1):
            ratio = index / divisions
            inverse = 1.0 - ratio
            x = (
                inverse**3 * curve_start.x
                + 3.0 * inverse**2 * ratio * first_control.x
                + 3.0 * inverse * ratio**2 * second_control.x
                + ratio**3 * curve_goal.x
            )
            y = (
                inverse**3 * curve_start.y
                + 3.0 * inverse**2 * ratio * first_control.y
                + 3.0 * inverse * ratio**2 * second_control.y
                + ratio**3 * curve_goal.y
            )
            dx = (
                3.0 * inverse**2 * (first_control.x - curve_start.x)
                + 6.0 * inverse * ratio * (second_control.x - first_control.x)
                + 3.0 * ratio**2 * (curve_goal.x - second_control.x)
            )
            dy = (
                3.0 * inverse**2 * (first_control.y - curve_start.y)
                + 6.0 * inverse * ratio * (second_control.y - first_control.y)
                + 3.0 * ratio**2 * (curve_goal.y - second_control.y)
            )
            yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else first.yaw
            point = PathPoint(
                x, y, normalize_angle(yaw), PathKind.CURVE_TURN, cell_id
            )
            if not point_is_safe(self.safe, point.point):
                return None
            if result and not line_is_safe(
                self.safe, result[-1].point, point.point
            ):
                return None
            append_path(result, [point])

        lead_out = densify_line(
            curve_goal,
            goal,
            self.spacing_px,
            PathKind.CURVE_TURN,
            cell_id,
            start_yaw=second.yaw,
            end_yaw=second.yaw,
        )
        append_path(result, lead_out)
        if any(
            not line_is_safe(self.safe, begin.point, end.point)
            for begin, end in zip(result, result[1:])
        ):
            return None
        result[0] = PathPoint(start.x, start.y, first.yaw, PathKind.CURVE_TURN, cell_id)
        result[-1] = PathPoint(goal.x, goal.y, second.yaw, PathKind.CURVE_TURN, cell_id)
        return result

    def _square(
        self,
        first: PathPoint,
        second: PathPoint,
        axis: SweepAxis,
        cell_id: int,
        overlap_px: float,
    ) -> list[PathPoint] | None:
        start = first.point
        goal = second.point
        blend = max(0.0, float(overlap_px))
        square_start = Point2D(
            start.x + blend * math.cos(first.yaw),
            start.y + blend * math.sin(first.yaw),
        )
        square_goal = Point2D(
            goal.x - blend * math.cos(second.yaw),
            goal.y - blend * math.sin(second.yaw),
        )
        if axis == SweepAxis.HORIZONTAL:
            corner = Point2D(square_start.x, square_goal.y)
        else:
            corner = Point2D(square_goal.x, square_start.y)

        route = [start]
        if math.hypot(square_start.x - start.x, square_start.y - start.y) > 1e-6:
            route.append(square_start)
        if math.hypot(corner.x - route[-1].x, corner.y - route[-1].y) > 1e-6:
            route.append(corner)
        if math.hypot(square_goal.x - route[-1].x, square_goal.y - route[-1].y) > 1e-6:
            route.append(square_goal)
        if math.hypot(goal.x - route[-1].x, goal.y - route[-1].y) > 1e-6:
            route.append(goal)

        if any(not point_is_safe(self.safe, point) for point in route):
            return None
        if any(
            not line_is_safe(self.safe, route_start, route_end)
            for route_start, route_end in zip(route, route[1:])
        ):
            return None

        result: list[PathPoint] = []
        current_yaw = first.yaw
        for route_start, route_end in zip(route, route[1:]):
            segment_yaw = heading(route_start, route_end, current_yaw)
            result.append(
                PathPoint(
                    route_start.x,
                    route_start.y,
                    segment_yaw,
                    PathKind.SQUARE_TURN,
                    cell_id,
                )
            )
            segment = densify_line(
                route_start,
                route_end,
                self.spacing_px,
                PathKind.SQUARE_TURN,
                cell_id,
                start_yaw=segment_yaw,
                end_yaw=segment_yaw,
            )
            append_path(result, segment)
            current_yaw = segment_yaw

        result.append(
            PathPoint(
                goal.x,
                goal.y,
                second.yaw,
                PathKind.SQUARE_TURN,
                cell_id,
            )
        )
        return result
