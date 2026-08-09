from __future__ import annotations

import heapq
import math

import numpy as np

from test_coverage.geometry import (
    densify_line,
    line_is_safe,
    normalize_angle,
    point_is_safe,
)
from test_coverage.models import PathKind, PathPoint, Point2D


_NEIGHBORS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


class TransitionPlanner:
    def __init__(
        self,
        safe: np.ndarray,
        spacing_px: float,
        max_iterations: int = 100_000,
        minimum_turning_radius_px: float = 0.0,
    ):
        self.safe = safe
        self.spacing_px = max(0.25, spacing_px)
        self.max_iterations = max(1, int(max_iterations))
        self.minimum_turning_radius_px = max(
            0.0,
            float(minimum_turning_radius_px),
        )
        self._cache: dict[tuple[int, int, int, int], list[tuple[int, int]]] = {}

    def connect(
        self,
        first: PathPoint,
        second: PathPoint,
        cell_id: int = -1,
    ) -> list[PathPoint]:
        start = first.point
        goal = second.point
        if line_is_safe(self.safe, start, goal):
            curve = self._tangent_curve(first, second, cell_id)
            if curve is not None:
                return curve
            return densify_line(
                start,
                goal,
                self.spacing_px,
                PathKind.TRANSIT,
                cell_id,
                start_yaw=first.yaw,
                end_yaw=second.yaw,
            )

        route = self._astar(start, goal)
        if not route:
            return []
        simplified = simplify_grid_route(route)
        points: list[PathPoint] = []
        for route_start, route_end in zip(simplified, simplified[1:]):
            start_point = Point2D(float(route_start[0]), float(route_start[1]))
            end_point = Point2D(float(route_end[0]), float(route_end[1]))
            segment = densify_line(
                start_point,
                end_point,
                self.spacing_px,
                PathKind.TRANSIT,
                cell_id,
            )
            if points:
                segment = segment[1:]
            points.extend(segment)
        if points:
            points[0] = PathPoint(
                points[0].x,
                points[0].y,
                first.yaw,
                PathKind.TRANSIT,
                cell_id,
            )
            points[-1] = PathPoint(
                points[-1].x,
                points[-1].y,
                second.yaw,
                PathKind.TRANSIT,
                cell_id,
            )
        return points

    def _tangent_curve(
        self,
        first: PathPoint,
        second: PathPoint,
        cell_id: int,
    ) -> list[PathPoint] | None:
        start = first.point
        goal = second.point
        separation = math.hypot(goal.x - start.x, goal.y - start.y)
        if separation <= 1e-9:
            return None

        direct_yaw = math.atan2(goal.y - start.y, goal.x - start.x)
        start_error = abs(normalize_angle(first.yaw - direct_yaw))
        end_error = abs(normalize_angle(second.yaw - direct_yaw))
        if max(start_error, end_error) < math.radians(5.0):
            return None

        control_distance = max(
            self.minimum_turning_radius_px,
            0.40 * separation,
        )
        first_control = Point2D(
            start.x + control_distance * math.cos(first.yaw),
            start.y + control_distance * math.sin(first.yaw),
        )
        second_control = Point2D(
            goal.x - control_distance * math.cos(second.yaw),
            goal.y - control_distance * math.sin(second.yaw),
        )
        approximate_length = separation + 2.0 * control_distance
        divisions = max(
            8,
            int(math.ceil(approximate_length / self.spacing_px)),
        )
        result: list[PathPoint] = []

        for index in range(divisions + 1):
            ratio = index / divisions
            inverse = 1.0 - ratio
            x = (
                inverse**3 * start.x
                + 3.0 * inverse**2 * ratio * first_control.x
                + 3.0 * inverse * ratio**2 * second_control.x
                + ratio**3 * goal.x
            )
            y = (
                inverse**3 * start.y
                + 3.0 * inverse**2 * ratio * first_control.y
                + 3.0 * inverse * ratio**2 * second_control.y
                + ratio**3 * goal.y
            )
            dx = (
                3.0 * inverse**2 * (first_control.x - start.x)
                + 6.0
                * inverse
                * ratio
                * (second_control.x - first_control.x)
                + 3.0 * ratio**2 * (goal.x - second_control.x)
            )
            dy = (
                3.0 * inverse**2 * (first_control.y - start.y)
                + 6.0
                * inverse
                * ratio
                * (second_control.y - first_control.y)
                + 3.0 * ratio**2 * (goal.y - second_control.y)
            )
            yaw = (
                math.atan2(dy, dx)
                if abs(dx) + abs(dy) > 1e-9
                else first.yaw
            )
            point = PathPoint(
                x,
                y,
                normalize_angle(yaw),
                PathKind.TRANSIT,
                cell_id,
            )
            if not point_is_safe(self.safe, point.point):
                return None
            if result and not line_is_safe(
                self.safe,
                result[-1].point,
                point.point,
            ):
                return None
            result.append(point)

        result[0] = PathPoint(
            start.x,
            start.y,
            first.yaw,
            PathKind.TRANSIT,
            cell_id,
        )
        result[-1] = PathPoint(
            goal.x,
            goal.y,
            second.yaw,
            PathKind.TRANSIT,
            cell_id,
        )
        return result

    def distance(self, first: Point2D, second: Point2D) -> float:
        if line_is_safe(self.safe, first, second):
            return math.hypot(second.x - first.x, second.y - first.y)
        route = self._astar(first, second)
        if not route:
            return math.inf
        return sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(route, route[1:])
        )

    def _astar(self, first: Point2D, second: Point2D) -> list[tuple[int, int]]:
        start = (int(round(first.x)), int(round(first.y)))
        goal = (int(round(second.x)), int(round(second.y)))
        key = (*start, *goal)
        reverse_key = (*goal, *start)
        if key in self._cache:
            return list(self._cache[key])
        if reverse_key in self._cache:
            return list(reversed(self._cache[reverse_key]))
        if not point_is_safe(self.safe, Point2D(*start)) or not point_is_safe(
            self.safe, Point2D(*goal)
        ):
            return []

        frontier = [(0.0, start)]
        cost = {start: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        visited: set[tuple[int, int]] = set()
        height, width = self.safe.shape
        iterations = 0

        while frontier and iterations < self.max_iterations:
            iterations += 1
            _, current = heapq.heappop(frontier)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                route = reconstruct(parent, goal)
                self._cache[key] = route
                return list(route)

            x, y = current
            for dx, dy, movement_cost in _NEIGHBORS:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height and self.safe[ny, nx]):
                    continue
                if dx and dy and (
                    not self.safe[y, nx] or not self.safe[ny, x]
                ):
                    continue
                neighbor = (nx, ny)
                candidate_cost = cost[current] + movement_cost
                if candidate_cost >= cost.get(neighbor, math.inf):
                    continue
                cost[neighbor] = candidate_cost
                parent[neighbor] = current
                priority = candidate_cost + octile(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
        return []


def reconstruct(
    parent: dict[tuple[int, int], tuple[int, int]], goal: tuple[int, int]
) -> list[tuple[int, int]]:
    route = [goal]
    while route[-1] in parent:
        route.append(parent[route[-1]])
    route.reverse()
    return route


def octile(first: tuple[int, int], second: tuple[int, int]) -> float:
    dx = abs(first[0] - second[0])
    dy = abs(first[1] - second[1])
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


def simplify_grid_route(route: list[tuple[int, int]]) -> list[tuple[int, int]]:
    if len(route) < 3:
        return route
    simplified = [route[0]]
    previous_direction = None
    for index in range(1, len(route)):
        direction = (
            route[index][0] - route[index - 1][0],
            route[index][1] - route[index - 1][1],
        )
        if previous_direction is not None and direction != previous_direction:
            simplified.append(route[index - 1])
        previous_direction = direction
    simplified.append(route[-1])
    return simplified
