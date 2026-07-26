from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class SweepAxis(str, Enum):
    HORIZONTAL = "horizontal"
    VERTICAL = "vertical"


class PathKind(str, Enum):
    COVERAGE = "coverage"
    CURVE_TURN = "curve_turn"
    SQUARE_TURN = "square_turn"
    TRANSIT = "transit"


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float
    kind: PathKind = PathKind.COVERAGE
    cell_id: int = -1

    @property
    def point(self) -> Point2D:
        return Point2D(self.x, self.y)


@dataclass(frozen=True)
class GridSegment:
    row: int
    start: int
    end: int
    cell_id: int

    @property
    def length(self) -> int:
        return self.end - self.start + 1


@dataclass
class Cell:
    cell_id: int
    axis: SweepAxis
    segments: list[GridSegment] = field(default_factory=list)

    @property
    def area_px(self) -> int:
        return sum(segment.length for segment in self.segments)

    @property
    def centroid(self) -> Point2D:
        if not self.segments:
            return Point2D(0.0, 0.0)
        along = sum(0.5 * (segment.start + segment.end) for segment in self.segments)
        across = sum(segment.row for segment in self.segments)
        count = float(len(self.segments))
        if self.axis == SweepAxis.HORIZONTAL:
            return Point2D(along / count, across / count)
        return Point2D(across / count, along / count)


@dataclass
class CellPath:
    cell_id: int
    variant_id: int
    points: list[PathPoint]

    @property
    def length_px(self) -> float:
        return polyline_length(self.points)


@dataclass
class PlanMetrics:
    axis: SweepAxis
    cell_count: int
    path_length_m: float
    entry_distance_m: float
    estimated_coverage_percent: float
    curve_turns: int
    square_turns: int
    transit_segments: int
    planning_time_ms: float = 0.0


@dataclass
class CoveragePlan:
    frame_id: str
    points: list[PathPoint]
    metrics: PlanMetrics
    cells: list[Cell] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class PlannerConfig:
    operation_width_m: float = 0.25
    overlap_m: float = 0.03
    robot_radius_m: float = 0.10
    safety_margin_m: float = 0.05
    headland_depth_m: float = 0.20
    headland_overlap_m: float = 0.05
    min_turning_radius_m: float = 0.12
    path_spacing_m: float = 0.05
    min_lane_length_m: float = 0.20
    min_cell_area_m2: float = 0.05
    occupied_threshold: int = 65
    unknown_is_obstacle: bool = True
    sweep_mode: str = "auto"
    turn_mode: str = "auto"
    max_astar_iterations: int = 100_000
    optimize_cell_order: bool = True

    def validate(self) -> None:
        positive = {
            "operation_width_m": self.operation_width_m,
            "robot_radius_m": self.robot_radius_m,
            "path_spacing_m": self.path_spacing_m,
            "min_lane_length_m": self.min_lane_length_m,
            "max_astar_iterations": float(self.max_astar_iterations),
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.overlap_m < 0.0 or self.overlap_m >= self.operation_width_m:
            raise ValueError("overlap_m must be non-negative and less than operation_width_m")
        if (
            self.safety_margin_m < 0.0
            or self.headland_depth_m < 0.0
            or self.headland_overlap_m < 0.0
        ):
            raise ValueError("safety and headland margins cannot be negative")
        if self.sweep_mode not in {"auto", "horizontal", "vertical"}:
            raise ValueError("sweep_mode must be auto, horizontal, or vertical")
        if self.turn_mode not in {"auto", "curve", "square"}:
            raise ValueError("turn_mode must be auto, curve, or square")


def polyline_length(points: list[PathPoint]) -> float:
    return sum(
        math.hypot(end.x - start.x, end.y - start.y)
        for start, end in zip(points, points[1:])
    )
