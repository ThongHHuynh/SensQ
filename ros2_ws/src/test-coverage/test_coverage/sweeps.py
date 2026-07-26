from __future__ import annotations

import math

from test_coverage.geometry import append_path, densify_line
from test_coverage.grid_map import sweep_to_grid
from test_coverage.models import (
    Cell,
    CellPath,
    GridSegment,
    PathKind,
    PathPoint,
    Point2D,
)
from test_coverage.transitions import TransitionPlanner
from test_coverage.turns import HeadlandTurnPlanner


def sample_segments(cell: Cell, lane_spacing_px: int) -> list[GridSegment]:
    ordered = sorted(cell.segments, key=lambda segment: segment.row)
    if not ordered:
        return []
    by_row = {segment.row: segment for segment in ordered}
    first_row = ordered[0].row
    last_row = ordered[-1].row
    offset = max(0, lane_spacing_px // 2)
    rows = list(range(first_row + offset, last_row + 1, max(1, lane_spacing_px)))
    if not rows:
        rows = [(first_row + last_row) // 2]
    if last_row - rows[-1] > lane_spacing_px // 2:
        rows.append(last_row)
    available_rows = sorted(by_row)
    sampled = []
    used = set()
    for requested_row in rows:
        row = min(available_rows, key=lambda value: abs(value - requested_row))
        if row not in used:
            sampled.append(by_row[row])
            used.add(row)
    return sampled


def create_cell_variants(
    cell: Cell,
    safe,
    lane_spacing_px: int,
    path_spacing_px: float,
    headland_depth_px: float,
    headland_overlap_px: float,
    minimum_turning_radius_px: float,
    minimum_lane_length_px: int,
    turn_mode: str,
    transition_planner: TransitionPlanner,
) -> list[CellPath]:
    sampled = sample_segments(cell, lane_spacing_px)
    if not sampled:
        return []

    variants = []
    for reverse_rows in (False, True):
        row_segments = list(reversed(sampled)) if reverse_rows else list(sampled)
        for first_forward in (True, False):
            variant_id = 2 * int(reverse_rows) + int(not first_forward)
            direction_forward = first_forward
            turn_planner = HeadlandTurnPlanner(
                safe,
                path_spacing_px,
                minimum_turning_radius_px,
                turn_mode,
            )
            lane_specs = []
            for segment in row_segments:
                margin = min(
                    max(0.0, headland_depth_px),
                    max(
                        0.0,
                        0.5 * (segment.length - max(2, minimum_lane_length_px)),
                    ),
                )
                low = float(segment.start) + margin
                high = float(segment.end) - margin
                if low > high:
                    low = float(segment.start)
                    high = float(segment.end)
                start_along, end_along = (
                    (low, high) if direction_forward else (high, low)
                )
                start = sweep_to_grid(start_along, float(segment.row), cell.axis)
                end = sweep_to_grid(end_along, float(segment.row), cell.axis)
                yaw = math.atan2(end.y - start.y, end.x - start.x)
                lane_specs.append((start, end, yaw))
                direction_forward = not direction_forward

            connection_overlaps = []
            for current, following in zip(lane_specs, lane_specs[1:]):
                current_length = math.hypot(
                    current[1].x - current[0].x,
                    current[1].y - current[0].y,
                )
                following_length = math.hypot(
                    following[1].x - following[0].x,
                    following[1].y - following[0].y,
                )
                current_allowance = max(
                    0.0,
                    0.25 * (current_length - minimum_lane_length_px),
                )
                following_allowance = max(
                    0.0,
                    0.25 * (following_length - minimum_lane_length_px),
                )
                connection_overlaps.append(
                    min(
                        max(0.0, headland_overlap_px),
                        current_allowance,
                        following_allowance,
                    )
                )

            points: list[PathPoint] = []
            failed = False
            for index, (nominal_start, nominal_end, yaw) in enumerate(lane_specs):
                start_overlap = (
                    connection_overlaps[index - 1] if index > 0 else 0.0
                )
                end_overlap = (
                    connection_overlaps[index]
                    if index < len(connection_overlaps)
                    else 0.0
                )
                cosine = math.cos(yaw)
                sine = math.sin(yaw)
                start = Point2D(
                    nominal_start.x + start_overlap * cosine,
                    nominal_start.y + start_overlap * sine,
                )
                end = Point2D(
                    nominal_end.x - end_overlap * cosine,
                    nominal_end.y - end_overlap * sine,
                )
                lane = densify_line(
                    start,
                    end,
                    path_spacing_px,
                    PathKind.COVERAGE,
                    cell.cell_id,
                    start_yaw=yaw,
                    end_yaw=yaw,
                )

                if points:
                    turn = turn_planner.create(
                        points[-1],
                        lane[0],
                        cell.axis,
                        cell.cell_id,
                        overlap_px=start_overlap,
                    )
                    if turn is not None:
                        append_path(points, turn.points)
                    else:
                        transition = transition_planner.connect(
                            points[-1], lane[0], cell.cell_id
                        )
                        if not transition:
                            failed = True
                            break
                        append_path(points, transition)
                append_path(points, lane)

            if not failed and len(points) >= 2:
                variants.append(CellPath(cell.cell_id, variant_id, points))
    return variants
