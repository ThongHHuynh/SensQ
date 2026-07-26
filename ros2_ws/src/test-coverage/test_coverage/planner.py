from __future__ import annotations

import math
import time

from test_coverage.decomposition import decompose
from test_coverage.grid_map import (
    GridMap,
    build_safe_grid,
    connected_component,
    meters_to_pixels,
)
from test_coverage.models import (
    CoveragePlan,
    PathKind,
    PlanMetrics,
    PlannerConfig,
    Point2D,
    SweepAxis,
    polyline_length,
)
from test_coverage.optimizer import assemble_route, optimize_route
from test_coverage.sweeps import create_cell_variants
from test_coverage.transitions import TransitionPlanner
from test_coverage.validator import count_kind_runs, estimate_coverage, validate_path


class BoustrophedonCoveragePlanner:
    def __init__(self, config: PlannerConfig):
        config.validate()
        self.config = config

    def plan(self, grid_map: GridMap, start_world: Point2D) -> CoveragePlan:
        started = time.perf_counter()
        resolution = grid_map.resolution
        inflation_px = meters_to_pixels(
            self.config.robot_radius_m + self.config.safety_margin_m,
            resolution,
        )
        safe = build_safe_grid(
            grid_map.occupancy,
            self.config.occupied_threshold,
            self.config.unknown_is_obstacle,
            inflation_px,
        )
        start_grid = grid_map.world_to_grid(start_world)
        safe_component, snapped_start = connected_component(safe, start_grid)
        start = Point2D(float(snapped_start[0]), float(snapped_start[1]))

        lane_spacing_px = meters_to_pixels(
            self.config.operation_width_m - self.config.overlap_m,
            resolution,
        )
        path_spacing_px = max(0.5, self.config.path_spacing_m / resolution)
        headland_depth_px = self.config.headland_depth_m / resolution
        headland_overlap_px = self.config.headland_overlap_m / resolution
        turning_radius_px = self.config.min_turning_radius_m / resolution
        minimum_lane_px = meters_to_pixels(
            self.config.min_lane_length_m, resolution
        )
        minimum_area_px = max(
            1,
            int(math.ceil(self.config.min_cell_area_m2 / (resolution * resolution))),
        )

        axes = self._candidate_axes()
        candidates: list[tuple[float, CoveragePlan]] = []
        failure_messages = []

        for axis in axes:
            cells = decompose(
                safe_component,
                axis,
                minimum_lane_px,
                minimum_area_px,
            )
            if not cells:
                failure_messages.append(f"{axis.value}: no cells")
                continue

            transition_planner = TransitionPlanner(
                safe_component,
                path_spacing_px,
                self.config.max_astar_iterations,
                minimum_turning_radius_px=turning_radius_px,
            )
            variants = {}
            failed_cells = []
            for cell in cells:
                cell_variants = create_cell_variants(
                    cell,
                    safe_component,
                    lane_spacing_px,
                    path_spacing_px,
                    headland_depth_px,
                    headland_overlap_px,
                    turning_radius_px,
                    minimum_lane_px,
                    self.config.turn_mode,
                    transition_planner,
                )
                if not cell_variants:
                    failed_cells.append(cell.cell_id)
                variants[cell.cell_id] = cell_variants
            if failed_cells:
                failure_messages.append(
                    f"{axis.value}: no feasible sweep for cells {failed_cells}"
                )
                continue

            selected, _ = optimize_route(
                cells,
                variants,
                start,
                transition_planner,
                self.config.optimize_cell_order,
            )
            points = assemble_route(selected, transition_planner)
            validation = validate_path(safe_component, points)
            if not validation.valid:
                failure_messages.append(
                    f"{axis.value}: unsafe points={validation.unsafe_points}, "
                    f"edges={validation.unsafe_edges}"
                )
                continue

            path_length_px = polyline_length(points)
            entry_distance_px = transition_planner.distance(start, points[0].point)
            coverage = estimate_coverage(
                safe_component,
                points,
                0.5 * self.config.operation_width_m / resolution,
            )
            metrics = PlanMetrics(
                axis=axis,
                cell_count=len(cells),
                path_length_m=path_length_px * resolution,
                entry_distance_m=entry_distance_px * resolution,
                estimated_coverage_percent=coverage,
                curve_turns=count_kind_runs(points, PathKind.CURVE_TURN),
                square_turns=count_kind_runs(points, PathKind.SQUARE_TURN),
                transit_segments=count_kind_runs(points, PathKind.TRANSIT),
            )
            warnings = []
            disconnected = int(np_count_nonzero(safe)) - int(
                np_count_nonzero(safe_component)
            )
            if disconnected:
                warnings.append(
                    f"{disconnected} inflated free cells are unreachable from the robot"
                )
            candidate = CoveragePlan(
                frame_id=grid_map.frame_id,
                points=points,
                metrics=metrics,
                cells=cells,
                warnings=warnings,
            )
            candidates.append(
                (metrics.path_length_m + metrics.entry_distance_m, candidate)
            )

        if not candidates:
            details = "; ".join(failure_messages) or "no feasible candidates"
            raise RuntimeError(f"coverage planning failed: {details}")

        _, best = min(candidates, key=lambda candidate: candidate[0])
        best.metrics.planning_time_ms = (time.perf_counter() - started) * 1000.0
        return best

    def _candidate_axes(self) -> list[SweepAxis]:
        if self.config.sweep_mode == "horizontal":
            return [SweepAxis.HORIZONTAL]
        if self.config.sweep_mode == "vertical":
            return [SweepAxis.VERTICAL]
        return [SweepAxis.HORIZONTAL, SweepAxis.VERTICAL]


def np_count_nonzero(array) -> int:
    # Isolated wrapper keeps numpy out of the public planner contract.
    return int(array.sum())
