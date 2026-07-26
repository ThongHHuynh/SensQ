import numpy as np

from test_coverage.grid_map import GridMap, build_safe_grid, meters_to_pixels
from test_coverage.models import PlannerConfig, Point2D, SweepAxis
from test_coverage.planner import BoustrophedonCoveragePlanner
from test_coverage.segmentation import coverage_segment_starts
from test_coverage.validator import validate_path


def _test_map() -> GridMap:
    occupancy = np.zeros((50, 80), dtype=np.int16)
    occupancy[0, :] = 100
    occupancy[-1, :] = 100
    occupancy[:, 0] = 100
    occupancy[:, -1] = 100
    occupancy[15:35, 37:43] = 100
    return GridMap(occupancy, resolution=0.05)


def _config(**overrides) -> PlannerConfig:
    defaults = {
        "operation_width_m": 0.25,
        "overlap_m": 0.05,
        "robot_radius_m": 0.05,
        "safety_margin_m": 0.0,
        "headland_depth_m": 0.15,
        "headland_overlap_m": 0.05,
        "min_turning_radius_m": 0.10,
        "path_spacing_m": 0.05,
        "min_lane_length_m": 0.15,
        "min_cell_area_m2": 0.01,
        "sweep_mode": "auto",
        "turn_mode": "auto",
    }
    defaults.update(overrides)
    return PlannerConfig(**defaults)


def test_integrated_plan_is_safe_and_selects_shortest_axis():
    grid_map = _test_map()
    config = _config()
    planner = BoustrophedonCoveragePlanner(config)

    plan = planner.plan(grid_map, Point2D(0.25, 0.25))

    safe = build_safe_grid(
        grid_map.occupancy,
        config.occupied_threshold,
        config.unknown_is_obstacle,
        meters_to_pixels(
            config.robot_radius_m + config.safety_margin_m,
            grid_map.resolution,
        ),
    )
    assert validate_path(safe, plan.points).valid
    assert plan.metrics.axis == SweepAxis.HORIZONTAL
    assert plan.metrics.cell_count >= 3
    assert len(plan.cells) == plan.metrics.cell_count
    assert plan.metrics.path_length_m > 0.0
    assert plan.metrics.estimated_coverage_percent >= 85.0
    starts = coverage_segment_starts(plan.points)
    assert starts[0] == 0
    assert len(starts) > 2


def test_square_turn_mode_reaches_integrated_plan():
    plan = BoustrophedonCoveragePlanner(
        _config(sweep_mode="horizontal", turn_mode="square")
    ).plan(_test_map(), Point2D(0.25, 0.25))

    assert plan.metrics.square_turns > 0
    assert plan.metrics.curve_turns == 0
