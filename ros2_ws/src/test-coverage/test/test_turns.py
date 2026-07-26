import math

import numpy as np

from test_coverage.models import PathKind, PathPoint, SweepAxis
from test_coverage.turns import HeadlandTurnPlanner
from test_coverage.validator import validate_path


def test_curve_headland_turn_is_smooth_and_safe():
    safe = np.ones((30, 30), dtype=bool)
    planner = HeadlandTurnPlanner(
        safe,
        spacing_px=0.25,
        minimum_turning_radius_px=2.0,
        mode="curve",
    )

    turn = planner.create(
        PathPoint(18.0, 10.0, 0.0),
        PathPoint(18.0, 14.0, math.pi),
        SweepAxis.HORIZONTAL,
        cell_id=3,
    )

    assert turn is not None
    assert turn.kind == PathKind.CURVE_TURN
    assert validate_path(safe, turn.points).valid
    assert max(point.x for point in turn.points) > 18.0


def test_curve_headland_turn_blends_into_both_lanes():
    safe = np.ones((30, 30), dtype=bool)
    planner = HeadlandTurnPlanner(
        safe,
        spacing_px=0.25,
        minimum_turning_radius_px=2.0,
        mode="curve",
    )

    turn = planner.create(
        PathPoint(17.0, 10.0, 0.0),
        PathPoint(17.0, 14.0, math.pi),
        SweepAxis.HORIZONTAL,
        cell_id=3,
        overlap_px=1.0,
    )

    assert turn is not None
    assert turn.points[0].x == 17.0
    assert turn.points[-1].x == 17.0
    assert any(abs(point.x - 18.0) < 1e-6 for point in turn.points)
    assert max(point.x for point in turn.points) > 18.0
    assert validate_path(safe, turn.points).valid


def test_square_headland_turn_uses_orthogonal_segments():
    safe = np.ones((30, 30), dtype=bool)
    planner = HeadlandTurnPlanner(
        safe,
        spacing_px=0.25,
        minimum_turning_radius_px=2.0,
        mode="square",
    )

    turn = planner.create(
        PathPoint(18.0, 10.0, 0.0),
        PathPoint(16.0, 14.0, math.pi),
        SweepAxis.HORIZONTAL,
        cell_id=3,
    )

    assert turn is not None
    assert turn.kind == PathKind.SQUARE_TURN
    assert all(point.kind == PathKind.SQUARE_TURN for point in turn.points)
    assert validate_path(safe, turn.points).valid
    assert any(
        abs(first.x - second.x) < 1e-9
        and abs(first.y - second.y) > 1e-9
        for first, second in zip(turn.points, turn.points[1:])
    )
