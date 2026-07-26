import math

import numpy as np

from test_coverage.models import PathKind, PathPoint
from test_coverage.transitions import TransitionPlanner
from test_coverage.validator import validate_path


def test_astar_transition_routes_through_a_gap():
    safe = np.ones((20, 20), dtype=bool)
    safe[:, 10] = False
    safe[14, 10] = True
    planner = TransitionPlanner(safe, spacing_px=0.5)

    path = planner.connect(
        PathPoint(4.0, 4.0, 0.0),
        PathPoint(16.0, 4.0, 0.0),
    )

    assert path
    assert all(point.kind == PathKind.TRANSIT for point in path)
    assert max(point.y for point in path) >= 14.0
    assert validate_path(safe, path).valid


def test_direct_transition_preserves_lane_tangents():
    safe = np.ones((40, 40), dtype=bool)
    planner = TransitionPlanner(
        safe,
        spacing_px=1.0,
        minimum_turning_radius_px=2.4,
    )
    first = PathPoint(28.0, 20.0, -0.5 * math.pi)
    second = PathPoint(17.0, 21.0, 0.5 * math.pi)

    path = planner.connect(first, second)
    geometric_yaws = [
        math.atan2(end.y - start.y, end.x - start.x)
        for start, end in zip(path, path[1:])
    ]
    heading_changes = [
        abs(math.atan2(math.sin(end - start), math.cos(end - start)))
        for start, end in zip(geometric_yaws, geometric_yaws[1:])
    ]

    assert validate_path(safe, path).valid
    assert path[0].yaw == first.yaw
    assert path[-1].yaw == second.yaw
    assert abs(
        math.atan2(
            math.sin(geometric_yaws[0] - first.yaw),
            math.cos(geometric_yaws[0] - first.yaw),
        )
    ) < math.radians(10.0)
    assert abs(
        math.atan2(
            math.sin(geometric_yaws[-1] - second.yaw),
            math.cos(geometric_yaws[-1] - second.yaw),
        )
    ) < math.radians(10.0)
    assert max(heading_changes) < math.radians(30.0)
