import numpy as np

from coverage_planner_archive.coverage_geometry import Waypoint
from coverage_planner_archive.transition_planner import astar_connect, connect, line_safe


def test_line_safe_open_space():
    grid = np.zeros((20, 20), dtype=np.int16)
    assert line_safe(Waypoint(2, 2, 0.0), Waypoint(18, 2, 0.0), grid, step_px=1)


def test_line_safe_through_wall():
    grid = np.zeros((20, 20), dtype=np.int16)
    grid[:, 10] = 100
    assert not line_safe(
        Waypoint(2, 5, 0.0),
        Waypoint(18, 5, 0.0),
        grid,
        step_px=1,
    )


def test_connect_does_not_cut_blocked_diagonal_corner():
    grid = np.zeros((3, 3), dtype=np.int16)
    grid[0, 1] = 100
    grid[1, 0] = 100

    result = connect(
        Waypoint(0, 0, 0.0),
        Waypoint(1, 1, 0.0),
        grid,
        validation_step_px=1,
        sample_px=1,
        use_diagonal=True,
    )

    assert result == []


def test_astar_respects_iteration_limit():
    grid = np.zeros((20, 20), dtype=np.int16)

    result = astar_connect(
        Waypoint(0, 0, 0.0),
        Waypoint(19, 19, 0.0),
        grid,
        sample_px=1,
        max_iterations=1,
    )

    assert result == []
