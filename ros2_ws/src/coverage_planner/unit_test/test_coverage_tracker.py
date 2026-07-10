import numpy as np

from coverage_planner.coverage_tracker import CoverageTracker


def test_mark_covered_vectorized_radius_only_free_cells():
    grid = np.zeros((7, 7), dtype=np.int16)
    grid[3, 4] = 100
    tracker = CoverageTracker(grid, coverage_radius_px=1)

    tracker.mark_covered(3, 3)

    assert tracker.covered[3, 3]
    assert tracker.covered[2, 3]
    assert tracker.covered[4, 3]
    assert tracker.covered[3, 2]
    assert not tracker.covered[3, 4]
    assert not tracker.covered[2, 2]


def test_empty_free_space_reports_complete():
    grid = np.full((3, 3), 100, dtype=np.int16)
    tracker = CoverageTracker(grid, coverage_radius_px=1)

    assert tracker.coverage_percentage == 100.0
