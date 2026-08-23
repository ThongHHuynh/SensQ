import numpy as np

from test_coverage.decomposition import decompose, row_intervals
from test_coverage.models import SweepAxis


def test_row_intervals_filters_short_runs():
    row = np.array([0, 1, 1, 0, 1, 1, 1, 0], dtype=bool)
    assert row_intervals(row, 3) == [(4, 6)]


def test_boustrophedon_decomposition_splits_and_merges_at_obstacle():
    safe = np.ones((12, 16), dtype=bool)
    safe[4:8, 7:9] = False

    cells = decompose(
        safe,
        SweepAxis.HORIZONTAL,
        minimum_lane_length_px=2,
        minimum_cell_area_px=1,
    )

    assert len(cells) == 4
    assert sum(cell.area_px for cell in cells) == int(np.count_nonzero(safe))
    assert all(cell.axis == SweepAxis.HORIZONTAL for cell in cells)
