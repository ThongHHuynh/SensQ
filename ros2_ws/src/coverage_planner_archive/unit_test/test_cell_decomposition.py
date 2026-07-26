import numpy as np
import pytest

from coverage_planner_archive.cell_decomposition import decompose_cells, row_intervals


def test_row_intervals_simple():
    row = np.array([100, 0, 0, 0, 0, 100, 0, 0, 100])
    intervals = row_intervals(row, min_len_px=2)
    assert intervals == [(1, 4), (6, 7)]


def test_row_intervals_min_length():
    row = np.array([0, 100, 0, 0, 0])
    intervals = row_intervals(row, min_len_px=2)
    # Single pixel run [0,0] is too short.
    assert intervals == [(2, 4)]


def test_decompose_empty_grid():
    grid = np.full((10, 10), 100, dtype=np.int16)
    cells = decompose_cells(grid, min_len_px=2, min_cell_area_px=4)
    assert cells == []


def test_decompose_single_room():
    grid = np.zeros((10, 10), dtype=np.int16)
    # Wall border.
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100

    cells = decompose_cells(grid, min_len_px=2, min_cell_area_px=4)
    assert len(cells) == 1
    assert cells[0].area_px > 0


def test_decompose_two_rooms():
    grid = np.zeros((10, 20), dtype=np.int16)
    # Vertical wall splitting into two rooms.
    grid[:, 10] = 100

    cells = decompose_cells(grid, min_len_px=2, min_cell_area_px=4)
    assert len(cells) == 2