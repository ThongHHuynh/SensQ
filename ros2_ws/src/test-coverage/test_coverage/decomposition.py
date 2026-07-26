from __future__ import annotations

import numpy as np

from test_coverage.models import Cell, GridSegment, SweepAxis


def row_intervals(row: np.ndarray, minimum_length: int) -> list[tuple[int, int]]:
    indices = np.flatnonzero(row)
    if len(indices) == 0:
        return []
    runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [
        (int(run[0]), int(run[-1]))
        for run in runs
        if len(run) >= max(1, minimum_length)
    ]


def intervals_overlap(first: tuple[int, int], second: tuple[int, int]) -> bool:
    return first[0] <= second[1] and second[0] <= first[1]


def decompose(
    safe: np.ndarray,
    axis: SweepAxis,
    minimum_lane_length_px: int,
    minimum_cell_area_px: int,
) -> list[Cell]:
    sweep_grid = safe if axis == SweepAxis.HORIZONTAL else safe.T
    cells: dict[int, Cell] = {}
    previous: list[GridSegment] = []
    next_cell_id = 0

    for row_index, row in enumerate(sweep_grid):
        current = row_intervals(row, minimum_lane_length_px)
        if not current:
            previous = []
            continue

        previous_to_current = [
            [
                current_index
                for current_index, interval in enumerate(current)
                if intervals_overlap((segment.start, segment.end), interval)
            ]
            for segment in previous
        ]
        current_to_previous = [
            [
                previous_index
                for previous_index, segment in enumerate(previous)
                if intervals_overlap((segment.start, segment.end), interval)
            ]
            for interval in current
        ]

        active: list[GridSegment] = []
        for current_index, interval in enumerate(current):
            predecessor_indices = current_to_previous[current_index]
            continues = (
                len(predecessor_indices) == 1
                and len(previous_to_current[predecessor_indices[0]]) == 1
            )
            if continues:
                cell_id = previous[predecessor_indices[0]].cell_id
            else:
                cell_id = next_cell_id
                next_cell_id += 1

            segment = GridSegment(
                row=row_index,
                start=interval[0],
                end=interval[1],
                cell_id=cell_id,
            )
            cells.setdefault(cell_id, Cell(cell_id=cell_id, axis=axis))
            cells[cell_id].segments.append(segment)
            active.append(segment)
        previous = active

    retained = [
        cell
        for cell in cells.values()
        if cell.area_px >= max(1, minimum_cell_area_px)
    ]
    retained.sort(key=lambda cell: cell.cell_id)
    return retained
