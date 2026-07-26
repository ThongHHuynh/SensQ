from coverage_planner_archive.coverage_geometry import Cell, Segment
from coverage_planner_archive.sweep_generator import order_cells


def make_cell(cell_id, x1, x2, y):
    return Cell(cell_id, [Segment(y, x1, x2, cell_id)])


def test_order_cells_starts_near_robot():
    near = make_cell(1, 1, 2, 1)
    far = make_cell(2, 20, 21, 20)

    ordered = order_cells([far, near], start_x=0, start_y=0)

    assert ordered[0].id == near.id
