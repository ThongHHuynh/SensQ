import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from test_coverage.grid_map import GridMap
from test_coverage.models import (
    Cell,
    CoveragePlan,
    GridSegment,
    PathKind,
    PathPoint,
    PlanMetrics,
    SweepAxis,
)
from test_coverage.planner_node import CoveragePlannerNode


def test_markers_show_cells_labels_path_types_and_endpoints():
    cell = Cell(
        cell_id=4,
        axis=SweepAxis.HORIZONTAL,
        segments=[
            GridSegment(row=2, start=2, end=4, cell_id=4),
            GridSegment(row=3, start=2, end=4, cell_id=4),
        ],
    )
    points = [
        PathPoint(2.0, 2.0, 0.0, PathKind.COVERAGE, 4),
        PathPoint(4.0, 2.0, 0.0, PathKind.COVERAGE, 4),
        PathPoint(4.0, 3.0, 1.57, PathKind.CURVE_TURN, 4),
    ]
    metrics = PlanMetrics(
        axis=SweepAxis.HORIZONTAL,
        cell_count=1,
        path_length_m=0.3,
        entry_distance_m=0.0,
        estimated_coverage_percent=100.0,
        curve_turns=1,
        square_turns=0,
        transit_segments=0,
    )
    plan = CoveragePlan("map", points, metrics, cells=[cell])
    grid_map = GridMap(np.zeros((8, 8), dtype=np.int16), resolution=0.1)

    markers = CoveragePlannerNode._to_markers(plan, grid_map, Time())
    namespaces = [marker.ns for marker in markers.markers]
    cell_marker = next(
        marker
        for marker in markers.markers
        if marker.ns == "coverage_cells"
    )

    assert markers.markers[0].action == Marker.DELETEALL
    assert len(cell_marker.points) == cell.area_px
    assert "coverage_cell_labels" in namespaces
    assert namespaces.count("coverage_path") == 2
    assert "coverage_start" in namespaces
    assert "coverage_end" in namespaces
