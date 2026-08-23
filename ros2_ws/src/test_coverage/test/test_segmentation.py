from test_coverage.models import PathKind, PathPoint
from test_coverage.segmentation import coverage_segment_starts


def _point(
    x: float,
    kind: PathKind,
    yaw: float = 0.0,
    cell_id: int = 1,
) -> PathPoint:
    return PathPoint(x, 0.0, yaw, kind, cell_id)


def test_cells_stay_whole_while_inter_cell_transits_are_isolated():
    points = [
        _point(0.0, PathKind.COVERAGE, cell_id=1),
        _point(1.0, PathKind.COVERAGE, cell_id=1),
        _point(1.0, PathKind.TRANSIT, cell_id=2),
        _point(0.5, PathKind.TRANSIT, cell_id=2),
        _point(0.4, PathKind.TRANSIT, 0.785, cell_id=2),
        _point(0.3, PathKind.TRANSIT, 0.785, cell_id=2),
        _point(0.5, PathKind.COVERAGE, cell_id=2),
        _point(1.5, PathKind.COVERAGE, cell_id=2),
        _point(1.6, PathKind.CURVE_TURN, cell_id=2),
        _point(1.7, PathKind.COVERAGE, cell_id=2),
    ]

    assert coverage_segment_starts(points) == [0, 2, 4, 6]


def test_intra_cell_fallback_transit_does_not_split_cell():
    points = [
        _point(0.0, PathKind.COVERAGE),
        _point(1.0, PathKind.TRANSIT),
        _point(1.1, PathKind.TRANSIT, 0.785),
        _point(1.2, PathKind.COVERAGE),
    ]

    assert coverage_segment_starts(points) == [0]


def test_empty_path_has_no_segments():
    assert coverage_segment_starts([]) == []
