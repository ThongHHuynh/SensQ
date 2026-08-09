import math

from test_coverage.geometry import normalize_angle
from test_coverage.models import PathKind, PathPoint


_TRANSIT_CORNER_ANGLE = math.radians(20.0)


def coverage_segment_starts(points: list[PathPoint]) -> list[int]:
    """Keep cells whole while isolating inter-cell transit legs."""
    if not points:
        return []
    starts = [0]
    in_inter_cell_transit = False
    for index in range(1, len(points)):
        begins_inter_cell_transit = (
            points[index].kind == PathKind.TRANSIT
            and points[index].cell_id != points[index - 1].cell_id
        )
        ends_inter_cell_transit = (
            in_inter_cell_transit
            and points[index].kind != PathKind.TRANSIT
            and points[index - 1].kind == PathKind.TRANSIT
        )
        begins_inter_cell_transit_leg = (
            in_inter_cell_transit
            and points[index].kind == PathKind.TRANSIT
            and points[index - 1].kind == PathKind.TRANSIT
            and abs(
                normalize_angle(points[index].yaw - points[index - 1].yaw)
            )
            >= _TRANSIT_CORNER_ANGLE
        )
        if (
            begins_inter_cell_transit
            or ends_inter_cell_transit
            or begins_inter_cell_transit_leg
        ):
            starts.append(index)
        if begins_inter_cell_transit:
            in_inter_cell_transit = True
        elif ends_inter_cell_transit:
            in_inter_cell_transit = False
    return starts
