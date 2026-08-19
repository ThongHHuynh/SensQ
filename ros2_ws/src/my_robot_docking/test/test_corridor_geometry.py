"""Unit tests for the approach-axis geometry helpers."""

import math

import pytest

from my_robot_docking.docking_geometry import (
    Pose2D,
    corridor_error,
    entry_waypoints,
    minimum_range_excluding_sector,
    normalize_angle,
    segment_point_distance,
    transform_pose,
)


# Tag at the origin with its normal along -x: the robot approaches from +x.
TAG = Pose2D(0.0, 0.0, math.pi)


def error(robot, distance=0.30, reverse=False):
    return corridor_error(robot, TAG, distance, 0.0, 0.0, reverse=reverse)


def test_along_and_cross_vanish_at_the_docked_pose():
    result = error(Pose2D(0.30, 0.0, math.pi))

    assert result.along == pytest.approx(0.0, abs=1e-9)
    assert result.cross == pytest.approx(0.0, abs=1e-9)
    assert result.yaw == pytest.approx(0.0, abs=1e-9)


def test_along_measures_travel_left_on_the_axis():
    assert error(Pose2D(1.30, 0.0, math.pi)).along == pytest.approx(1.0)


def test_along_is_negative_behind_the_docked_pose():
    """The sign that used to abort every angled approach as an overshoot."""

    assert error(Pose2D(0.0, -1.5, math.pi / 2.0)).along == pytest.approx(-0.30)


def test_cross_is_perpendicular_deviation_not_a_bearing():
    """A bearing to the goal grows with distance; cross-track does not."""

    near = error(Pose2D(0.60, -0.05, math.pi))
    far = error(Pose2D(2.00, -0.05, math.pi))

    assert near.cross == pytest.approx(far.cross)
    assert near.cross == pytest.approx(-0.05)


def test_reverse_yaw_is_measured_against_the_backing_heading():
    forward = error(Pose2D(1.0, 0.0, math.pi))
    reverse = error(Pose2D(1.0, 0.0, math.pi), reverse=True)

    assert forward.yaw == pytest.approx(0.0, abs=1e-9)
    assert abs(reverse.yaw) == pytest.approx(math.pi, abs=1e-9)


def test_lateral_offset_shifts_the_goal_across_the_axis():
    offset = corridor_error(Pose2D(1.0, 0.0, math.pi), TAG, 0.30, 0.10, 0.0)

    assert offset.along == pytest.approx(0.70)
    assert offset.cross == pytest.approx(0.10)


def test_transform_pose_round_trips_through_a_moved_frame():
    frame = Pose2D(2.0, -1.0, math.pi / 3.0)
    local = Pose2D(0.5, 0.25, -0.4)

    world = transform_pose(frame, local)
    back = transform_pose(
        Pose2D(
            -(
                math.cos(-frame.yaw) * frame.x - math.sin(-frame.yaw) * frame.y
            ),
            -(
                math.sin(-frame.yaw) * frame.x + math.cos(-frame.yaw) * frame.y
            ),
            -frame.yaw,
        ),
        world,
    )

    assert back.x == pytest.approx(local.x)
    assert back.y == pytest.approx(local.y)
    assert back.yaw == pytest.approx(local.yaw)


def test_segment_point_distance_clamps_to_the_endpoints():
    assert segment_point_distance(0, 0, 1, 0, 2, 0) == pytest.approx(1.0)
    assert segment_point_distance(0, 0, 1, 0, 0.5, 0.25) == pytest.approx(0.25)
    assert segment_point_distance(0, 0, 0, 0, 3, 4) == pytest.approx(5.0)


def test_clear_run_to_the_entry_point_is_a_single_leg():
    waypoints = entry_waypoints(
        Pose2D(0.0, -1.5, math.pi / 2.0),
        TAG,
        entry_distance=1.0,
        lateral_offset=0.0,
        yaw_offset=0.0,
        keepout_radius=0.35,
        arc_step_angle=0.5,
    )

    assert len(waypoints) == 1
    assert waypoints[0].x == pytest.approx(1.0)
    assert waypoints[0].y == pytest.approx(0.0, abs=1e-9)


def test_a_run_that_would_cross_the_dock_face_orbits_instead():
    """From behind the dock, the straight line to the entry point runs over
    the tag, so the legs must go around it."""

    robot = Pose2D(-1.0, 0.02, 0.0)
    waypoints = entry_waypoints(
        robot,
        TAG,
        entry_distance=1.0,
        lateral_offset=0.0,
        yaw_offset=0.0,
        keepout_radius=0.35,
        arc_step_angle=0.5,
    )

    assert len(waypoints) > 1
    legs = [robot] + list(waypoints)
    for start, end in zip(legs, legs[1:]):
        clearance = segment_point_distance(
            start.x, start.y, end.x, end.y, TAG.x, TAG.y
        )
        assert clearance >= 0.35 - 1e-6, f'leg clears only {clearance:.3f} m'
    assert waypoints[-1].x == pytest.approx(1.0)
    assert waypoints[-1].y == pytest.approx(0.0, abs=1e-9)


def test_orbit_legs_respect_the_arc_step():
    waypoints = entry_waypoints(
        Pose2D(-1.0, 0.02, 0.0),
        TAG,
        entry_distance=1.0,
        lateral_offset=0.0,
        yaw_offset=0.0,
        keepout_radius=0.35,
        arc_step_angle=0.4,
    )

    bearings = [math.atan2(point.y, point.x) for point in waypoints]
    for previous, current in zip(bearings, bearings[1:]):
        assert abs(normalize_angle(current - previous)) <= 0.4 + 1e-6


def scan(bearings_to_ranges, forward_angle=0.0):
    """Build a 360-sample scan from a {bearing: range} mapping."""

    increment = 2.0 * math.pi / 360.0
    ranges = []
    for index in range(360):
        bearing = normalize_angle(-math.pi + index * increment)
        value = math.inf
        for target, distance in bearings_to_ranges.items():
            if abs(normalize_angle(bearing - target)) < increment:
                value = distance
        ranges.append(value)
    return ranges, -math.pi, increment


def test_rotation_clearance_sees_the_closest_return():
    ranges, angle_min, increment = scan({0.0: 0.5, math.pi / 2.0: 0.2})

    closest = minimum_range_excluding_sector(
        ranges, angle_min, increment, 0.05, 10.0, 0.0, 0.0, 0.0
    )

    assert closest == pytest.approx(0.2)


def test_rotation_clearance_ignores_the_docks_own_bearing():
    """Parked in front of a dock, the dock is the nearest thing all round.

    Counting it would veto every rotation near a station, which is what made
    retry recovery stop with a safety fault every time.
    """

    ranges, angle_min, increment = scan({0.0: 0.25, math.pi: 0.9})

    with_dock = minimum_range_excluding_sector(
        ranges, angle_min, increment, 0.05, 10.0, 0.0, 0.0, 0.0
    )
    without_dock = minimum_range_excluding_sector(
        ranges, angle_min, increment, 0.05, 10.0, 0.0, 0.0, 0.6
    )

    assert with_dock == pytest.approx(0.25)
    assert without_dock == pytest.approx(0.9)


def test_rotation_clearance_still_reports_obstacles_outside_the_dock_sector():
    ranges, angle_min, increment = scan({0.0: 0.25, 1.2: 0.18})

    closest = minimum_range_excluding_sector(
        ranges, angle_min, increment, 0.05, 10.0, 0.0, 0.0, 0.6
    )

    assert closest == pytest.approx(0.18)


def test_rotation_clearance_respects_the_scan_mounting_angle():
    """The lidar is mounted a half turn from the base, so pi is dead ahead."""

    ranges, angle_min, increment = scan({math.pi: 0.3, 0.0: 0.9})

    closest = minimum_range_excluding_sector(
        ranges, angle_min, increment, 0.05, 10.0, math.pi, 0.0, 0.6
    )

    # Excluding the robot-frame bearing 0 removes the return at scan angle pi.
    assert closest == pytest.approx(0.9)
