"""Unit tests for the approach-axis geometry helpers."""

import math

import pytest

from my_robot_docking.docking_geometry import (
    Pose2D,
    corridor_error,
    entry_steering,
    minimum_range_excluding_sector,
    normalize_angle,
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


def test_entry_steering_is_zero_when_already_on_axis():
    assert entry_steering(0.0, 0.0, 1.2, 0.4, 0.35) == pytest.approx(0.0)


def test_entry_steering_points_at_the_axis_when_far_off_it():
    """A robot far off the centerline should steer hard toward it, bounded
    by max_angular_speed rather than the raw proportional term."""

    angular = entry_steering(5.0, 0.0, 1.2, 0.4, 0.35)

    assert angular == pytest.approx(0.35)


def test_entry_steering_sign_matches_the_run_phase_convention():
    """A positive cross-track error (goal to the robot's left) should steer
    the same direction the run phase's Stanley term already does."""

    left = entry_steering(0.05, 0.0, 1.2, 0.4, 0.35)
    right = entry_steering(-0.05, 0.0, 1.2, 0.4, 0.35)

    assert left == pytest.approx(-right)
    assert left > 0.0


def test_entry_steering_never_saturates_from_a_zero_speed_term():
    """Unlike atan2(k*cross, speed), this law has no speed denominator, so a
    large cross-track error at any heading stays a bounded, finite command
    rather than the stationary-spin saturation the old entry legs could hit."""

    angular = entry_steering(cross=10.0, yaw=1.5, cross_track_gain=1.2, yaw_gain=0.4, max_angular_speed=0.35)

    assert math.isfinite(angular)
    assert abs(angular) <= 0.35 + 1e-9


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
