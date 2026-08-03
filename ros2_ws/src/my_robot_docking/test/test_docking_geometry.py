import math

import pytest

from my_robot_docking.docking_geometry import (
    Pose2D,
    normalize_angle,
    relative_control_error,
    reverse_target_from_tag,
    scan_sector_clearances,
)


def test_reverse_target_keeps_forward_position_and_flips_yaw():
    target = reverse_target_from_tag(
        robot_pose=Pose2D(0.0, 0.0, 0.0),
        tag_x_base=1.0,
        tag_y_base=0.0,
        tag_normal_yaw_base=0.0,
        final_distance=0.25,
        lateral_offset=0.0,
        yaw_offset=0.0,
    )

    assert target.tag_x == pytest.approx(1.0)
    assert target.tag_y == pytest.approx(0.0)
    assert target.base_pose.x == pytest.approx(0.75)
    assert target.base_pose.y == pytest.approx(0.0)
    assert abs(normalize_angle(target.base_pose.yaw - math.pi)) < 1e-9


def test_reverse_target_rotates_observation_into_odom():
    target = reverse_target_from_tag(
        robot_pose=Pose2D(2.0, 3.0, math.pi / 2.0),
        tag_x_base=1.0,
        tag_y_base=0.0,
        tag_normal_yaw_base=0.0,
        final_distance=0.25,
        lateral_offset=0.10,
        yaw_offset=0.0,
    )

    assert target.tag_x == pytest.approx(2.0)
    assert target.tag_y == pytest.approx(4.0)
    assert target.base_pose.x == pytest.approx(1.90)
    assert target.base_pose.y == pytest.approx(3.75)
    assert target.base_pose.yaw == pytest.approx(-math.pi / 2.0)


def test_reverse_error_is_positive_along_rear_travel_axis():
    error = relative_control_error(
        robot_pose=Pose2D(0.0, 0.0, math.pi),
        target_pose=Pose2D(0.75, 0.0, math.pi),
        reverse=True,
    )

    assert error.target_x == pytest.approx(-0.75)
    assert error.target_y == pytest.approx(0.0, abs=1e-12)
    assert error.longitudinal == pytest.approx(0.75)
    assert error.heading == pytest.approx(0.0, abs=1e-12)
    assert error.yaw == pytest.approx(0.0)


def test_reverse_heading_has_correct_steering_sign():
    error = relative_control_error(
        robot_pose=Pose2D(0.0, 0.0, 0.0),
        target_pose=Pose2D(-1.0, 0.20, 0.0),
        reverse=True,
    )

    # A target behind-left requires negative yaw while backing up.
    assert error.heading < 0.0
    assert error.longitudinal > 0.0


def test_lidar_yaw_pi_maps_zero_to_rear_and_pi_to_front():
    ranges = [math.inf] * 5
    ranges[0] = 0.40  # angle -pi: physical front
    ranges[2] = 0.20  # angle 0: physical rear
    front, rear = scan_sector_clearances(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=math.pi / 2.0,
        range_min=0.05,
        range_max=10.0,
        forward_angle=math.pi,
        front_half_angle=0.20,
        rear_half_angle=0.20,
    )

    assert front == pytest.approx(0.40)
    assert rear == pytest.approx(0.20)
