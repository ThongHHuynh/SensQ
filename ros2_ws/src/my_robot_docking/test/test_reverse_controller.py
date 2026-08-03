import math

import pytest

from my_robot_docking.docking_geometry import (
    Pose2D,
    relative_control_error,
)
from my_robot_docking.docking_server import DockingServer


def make_server():
    """Create only the state needed by the pure controller method."""

    server = DockingServer.__new__(DockingServer)
    server.distance_tolerance = 0.03
    server.lateral_tolerance = 0.03
    server.yaw_tolerance = 0.08
    server.rotate_in_place_threshold = 0.35
    server.max_linear_speed = 0.10
    server.max_angular_speed = 0.40
    server.coarse_approach_distance = 0.70
    server.distance_kp = 0.40
    server.heading_kp = 1.20
    server.lateral_kp = 0.80
    server.yaw_kp = 0.60
    return server


def reverse_error(robot_pose, target_pose):
    return relative_control_error(robot_pose, target_pose, reverse=True)


def test_reverse_command_drives_backward_when_aligned():
    server = make_server()
    error = reverse_error(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(-1.0, 0.0, 0.0),
    )

    command, reached, overshot = server._compute_reverse_command(error)

    assert command.linear.x < 0.0
    assert command.angular.z == pytest.approx(0.0)
    assert not reached
    assert not overshot


def test_reverse_command_steers_correctly_toward_behind_left_target():
    server = make_server()
    error = reverse_error(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(-1.0, 0.20, 0.0),
    )

    command, reached, overshot = server._compute_reverse_command(error)

    assert command.linear.x < 0.0
    assert command.angular.z < 0.0
    assert not reached
    assert not overshot


def test_reverse_command_stops_when_target_is_reached():
    server = make_server()
    error = reverse_error(
        Pose2D(0.75, 0.0, math.pi),
        Pose2D(0.75, 0.0, math.pi),
    )

    command, reached, overshot = server._compute_reverse_command(error)

    assert command.linear.x == pytest.approx(0.0)
    assert command.angular.z == pytest.approx(0.0)
    assert reached
    assert not overshot


def test_reverse_command_stops_after_passing_target():
    server = make_server()
    error = reverse_error(
        Pose2D(0.80, 0.0, math.pi),
        Pose2D(0.75, 0.0, math.pi),
    )

    command, reached, overshot = server._compute_reverse_command(error)

    assert command.linear.x == pytest.approx(0.0)
    assert command.angular.z == pytest.approx(0.0)
    assert not reached
    assert overshot
