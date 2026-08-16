import math
from types import SimpleNamespace

import pytest

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


def observation(x, y, normal_yaw):
    return SimpleNamespace(x=x, y=y, normal_yaw=normal_yaw)


def compute(server, tag_observation):
    return server._compute_command(
        tag_observation,
        final_distance=1.5,
        lateral_offset=0.0,
        yaw_offset=0.0,
    )


def test_forward_command_drives_toward_aligned_target():
    server = make_server()

    command, _, _, _, reached, overshot = compute(
        server,
        observation(1.7, 0.0, 0.0),
    )

    assert command.linear.x > 0.0
    assert command.angular.z == pytest.approx(0.0)
    assert not reached
    assert not overshot


def test_heading_mismatch_is_not_mistaken_for_overshoot():
    server = make_server()

    command, _, _, _, reached, overshot = compute(
        server,
        observation(-2.0, 0.0, -math.pi),
    )

    assert command.linear.x == pytest.approx(0.0)
    assert abs(command.angular.z) > 0.0
    assert not reached
    assert not overshot


def test_forward_command_stops_after_passing_approach_plane():
    server = make_server()

    command, _, _, _, reached, overshot = compute(
        server,
        observation(1.4, 0.0, 0.0),
    )

    assert command.linear.x == pytest.approx(0.0)
    assert command.angular.z == pytest.approx(0.0)
    assert not reached
    assert overshot
