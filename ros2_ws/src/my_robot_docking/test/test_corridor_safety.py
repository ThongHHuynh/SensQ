"""Server-side anchoring and safety gating around the corridor approach."""

import math

import pytest

from my_robot_docking.approach_controller import ApproachState
from my_robot_docking.docking_geometry import Pose2D
from my_robot_docking.docking_server import DockingServer


def make_server(**overrides):
    server = DockingServer.__new__(DockingServer)
    server.minimum_tag_distance = 0.10
    server.rotation_stop_distance = 0.22
    server.rotation_clearance_exclusion = 0.60
    server.front_stop_distance = 0.22
    server.rear_stop_distance = 0.22
    server.dock_contact_distance = 0.15
    server.anchor_filter_alpha = 0.35
    server._front_clearance = math.inf
    server._rear_clearance = math.inf
    server._rotation_clearance = lambda exclude_bearing=None: math.inf
    for name, value in overrides.items():
        setattr(server, name, value)
    return server


def safety(server, state, linear, tag_distance, target_distance=0.30):
    return server._corridor_safety_reason(
        state,
        linear,
        tag_distance,
        target_distance,
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(-1.0, 0.0, 0.0),
    )


# ----------------------------------------------------------------------
# Anchor averaging and filtering
# ----------------------------------------------------------------------

def test_averaging_cancels_symmetric_observation_noise():
    poses = [
        Pose2D(1.00, 0.02, 0.10),
        Pose2D(1.02, -0.02, -0.10),
        Pose2D(0.98, 0.00, 0.00),
    ]

    mean = DockingServer._average_poses(poses)

    assert mean.x == pytest.approx(1.0)
    assert mean.y == pytest.approx(0.0)
    assert mean.yaw == pytest.approx(0.0, abs=1e-9)


def test_averaging_drops_the_worst_outlier():
    """One bad detection tilts the whole approach axis, so it is discarded."""

    good = [Pose2D(1.0, 0.0, 0.0) for _ in range(4)]
    mean = DockingServer._average_poses(good + [Pose2D(1.0, 2.0, 0.0)])

    assert mean.y == pytest.approx(0.0)


def test_averaging_wraps_yaw_across_the_branch_cut():
    poses = [
        Pose2D(0.0, 0.0, math.pi - 0.05),
        Pose2D(0.0, 0.0, -math.pi + 0.05),
    ]

    mean = DockingServer._average_poses(poses)

    assert abs(mean.yaw) == pytest.approx(math.pi, abs=1e-9)


def test_blending_low_passes_a_new_observation():
    server = make_server()

    blended = server._blend_pose(Pose2D(1.0, 0.0, 0.0), Pose2D(2.0, 0.0, 0.0))

    assert blended.x == pytest.approx(1.35)


def test_blending_takes_the_shortest_way_round_in_yaw():
    server = make_server()

    blended = server._blend_pose(
        Pose2D(0.0, 0.0, math.pi - 0.10),
        Pose2D(0.0, 0.0, -math.pi + 0.10),
    )

    assert abs(blended.yaw) > math.pi - 0.11


def test_first_observation_becomes_the_anchor_unfiltered():
    server = make_server()

    assert server._blend_pose(None, Pose2D(1.0, 2.0, 0.5)).x == 1.0


# ----------------------------------------------------------------------
# Safety gating
# ----------------------------------------------------------------------

def test_tag_inside_the_minimum_distance_always_stops():
    server = make_server()

    reason = safety(server, ApproachState.RUN, 0.1, tag_distance=0.05)

    assert reason is not None
    assert 'minimum safe distance' in reason


def test_rotating_uses_the_swept_clearance_not_the_travel_sector():
    server = make_server(
        _rotation_clearance=lambda exclude_bearing=None: 0.15,
    )

    reason = safety(server, ApproachState.ALIGN, 0.0, tag_distance=0.80)

    assert reason is not None
    assert 'rotation stop distance' in reason


def test_rotating_excludes_the_dock_from_the_swept_clearance():
    """The half turn before backing in happens right in front of the dock."""

    seen = {}

    def clearance(exclude_bearing=None):
        seen['bearing'] = exclude_bearing
        return math.inf

    server = make_server(_rotation_clearance=clearance)

    assert safety(server, ApproachState.ALIGN, 0.0, tag_distance=0.80) is None
    # Robot at (-1, 0) facing +x, tag at the origin: dead ahead.
    assert seen['bearing'] == pytest.approx(0.0)


def test_forward_travel_stops_for_an_obstacle_ahead():
    server = make_server(_front_clearance=0.15)

    reason = safety(server, ApproachState.RUN, 0.1, tag_distance=0.80)

    assert reason is not None
    assert 'Front obstacle' in reason


def test_backward_travel_stops_for_an_obstacle_behind():
    server = make_server(_rear_clearance=0.15)

    reason = safety(server, ApproachState.RUN, -0.1, tag_distance=0.80)

    assert reason is not None
    assert 'Rear obstacle' in reason


def test_the_dock_itself_does_not_trip_the_travel_stop():
    """Inside the contact band the dock fills the sector being driven into.

    Without muting, the final centimetres of every approach fault on the very
    surface the robot is aiming at.
    """

    server = make_server(_front_clearance=0.20)

    # The band ends at final_distance + dock_contact_distance = 0.45 m.
    assert safety(server, ApproachState.RUN, 0.1, tag_distance=0.60) is not None
    assert safety(server, ApproachState.RUN, 0.1, tag_distance=0.40) is None

    # Muting the sector does not mute the tag range check underneath it.
    reason = safety(server, ApproachState.RUN, 0.1, tag_distance=0.05)
    assert reason is not None and 'minimum safe distance' in reason


def test_a_stationary_robot_is_not_gated_on_travel_clearance():
    server = make_server(_front_clearance=0.05, _rear_clearance=0.05)

    assert safety(server, ApproachState.RUN, 0.0, tag_distance=0.80) is None
