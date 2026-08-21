"""Corridor approach behaviour when the robot backs into the dock."""

import math

import pytest

from my_robot_docking.approach_controller import ApproachState
from my_robot_docking.docking_geometry import Pose2D

from test_forward_controller import (
    ENTRY_DISTANCE,
    FINAL_DISTANCE,
    TAG,
    make_controller,
)


def reverse_controller(**overrides):
    return make_controller(reverse=True, **overrides)


def test_reverse_align_turns_the_rear_toward_the_dock():
    """Arriving nose-first, the half turn is what ALIGN exists to do."""

    controller = reverse_controller()

    # On the axis, facing the tag: the robot must end up facing away from it.
    update = controller.update(Pose2D(1.0, 0.0, math.pi), tag_fresh=True)

    assert update.state == ApproachState.ALIGN
    assert update.linear == pytest.approx(0.0)
    assert abs(update.angular) > 0.0


def test_reverse_run_drives_backward_down_the_axis():
    controller = reverse_controller()

    # Aligned for backing in: rear points along the axis toward the tag.
    update = controller.update(Pose2D(1.0, 0.0, 0.0), tag_fresh=False)

    assert update.state == ApproachState.RUN
    assert update.linear < 0.0
    assert update.angular == pytest.approx(0.0, abs=1e-9)
    assert update.error.along == pytest.approx(0.70, abs=1e-6)


def test_reverse_cross_track_steers_the_rear_back_onto_the_axis():
    """Backing up inverts the steering sign: the rear leads, not the nose."""

    controller = reverse_controller()

    update = controller.update(Pose2D(1.0, 0.05, 0.0), tag_fresh=False)
    mirrored = reverse_controller().update(
        Pose2D(1.0, -0.05, 0.0),
        tag_fresh=False,
    )

    assert update.state == ApproachState.RUN
    assert update.linear < 0.0
    assert update.error.cross == pytest.approx(0.05, abs=1e-6)
    # Turning counter-clockwise swings the nose to +y and so the rear to -y,
    # which is the direction that closes a positive cross-track error.
    assert update.angular > 0.0
    assert mirrored.angular == pytest.approx(-update.angular)


def test_reverse_run_does_not_require_the_tag():
    """The camera looks forward, so backing in is odometry-guided by design."""

    controller = reverse_controller()
    controller.update(Pose2D(1.0, 0.0, 0.0), tag_fresh=False)

    assert controller.state == ApproachState.RUN
    assert not controller.requires_tag


def test_reverse_reaches_the_docked_pose():
    controller = reverse_controller()

    update = controller.update(Pose2D(FINAL_DISTANCE, 0.0, 0.0), tag_fresh=False)

    assert update.reached
    assert update.linear == pytest.approx(0.0)
    assert update.angular == pytest.approx(0.0)


def test_reverse_ninety_degree_start_backs_clear_of_the_corridor_first():
    """Entry legs always drive forward once curving in, so the tag stays in
    frame as long as possible; only the last stretch is blind. This start is
    level with the tag but off axis, past the corridor mouth in along-axis
    terms, so recovery starts with the same wall-safe backout used forward
    (along/cross do not depend on ``reverse``, only the final run does)."""

    controller = reverse_controller()

    update = controller.update(Pose2D(0.0, -1.5, math.pi / 2.0), tag_fresh=True)

    assert update.state == ApproachState.RETREAT
    assert not update.overshot
    assert update.linear < 0.0


def test_reverse_entry_distance_bounds_the_blind_leg():
    """The blind rear-first stretch is entry minus final, nothing more."""

    controller = reverse_controller()
    controller.update(Pose2D(ENTRY_DISTANCE, 0.0, 0.0), tag_fresh=False)

    assert controller.state == ApproachState.RUN
    goal = controller.goal_pose()
    assert math.hypot(goal.x - ENTRY_DISTANCE, goal.y) == pytest.approx(
        ENTRY_DISTANCE - FINAL_DISTANCE,
        abs=1e-6,
    )


def test_reverse_goal_pose_faces_away_from_the_tag():
    controller = reverse_controller()
    controller.anchor(TAG)

    goal = controller.goal_pose()

    assert goal.x == pytest.approx(FINAL_DISTANCE, abs=1e-6)
    assert goal.y == pytest.approx(0.0, abs=1e-6)
    # The tag normal is pi; backing in means holding the opposite heading.
    assert goal.yaw == pytest.approx(0.0, abs=1e-6)
