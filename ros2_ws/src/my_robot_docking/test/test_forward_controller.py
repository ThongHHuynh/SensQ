"""Corridor approach behaviour when the robot drives in nose-first."""

import math

import pytest

from my_robot_docking.approach_controller import (
    ApproachConfig,
    ApproachState,
    CorridorApproachController,
)
from my_robot_docking.docking_geometry import Pose2D


FINAL_DISTANCE = 0.30
ENTRY_DISTANCE = 1.00

# The dock sits at the origin with its approach axis running along world +x:
# the robot parks at (+0.30, 0) facing back toward the tag.
TAG = Pose2D(0.0, 0.0, math.pi)


def make_config(**overrides):
    values = dict(
        corridor_half_width=0.12,
        entry_margin=0.05,
        entry_yaw_tolerance=0.25,
        entry_position_tolerance=0.06,
        align_yaw_tolerance=0.05,
        distance_tolerance=0.03,
        lateral_tolerance=0.03,
        yaw_tolerance=0.08,
        overshoot_margin=0.08,
        max_linear_speed=0.10,
        min_linear_speed=0.03,
        max_angular_speed=0.35,
        min_angular_speed=0.10,
        distance_kp=0.40,
        heading_kp=0.80,
        cross_track_kp=1.20,
        yaw_kp=0.40,
        approach_taper_distance=0.25,
        enter_drive_abort_angle=0.60,
        run_abort_yaw=0.50,
        dock_keepout_radius=0.35,
        entry_arc_step=0.50,
        max_corridor_replans=3,
    )
    values.update(overrides)
    return ApproachConfig(**values)


def make_controller(reverse=False, **overrides):
    controller = CorridorApproachController(
        make_config(**overrides),
        FINAL_DISTANCE,
        ENTRY_DISTANCE,
        lateral_offset=0.0,
        yaw_offset=0.0,
        reverse=reverse,
    )
    controller.anchor(TAG)
    return controller


def test_aligned_robot_on_the_axis_runs_straight_in():
    controller = make_controller()

    update = controller.update(Pose2D(1.0, 0.0, math.pi), tag_fresh=True)

    assert update.state == ApproachState.RUN
    assert update.linear > 0.0
    assert update.angular == pytest.approx(0.0, abs=1e-9)
    assert not update.reached
    assert not update.overshot


def test_ninety_degree_approach_is_not_reported_as_overshoot():
    """The regression that made every oblique approach abort instantly.

    A robot beside the dock sits behind the docked pose's plane, so the
    along-axis error is negative from the first tick. That is ordinary
    geometry for an angled start, not a robot that drove past its goal.
    """

    controller = make_controller()

    update = controller.update(Pose2D(0.0, -1.5, math.pi / 2.0), tag_fresh=True)

    assert update.error.along < 0.0
    assert not update.overshot
    assert update.state == ApproachState.ENTER_TURN


def test_ninety_degree_approach_turns_toward_the_corridor_entry():
    controller = make_controller()

    update = controller.update(Pose2D(0.0, -1.5, math.pi / 2.0), tag_fresh=True)

    assert update.linear == pytest.approx(0.0)
    assert update.waypoint is not None
    # The entry point is on the axis, one entry distance out in front.
    assert update.waypoint.x == pytest.approx(ENTRY_DISTANCE, abs=1e-6)
    assert update.waypoint.y == pytest.approx(0.0, abs=1e-6)
    # Facing +y, the entry point is off to the right, so the robot turns right.
    assert update.angular < 0.0


def test_one_eighty_approach_aligns_in_place_without_repositioning():
    """On the axis but facing away: a rotation is enough, no entry legs."""

    controller = make_controller()

    update = controller.update(Pose2D(2.0, 0.0, 0.0), tag_fresh=True)

    assert update.state == ApproachState.ALIGN
    assert update.linear == pytest.approx(0.0)
    assert abs(update.angular) > 0.0
    assert not update.overshot


def test_half_turn_direction_is_latched_against_branch_cut_noise():
    """Near +/-pi the sign of the yaw error flips on noise.

    Without latching, the robot reverses its turn every time the estimate
    crosses the cut and never completes the half turn.
    """

    controller = make_controller()

    first = controller.update(Pose2D(2.0, 0.0, 1e-4), tag_fresh=True)
    # Same physical heading, opposite side of the branch cut.
    second = controller.update(Pose2D(2.0, 0.0, -1e-4), tag_fresh=True)

    assert first.state == ApproachState.ALIGN
    assert second.state == ApproachState.ALIGN
    assert first.angular * second.angular > 0.0


def test_cross_track_error_steers_back_onto_the_axis():
    controller = make_controller()

    # Aligned with the axis but displaced off the line. Travelling along -x,
    # the robot's left is -y, so a goal at +y sits to its right.
    update = controller.update(Pose2D(1.0, -0.05, math.pi), tag_fresh=True)

    assert update.state == ApproachState.RUN
    assert update.linear > 0.0
    assert update.error.cross == pytest.approx(-0.05, abs=1e-6)
    assert update.angular < 0.0


def test_cross_track_steering_is_symmetric():
    left = make_controller().update(Pose2D(1.0, 0.05, math.pi), tag_fresh=True)
    right = make_controller().update(Pose2D(1.0, -0.05, math.pi), tag_fresh=True)

    assert left.angular == pytest.approx(-right.angular)


def test_overshoot_is_reported_once_the_axis_run_passes_the_goal():
    """Only a robot that was running the axis can genuinely overshoot."""

    controller = make_controller()

    running = controller.update(Pose2D(0.60, 0.0, math.pi), tag_fresh=True)
    assert running.state == ApproachState.RUN

    # Aligned, on the axis, but now well past the docked pose toward the tag.
    update = controller.update(Pose2D(0.15, 0.0, math.pi), tag_fresh=True)

    assert update.overshot
    assert update.linear == pytest.approx(0.0)
    assert update.angular == pytest.approx(0.0)


def test_starting_behind_the_goal_repositions_instead_of_aborting():
    """A cold start behind the docked pose is geometry, not a control fault."""

    controller = make_controller()

    update = controller.update(Pose2D(0.15, 0.0, math.pi), tag_fresh=True)

    assert update.state == ApproachState.ENTER_TURN
    assert not update.overshot


def test_goal_is_reached_and_re_verified_while_held():
    controller = make_controller()
    docked = Pose2D(FINAL_DISTANCE, 0.0, math.pi)

    reached = controller.update(docked, tag_fresh=True)
    assert reached.reached
    assert reached.state == ApproachState.REACHED

    # Still at the goal: stays reached and commands nothing.
    held = controller.update(docked, tag_fresh=True)
    assert held.reached
    assert held.linear == pytest.approx(0.0)

    # Drifting out of tolerance during verification resumes the run.
    drifted = controller.update(
        Pose2D(FINAL_DISTANCE + 0.20, 0.0, math.pi),
        tag_fresh=True,
    )
    assert not drifted.reached
    assert drifted.state == ApproachState.RUN
    assert drifted.linear > 0.0


def test_run_does_not_start_until_the_tag_is_back_in_frame():
    """Nose-first docking is a visual servo, so alignment waits for the tag."""

    controller = make_controller()
    # On the axis but mis-aligned, so the controller starts in ALIGN.
    turning = Pose2D(1.0, 0.0, math.pi - 0.30)
    aligned = Pose2D(1.0, 0.0, math.pi)

    assert controller.update(turning, tag_fresh=False).state == (
        ApproachState.ALIGN
    )

    # Heading is now good, but the tag is still out of frame: hold position.
    stale = controller.update(aligned, tag_fresh=False)
    assert stale.state == ApproachState.ALIGN
    assert stale.linear == pytest.approx(0.0)
    assert stale.angular == pytest.approx(0.0)

    fresh = controller.update(aligned, tag_fresh=True)
    assert fresh.state == ApproachState.RUN
    assert fresh.linear > 0.0


def test_entry_legs_tolerate_losing_the_tag():
    controller = make_controller()

    controller.update(Pose2D(0.0, -1.5, math.pi / 2.0), tag_fresh=True)

    assert controller.state == ApproachState.ENTER_TURN
    assert not controller.requires_tag


def test_running_nose_first_requires_the_tag():
    controller = make_controller()

    controller.update(Pose2D(1.0, 0.0, math.pi), tag_fresh=True)

    assert controller.state == ApproachState.RUN
    assert controller.requires_tag
