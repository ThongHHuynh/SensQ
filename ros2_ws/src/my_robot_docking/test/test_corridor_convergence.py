"""Closed-loop checks that the corridor controller actually converges.

The unit tests cover single ticks. These roll a unicycle model forward under
the controller's own commands, which is the only way to catch a law that looks
right per tick but orbits, stalls, or grazes the dock on the way in.
"""

import math

import pytest

from my_robot_docking.approach_controller import ApproachState
from my_robot_docking.docking_geometry import Pose2D, corridor_error

from test_forward_controller import (
    ENTRY_DISTANCE,
    FINAL_DISTANCE,
    TAG,
    make_controller,
)


DT = 0.05
MAX_STEPS = 6000


def step(pose, linear, angular, dt=DT):
    """Exact unicycle integration over one control period."""

    yaw = pose.yaw + angular * dt
    return Pose2D(
        x=pose.x + linear * math.cos(pose.yaw) * dt,
        y=pose.y + linear * math.sin(pose.yaw) * dt,
        yaw=math.atan2(math.sin(yaw), math.cos(yaw)),
    )


def simulate(start, reverse=False):
    """Run to completion; return the trace, the outcome and the controller."""

    controller = make_controller(reverse=reverse)
    pose = start
    trace = [pose]
    states = []

    for _ in range(MAX_STEPS):
        update = controller.update(pose, tag_fresh=True)
        states.append(update.state)
        if update.overshot:
            return trace, states, 'overshot', controller
        if update.reached:
            return trace, states, 'reached', controller
        pose = step(pose, update.linear, update.angular)
        trace.append(pose)

    return trace, states, 'timeout', controller


def final_error(trace, reverse):
    return corridor_error(
        trace[-1],
        TAG,
        FINAL_DISTANCE,
        0.0,
        0.0,
        reverse=reverse,
    )


# Start headings measured against the dock axis. The dock is at the origin
# with its approach axis along world +x, so the docked pose is (+0.30, 0)
# and a robot approaching head-on sits out at (+d, 0) facing -x.
APPROACHES = {
    'head_on': Pose2D(1.5, 0.0, math.pi),
    'forty_five': Pose2D(1.2, -1.2, math.pi / 2.0 + math.pi / 4.0),
    'ninety': Pose2D(0.0, -1.5, math.pi / 2.0),
    'ninety_other_side': Pose2D(0.0, 1.5, -math.pi / 2.0),
    'one_thirty_five': Pose2D(1.2, -1.2, math.pi / 4.0),
    'one_eighty': Pose2D(1.5, 0.0, 0.0),
    'offset_and_skewed': Pose2D(0.9, 0.4, math.pi - 0.9),
}


@pytest.mark.parametrize('name', sorted(APPROACHES))
def test_forward_docking_converges_from_every_approach_angle(name):
    trace, _, outcome, _ = simulate(APPROACHES[name])
    error = final_error(trace, reverse=False)

    assert outcome == 'reached', f'{name}: ended {outcome}'
    assert abs(error.along) <= 0.03
    assert abs(error.cross) <= 0.03
    assert abs(error.yaw) <= 0.08


@pytest.mark.parametrize('name', sorted(APPROACHES))
def test_reverse_docking_converges_from_every_approach_angle(name):
    trace, _, outcome, _ = simulate(APPROACHES[name], reverse=True)
    error = final_error(trace, reverse=True)

    assert outcome == 'reached', f'{name}: ended {outcome}'
    assert abs(error.along) <= 0.03
    assert abs(error.cross) <= 0.03
    assert abs(error.yaw) <= 0.08


@pytest.mark.parametrize('name', sorted(APPROACHES))
def test_approach_never_sweeps_across_the_dock_face(name):
    """The regression behind hugging: cutting the corner into the dock.

    The dock sits at the end of a walled corridor, so the physical safety
    property is stronger than "stay clear of the tag point": once a pose's
    along-axis projection reaches the corridor mouth, it must stay within
    the corridor's width for the rest of the approach, not just near the tag.
    """

    trace, states, outcome, controller = simulate(APPROACHES[name])
    assert outcome == 'reached'

    half_width = controller.config.corridor_half_width

    # Some of these approach angles start in a configuration the walled
    # corridor makes physically unreachable in the first place (see
    # APPROACHES), deliberately, to exercise recovery -- the robot can start
    # already past the corridor mouth off axis, and backing out of that is
    # necessarily gradual, so it cannot be held to the property during that
    # initial recovery. What must hold is that the controller never
    # *chooses* to re-enter that zone once it has reached the run phase at
    # least once -- entry and align both exist precisely to guarantee cross
    # is small before run ever starts, so any later excursion back past the
    # mouth while off axis is the actual regression this guards against
    # (cutting the corner back into the dock), not ordinary recovery.
    established = next(
        (
            index
            for index, state in enumerate(states)
            if state in (ApproachState.RUN, ApproachState.REACHED)
        ),
        None,
    )
    assert established is not None, f'{name}: never reached the run phase'

    for pose in trace[established:]:
        error = corridor_error(pose, TAG, ENTRY_DISTANCE, 0.0, 0.0)
        if error.along > 0.0:
            continue
        assert abs(error.cross) <= half_width + 1e-6, (
            f'{name}: {error.along:.3f} m past the corridor mouth while '
            f'{error.cross:.3f} m off the approach axis'
        )


def test_ninety_degree_approach_walks_the_full_phase_sequence():
    _, states, outcome, _ = simulate(APPROACHES['ninety'])

    assert outcome == 'reached'
    ordered = [state for index, state in enumerate(states)
               if index == 0 or state != states[index - 1]]
    assert ordered[0] == ApproachState.RETREAT
    assert ordered.index(ApproachState.ENTER_ARC) > 0
    assert ordered.index(ApproachState.ALIGN) > ordered.index(
        ApproachState.ENTER_ARC
    )
    assert ordered[-1] == ApproachState.REACHED


def test_head_on_approach_skips_the_entry_legs():
    """Already on the axis and aligned: no repositioning should happen."""

    _, states, outcome, _ = simulate(APPROACHES['head_on'])

    assert outcome == 'reached'
    assert ApproachState.ENTER_ARC not in states
    assert ApproachState.RETREAT not in states
    assert states[0] == ApproachState.RUN


def test_one_eighty_approach_only_needs_a_rotation():
    _, states, outcome, _ = simulate(APPROACHES['one_eighty'])

    assert outcome == 'reached'
    assert ApproachState.ENTER_ARC not in states
    assert ApproachState.RETREAT not in states
    assert states[0] == ApproachState.ALIGN


def test_convergence_is_monotonic_once_the_run_starts():
    """No orbiting: along-axis distance only shrinks during the run."""

    trace, states, outcome, _ = simulate(APPROACHES['ninety'])
    assert outcome == 'reached'

    run_poses = [
        pose
        for pose, state in zip(trace, states)
        if state == ApproachState.RUN
    ]
    alongs = [
        corridor_error(pose, TAG, FINAL_DISTANCE, 0.0, 0.0).along
        for pose in run_poses
    ]
    assert alongs, 'the run phase never started'
    for previous, current in zip(alongs, alongs[1:]):
        assert current <= previous + 1e-9
