"""Corridor-following approach controller for AprilTag docking.

The robot is driven onto the dock's approach axis and then down it, instead of
being driven straight at the docked pose. Steering at the pose cuts the corner
on angled approaches, which sweeps the robot across the dock face; running the
axis cannot.

Getting onto the axis is itself a continuous curve (``ENTER_ARC``), not a
sequence of point turns and straight legs: it is recomputed every tick from
wherever the robot actually is, with steering and translation happening at
once, so it adapts to drift instead of chasing a plan made once at the start.
The dock sits at the end of a walled corridor, so that curve is only safe far
enough from the tag; too close while still off axis, or out of along-axis
room to keep curving, ``RETREAT`` backs the robot into open space before
``ENTER_ARC`` tries again. ``ALIGN`` and ``RUN`` remain a rotation in place
and a straight line respectively, so the swept area stays bounded once the
robot is actually on the axis.

The controller is deliberately free of ROS types: it consumes plain
:class:`~my_robot_docking.docking_geometry.Pose2D` values in one fixed frame
(odometry) and returns scalar velocities, so the whole state machine is unit
testable without a running graph.
"""

from dataclasses import dataclass
import math
from typing import Optional

from my_robot_docking.docking_geometry import (
    CorridorError,
    Pose2D,
    corridor_error,
    entry_steering,
    normalize_angle,
)


class ApproachState:
    """Phases of a corridor approach, in the order they normally occur."""

    ENTER_ARC = 'enter_arc'
    RETREAT = 'retreat'
    ALIGN = 'align'
    RUN = 'run'
    REACHED = 'reached'

    ROTATING = (ALIGN,)


@dataclass(frozen=True)
class ApproachConfig:
    """Tuning for one corridor approach."""

    corridor_half_width: float
    entry_margin: float
    entry_yaw_tolerance: float
    align_yaw_tolerance: float
    distance_tolerance: float
    lateral_tolerance: float
    yaw_tolerance: float
    overshoot_margin: float
    max_linear_speed: float
    min_linear_speed: float
    max_angular_speed: float
    min_angular_speed: float
    distance_kp: float
    cross_track_kp: float
    yaw_kp: float
    approach_taper_distance: float
    run_abort_yaw: float
    entry_linear_speed: float
    dock_keepout_radius: float
    max_corridor_replans: int


@dataclass(frozen=True)
class ApproachCommand:
    """One control tick: velocities plus the reason they were chosen."""

    linear: float
    angular: float
    state: str
    error: CorridorError
    reached: bool
    overshot: bool
    waypoint: Optional[Pose2D]
    replanned: bool
    stuck: bool


class CorridorApproachController:
    """Drive a differential base onto a tag's approach axis and down it."""

    def __init__(
        self,
        config: ApproachConfig,
        target_distance: float,
        entry_distance: float,
        lateral_offset: float,
        yaw_offset: float,
        reverse: bool = False,
    ):
        if entry_distance <= target_distance:
            raise ValueError('entry_distance must exceed target_distance')

        self.config = config
        self.target_distance = target_distance
        self.entry_distance = entry_distance
        self.lateral_offset = lateral_offset
        self.yaw_offset = yaw_offset
        self.reverse = reverse

        self._tag_pose: Optional[Pose2D] = None
        self._state: Optional[str] = None
        self._turn_direction = 1.0
        self._replans = 0

    # ------------------------------------------------------------------
    # Anchoring
    # ------------------------------------------------------------------

    def anchor(self, tag_pose: Pose2D) -> None:
        """Latch the newest tag pose; the corridor is rebuilt from it.

        Re-anchoring is safe at any time. Only the docked pose and the axis
        move with it, never the entry legs already being executed, so a small
        correction nudges the goal rather than restarting the plan.
        """

        self._tag_pose = tag_pose

    @property
    def anchored(self) -> bool:
        return self._tag_pose is not None

    @property
    def tag_pose(self) -> Optional[Pose2D]:
        return self._tag_pose

    @property
    def state(self) -> Optional[str]:
        return self._state

    @property
    def replans(self) -> int:
        return self._replans

    @property
    def requires_tag(self) -> bool:
        """Whether losing the tag now should be treated as a failure.

        Entry legs run on odometry by design: the camera looks forward with a
        limited field of view, so turning toward the corridor entry point
        necessarily swings the tag out of frame. Backing in loses the tag for
        the same reason.
        """

        if self._state == ApproachState.RUN:
            return not self.reverse
        return False

    def goal_pose(self) -> Optional[Pose2D]:
        """The docked pose implied by the current anchor."""

        if self._tag_pose is None:
            return None
        error = self._error(Pose2D(0.0, 0.0, 0.0))
        travel_yaw = (
            normalize_angle(error.axis_yaw + math.pi)
            if self.reverse
            else error.axis_yaw
        )
        return Pose2D(error.goal_x, error.goal_y, travel_yaw)

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def update(self, robot_pose: Pose2D, tag_fresh: bool) -> ApproachCommand:
        """Advance the state machine one tick and return the command."""

        if self._tag_pose is None:
            raise RuntimeError('Controller was updated before it was anchored')

        error = self._error(robot_pose)

        if self._state is None:
            self._state = self._initial_state(robot_pose, error)

        if self._state == ApproachState.ENTER_ARC:
            return self._enter_arc(robot_pose, error, tag_fresh)
        if self._state == ApproachState.RETREAT:
            return self._retreat(robot_pose, error, tag_fresh)
        if self._state == ApproachState.ALIGN:
            return self._align(robot_pose, error, tag_fresh)
        if self._state == ApproachState.RUN:
            return self._run(robot_pose, error, tag_fresh)

        # Held at the goal. Verification is only meaningful if the tolerances
        # are re-checked while the robot sits there, so drifting out of them
        # drops straight back into the run phase.
        if self._within_tolerances(error):
            return self._command(0.0, 0.0, error, reached=True)
        self._state = ApproachState.RUN
        return self._run(robot_pose, error, tag_fresh)

    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------

    def _initial_state(self, robot_pose: Pose2D, error: CorridorError) -> str:
        """Pick the cheapest phase that can still reach the dock safely.

        Being off the axis or behind the docked pose needs repositioning; a
        pure heading error only needs a rotation in place.
        """

        if self._off_corridor(error):
            return self._entry_state(robot_pose)
        if abs(error.yaw) > self.config.align_yaw_tolerance:
            return ApproachState.ALIGN
        return ApproachState.RUN

    def _off_corridor(self, error: CorridorError) -> bool:
        return (
            abs(error.cross) > self.config.corridor_half_width
            or error.along < -self.config.entry_margin
        )

    def _distance_to_tag(self, robot_pose: Pose2D) -> float:
        return math.hypot(
            robot_pose.x - self._tag_pose.x, robot_pose.y - self._tag_pose.y
        )

    def _entry_state(self, robot_pose: Pose2D) -> str:
        """Choose between curving in and backing clear of the dock.

        Distance to the tag -- not along-axis position -- is what the walled
        section of the corridor actually scales with (it constrains lateral
        movement "for some distance out from the tag", not for the whole
        approach), so it is what decides whether a curve is still safe.
        """

        if self._distance_to_tag(robot_pose) <= self.config.dock_keepout_radius:
            return ApproachState.RETREAT
        return ApproachState.ENTER_ARC

    def _entry_yaw(self, error: CorridorError) -> float:
        """Heading error against the forward (nose-first) axis direction.

        Entry always drives forward regardless of ``self.reverse`` so the
        camera keeps the tag in view as long as possible; ``error.yaw`` is
        measured against the backing heading when reversing, so it is
        rotated back here.
        """

        return (
            normalize_angle(error.yaw + math.pi) if self.reverse else error.yaw
        )

    def _enter_arc(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        # corridor_half_width is a coarse "no repositioning needed at all"
        # bound, not a safe hand-off point: the run phase's Stanley term
        # divides by along-axis speed, which the taper deliberately shrinks
        # near the goal, so handing off with a residual cross error that
        # size at low speed saturates its steering. Requiring the same
        # precision the run phase already targets for itself keeps every
        # hand-off within the range that steering can actually condition on.
        #
        # Yaw is deliberately not part of this test: it converges
        # asymptotically and can take much longer than cross to close, and
        # this phase has no notion of overshoot, so waiting on it here would
        # mean driving straight through the goal at a constant speed while
        # still waiting. Align's in-place rotation is what finishes heading
        # convergence -- it does not move the robot, so it cannot re-open
        # the cross error this phase just closed.
        if abs(error.cross) <= self.config.lateral_tolerance:
            self._state = ApproachState.ALIGN
            return self._align(robot_pose, error, tag_fresh)

        # Close to the tag is only dangerous while still meaningfully off
        # axis (about to graze the dock from the side); close and already
        # converging is just the ordinary final approach and must be left
        # to finish rather than punted into a retreat it does not need.
        #
        # A large enough initial cross error can take more along-axis
        # distance to close than remains before the docked pose -- this
        # phase runs at a constant speed with no overshoot check of its own
        # (unlike run, which zeroes its speed past ``along <= 0``), so
        # without this it would cruise straight through the goal still
        # curving. Running out of runway is exactly the unsafe case retreat
        # exists for, regardless of how close cross already is.
        if (
            abs(error.cross) > self.config.corridor_half_width
            and self._distance_to_tag(robot_pose) <= self.config.dock_keepout_radius
        ) or error.along <= 0.0:
            self._state = ApproachState.RETREAT
            return self._retreat(robot_pose, error, tag_fresh)

        angular = entry_steering(
            error.cross,
            self._entry_yaw(error),
            self.config.cross_track_kp,
            self.config.yaw_kp,
            self.config.max_angular_speed,
        )
        return self._command(self.config.entry_linear_speed, angular, error)

    def _retreat(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        # Hysteresis against ``ENTER_ARC``'s own thresholds: entering the
        # curve drives toward the tag, which shrinks the keepout margin (and
        # ``along``) again, so leaving right at either entry threshold
        # chatters between the two every tick with no net progress. Backing
        # out has to clear both by a real margin before the curve is allowed
        # to try again -- in particular ``along`` must clear zero, or the two
        # states call each other back and forth against the same error with
        # no robot motion in between.
        clear_radius = self.config.dock_keepout_radius + self.config.entry_margin
        if (
            self._distance_to_tag(robot_pose) > clear_radius
            and error.along > self.config.entry_margin
        ):
            self._state = ApproachState.ENTER_ARC
            return self._enter_arc(robot_pose, error, tag_fresh)

        # Backing straight along whatever heading the robot happens to hold
        # does not, in general, move it along the corridor axis at all (a
        # heading perpendicular to the axis backs it further off-axis
        # instead). Steering the nose toward the corridor's forward
        # direction while driving in reverse means the resulting motion
        # heads away from the tag once the nose gets there, the same
        # nose/motion-direction relationship the run phase already relies
        # on for reverse docking. No cross-track term here: centering on the
        # axis is ``ENTER_ARC``'s job once back in the open region.
        angular = self._clamp(
            self.config.yaw_kp * self._entry_yaw(error),
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        )
        return self._command(-self.config.entry_linear_speed, angular, error)

    def _align(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        aligned = abs(error.yaw) <= self.config.align_yaw_tolerance
        # Driving nose-first is a visual servo, so the tag must be back in
        # frame before the run starts. Backing in cannot see it at all.
        if aligned and (tag_fresh or self.reverse):
            self._state = ApproachState.RUN
            return self._run(robot_pose, error, tag_fresh)

        if aligned:
            return self._command(0.0, 0.0, error)

        return self._command(0.0, self._rotate(error.yaw), error)

    def _within_tolerances(self, error: CorridorError) -> bool:
        return (
            abs(error.along) <= self.config.distance_tolerance
            and abs(error.cross) <= self.config.lateral_tolerance
            and abs(error.yaw) <= self.config.yaw_tolerance
        )

    def _run(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        if self._within_tolerances(error):
            self._state = ApproachState.REACHED
            return self._command(0.0, 0.0, error, reached=True)

        # A genuine overshoot is only possible while travelling down the axis.
        # Sitting behind the docked pose during entry is ordinary geometry for
        # any approach that starts off to one side, not a control failure.
        if (
            error.along < -self.config.overshoot_margin
            and abs(error.cross) <= self.config.corridor_half_width
            and abs(error.yaw) <= self.config.entry_yaw_tolerance
        ):
            return self._command(0.0, 0.0, error, overshot=True)

        # Steering only bends the heading while the robot is moving, so a
        # residual yaw error at a position that is already good can never be
        # driven out from here. A rotation in place is the only thing left.
        if (
            abs(error.along) <= self.config.distance_tolerance
            and abs(error.cross) <= self.config.lateral_tolerance
        ):
            self._state = ApproachState.ALIGN
            return self._align(robot_pose, error, tag_fresh)

        if (
            abs(error.cross) > 1.5 * self.config.corridor_half_width
            or abs(error.yaw) > self.config.run_abort_yaw
        ):
            if self._replans >= self.config.max_corridor_replans:
                return self._command(0.0, 0.0, error, stuck=True)
            self._replans += 1
            self._state = self._entry_state(robot_pose)
            return self._command(0.0, 0.0, error, replanned=True)

        speed = self._linear_speed(max(error.along, 0.0))
        if error.along <= 0.0:
            speed = 0.0

        # Stanley-style steering: hold the axis heading and close the true
        # perpendicular deviation. There is no bearing-to-goal term, so the
        # law stays well conditioned as the goal distance goes to zero.
        #
        # The cross-track term needs no sign flip when backing in. Reversing
        # mirrors the travelling frame's left against the axis, and it moves
        # the leading point from the nose to the rear; the two cancel.
        angular = self._clamp(
            self.config.yaw_kp * error.yaw
            + math.atan2(
                self.config.cross_track_kp * error.cross,
                speed + 0.05,
            ),
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        )
        linear = -speed if self.reverse else speed
        return self._command(linear, angular, error)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _error(self, robot_pose: Pose2D) -> CorridorError:
        return corridor_error(
            robot_pose,
            self._tag_pose,
            self.target_distance,
            self.lateral_offset,
            self.yaw_offset,
            reverse=self.reverse,
        )

    def _rotate(self, yaw_error: float) -> float:
        """Rotate in place toward ``yaw_error`` on a committed direction.

        At a half-turn the error sits on the +/-pi branch cut, where noise
        flips its sign and the robot would reverse mid-turn. The direction is
        therefore latched until the error is comfortably off the cut.
        """

        if abs(yaw_error) < 0.5 * math.pi:
            self._turn_direction = 1.0 if yaw_error >= 0.0 else -1.0

        speed = self._clamp(
            abs(self.config.yaw_kp * yaw_error),
            self.config.min_angular_speed,
            self.config.max_angular_speed,
        )
        return self._turn_direction * speed

    def _linear_speed(self, remaining: float) -> float:
        if remaining <= 0.0:
            return 0.0
        speed = self._clamp(
            self.config.distance_kp * remaining,
            self.config.min_linear_speed,
            self.config.max_linear_speed,
        )
        taper = self.config.approach_taper_distance
        if taper > 0.0 and remaining < taper:
            speed = min(
                speed,
                max(
                    self.config.min_linear_speed,
                    self.config.max_linear_speed * (remaining / taper),
                ),
            )
        return speed

    def _command(
        self,
        linear,
        angular,
        error,
        reached=False,
        overshot=False,
        waypoint=None,
        replanned=False,
        stuck=False,
    ) -> ApproachCommand:
        return ApproachCommand(
            linear=float(linear),
            angular=float(angular),
            state=self._state,
            error=error,
            reached=reached,
            overshot=overshot,
            waypoint=waypoint,
            replanned=replanned,
            stuck=stuck,
        )

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))
