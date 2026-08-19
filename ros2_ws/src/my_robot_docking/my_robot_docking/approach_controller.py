"""Corridor-following approach controller for AprilTag docking.

The robot is driven onto the dock's approach axis and then down it, instead of
being driven straight at the docked pose. Steering at the pose cuts the corner
on angled approaches, which sweeps the robot across the dock face; running the
axis cannot. Every phase is either a rotation in place or a straight line, so
the executed path stays predictable and the swept area is bounded.

The controller is deliberately free of ROS types: it consumes plain
:class:`~my_robot_docking.docking_geometry.Pose2D` values in one fixed frame
(odometry) and returns scalar velocities, so the whole state machine is unit
testable without a running graph.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from my_robot_docking.docking_geometry import (
    CorridorError,
    Pose2D,
    corridor_error,
    entry_waypoints,
    normalize_angle,
)


class ApproachState:
    """Phases of a corridor approach, in the order they normally occur."""

    ENTER_TURN = 'enter_turn'
    ENTER_DRIVE = 'enter_drive'
    ALIGN = 'align'
    RUN = 'run'
    REACHED = 'reached'

    ROTATING = (ENTER_TURN, ALIGN)


@dataclass(frozen=True)
class ApproachConfig:
    """Tuning for one corridor approach."""

    corridor_half_width: float
    entry_margin: float
    entry_yaw_tolerance: float
    entry_position_tolerance: float
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
    heading_kp: float
    cross_track_kp: float
    yaw_kp: float
    approach_taper_distance: float
    enter_drive_abort_angle: float
    run_abort_yaw: float
    dock_keepout_radius: float
    entry_arc_step: float
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
        self._waypoints: Tuple[Pose2D, ...] = ()
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
            self._state = self._initial_state(error)
            if self._state == ApproachState.ENTER_TURN:
                self._plan_entry(robot_pose)

        if self._state == ApproachState.ENTER_TURN:
            return self._enter_turn(robot_pose, error, tag_fresh)
        if self._state == ApproachState.ENTER_DRIVE:
            return self._enter_drive(robot_pose, error, tag_fresh)
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

    def _initial_state(self, error: CorridorError) -> str:
        """Pick the cheapest phase that can still reach the dock safely.

        Being off the axis or behind the docked pose needs repositioning; a
        pure heading error only needs a rotation in place.
        """

        if self._off_corridor(error):
            return ApproachState.ENTER_TURN
        if abs(error.yaw) > self.config.align_yaw_tolerance:
            return ApproachState.ALIGN
        return ApproachState.RUN

    def _off_corridor(self, error: CorridorError) -> bool:
        return (
            abs(error.cross) > self.config.corridor_half_width
            or error.along < -self.config.entry_margin
        )

    def _enter_turn(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        waypoint = self._current_waypoint()
        if waypoint is None:
            return self._transition_out_of_entry(robot_pose, error, tag_fresh)

        bearing = math.atan2(
            waypoint.y - robot_pose.y,
            waypoint.x - robot_pose.x,
        )
        heading_error = normalize_angle(bearing - robot_pose.yaw)

        if abs(heading_error) <= self.config.align_yaw_tolerance:
            self._state = ApproachState.ENTER_DRIVE
            return self._enter_drive(robot_pose, error, tag_fresh)

        return self._command(
            0.0,
            self._rotate(heading_error),
            error,
            waypoint=waypoint,
        )

    def _enter_drive(self, robot_pose, error, tag_fresh) -> ApproachCommand:
        waypoint = self._current_waypoint()
        if waypoint is None:
            return self._transition_out_of_entry(robot_pose, error, tag_fresh)

        delta_x = waypoint.x - robot_pose.x
        delta_y = waypoint.y - robot_pose.y
        remaining = math.hypot(delta_x, delta_y)

        if remaining <= self.config.entry_position_tolerance:
            self._waypoints = self._waypoints[1:]
            if self._waypoints:
                self._state = ApproachState.ENTER_TURN
                return self._enter_turn(robot_pose, error, tag_fresh)
            return self._transition_out_of_entry(robot_pose, error, tag_fresh)

        heading_error = normalize_angle(
            math.atan2(delta_y, delta_x) - robot_pose.yaw
        )
        if abs(heading_error) > self.config.enter_drive_abort_angle:
            self._state = ApproachState.ENTER_TURN
            return self._enter_turn(robot_pose, error, tag_fresh)

        # Close to the waypoint the bearing is dominated by noise, so the leg
        # is finished straight rather than chasing an ill-conditioned angle.
        angular = 0.0
        if remaining > 2.0 * self.config.entry_position_tolerance:
            angular = self._clamp(
                self.config.heading_kp * heading_error,
                -self.config.max_angular_speed,
                self.config.max_angular_speed,
            )

        return self._command(
            self._linear_speed(remaining),
            angular,
            error,
            waypoint=waypoint,
        )

    def _transition_out_of_entry(
        self,
        robot_pose,
        error,
        tag_fresh,
    ) -> ApproachCommand:
        self._waypoints = ()
        self._state = ApproachState.ALIGN
        return self._align(robot_pose, error, tag_fresh)

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
            if self._replans < self.config.max_corridor_replans:
                self._replans += 1
                self._state = ApproachState.ENTER_TURN
                self._plan_entry(robot_pose)
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

    def _plan_entry(self, robot_pose: Pose2D) -> None:
        self._waypoints = entry_waypoints(
            robot_pose,
            self._tag_pose,
            self.entry_distance,
            self.lateral_offset,
            self.yaw_offset,
            self.config.dock_keepout_radius,
            self.config.entry_arc_step,
        )

    def _current_waypoint(self) -> Optional[Pose2D]:
        return self._waypoints[0] if self._waypoints else None

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
        )

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))
