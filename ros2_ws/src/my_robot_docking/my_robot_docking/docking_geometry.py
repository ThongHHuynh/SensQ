"""Pure 2D geometry helpers shared by docking control and tests."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in a common parent frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ReverseDockTarget:
    """Frozen rear-first base goal and observed tag position."""

    base_pose: Pose2D
    tag_x: float
    tag_y: float


@dataclass(frozen=True)
class RelativeControlError:
    """Target pose error expressed in the current robot frame."""

    target_x: float
    target_y: float
    distance: float
    lateral: float
    yaw: float
    heading: float
    longitudinal: float


@dataclass(frozen=True)
class CorridorError:
    """Docking errors expressed in approach-axis coordinates.

    The axis runs from the docked pose into the tag. ``along`` and ``cross``
    are the goal's offset from the robot resolved on that axis, so both go to
    zero exactly at the docked pose, and ``cross`` is the true perpendicular
    deviation from the approach line rather than a bearing to the goal.
    """

    along: float
    cross: float
    yaw: float
    distance: float
    goal_x: float
    goal_y: float
    axis_yaw: float


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def target_pose_from_tag(
    tag_pose: Pose2D,
    distance: float,
    lateral_offset: float,
    yaw_offset: float,
) -> Pose2D:
    """Return a base pose on the configured approach line of a tag."""

    yaw = normalize_angle(tag_pose.yaw + yaw_offset)
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    left_x = -forward_y
    left_y = forward_x
    return Pose2D(
        x=(
            tag_pose.x
            - distance * forward_x
            + lateral_offset * left_x
        ),
        y=(
            tag_pose.y
            - distance * forward_y
            + lateral_offset * left_y
        ),
        yaw=yaw,
    )


def reverse_target_from_tag(
    robot_pose: Pose2D,
    tag_x_base: float,
    tag_y_base: float,
    tag_normal_yaw_base: float,
    final_distance: float,
    lateral_offset: float,
    yaw_offset: float,
) -> ReverseDockTarget:
    """Create an odometry-frame rear-first target from one tag observation.

    The base position is identical to forward docking. Its final heading is
    rotated pi radians, so the robot reaches that position by driving backward.
    """

    cosine = math.cos(robot_pose.yaw)
    sine = math.sin(robot_pose.yaw)
    tag_x = robot_pose.x + cosine * tag_x_base - sine * tag_y_base
    tag_y = robot_pose.y + sine * tag_x_base + cosine * tag_y_base

    forward_yaw = normalize_angle(
        robot_pose.yaw + tag_normal_yaw_base + yaw_offset
    )
    forward_x = math.cos(forward_yaw)
    forward_y = math.sin(forward_yaw)
    left_x = -forward_y
    left_y = forward_x

    base_pose = Pose2D(
        x=(
            tag_x
            - final_distance * forward_x
            + lateral_offset * left_x
        ),
        y=(
            tag_y
            - final_distance * forward_y
            + lateral_offset * left_y
        ),
        yaw=normalize_angle(forward_yaw + math.pi),
    )
    return ReverseDockTarget(base_pose=base_pose, tag_x=tag_x, tag_y=tag_y)


def relative_control_error(
    robot_pose: Pose2D,
    target_pose: Pose2D,
    reverse: bool,
) -> RelativeControlError:
    """Return non-holonomic pose errors for forward or reverse travel."""

    delta_x = target_pose.x - robot_pose.x
    delta_y = target_pose.y - robot_pose.y
    cosine = math.cos(robot_pose.yaw)
    sine = math.sin(robot_pose.yaw)
    target_x = cosine * delta_x + sine * delta_y
    target_y = -sine * delta_x + cosine * delta_y

    direction = -1.0 if reverse else 1.0
    return RelativeControlError(
        target_x=target_x,
        target_y=target_y,
        distance=math.hypot(target_x, target_y),
        lateral=target_y,
        yaw=normalize_angle(target_pose.yaw - robot_pose.yaw),
        heading=math.atan2(direction * target_y, direction * target_x),
        longitudinal=direction * target_x,
    )


def transform_pose(reference_pose: Pose2D, local_pose: Pose2D) -> Pose2D:
    """Express ``local_pose``, given in ``reference_pose``'s frame, in its parent."""

    cosine = math.cos(reference_pose.yaw)
    sine = math.sin(reference_pose.yaw)
    return Pose2D(
        x=reference_pose.x + cosine * local_pose.x - sine * local_pose.y,
        y=reference_pose.y + sine * local_pose.x + cosine * local_pose.y,
        yaw=normalize_angle(reference_pose.yaw + local_pose.yaw),
    )


def corridor_error(
    robot_pose: Pose2D,
    tag_pose: Pose2D,
    target_distance: float,
    lateral_offset: float,
    yaw_offset: float,
    reverse: bool = False,
) -> CorridorError:
    """Resolve the docking error onto the tag's approach axis.

    ``tag_pose.yaw`` is the tag's outward normal. ``yaw`` is measured against
    the heading the robot must hold to travel down the axis, which is the axis
    itself when driving nose-first and its opposite when backing in.
    """

    axis_yaw = normalize_angle(tag_pose.yaw + yaw_offset)
    forward_x = math.cos(axis_yaw)
    forward_y = math.sin(axis_yaw)
    left_x = -forward_y
    left_y = forward_x

    goal_x = tag_pose.x - target_distance * forward_x + lateral_offset * left_x
    goal_y = tag_pose.y - target_distance * forward_y + lateral_offset * left_y

    delta_x = goal_x - robot_pose.x
    delta_y = goal_y - robot_pose.y
    travel_yaw = normalize_angle(axis_yaw + math.pi) if reverse else axis_yaw

    return CorridorError(
        along=delta_x * forward_x + delta_y * forward_y,
        cross=delta_x * left_x + delta_y * left_y,
        yaw=normalize_angle(travel_yaw - robot_pose.yaw),
        distance=math.hypot(delta_x, delta_y),
        goal_x=goal_x,
        goal_y=goal_y,
        axis_yaw=axis_yaw,
    )


def segment_point_distance(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    point_x: float,
    point_y: float,
) -> float:
    """Return the shortest distance from a point to a finite segment."""

    segment_x = end_x - start_x
    segment_y = end_y - start_y
    length_squared = segment_x * segment_x + segment_y * segment_y
    if length_squared <= 1e-12:
        return math.hypot(point_x - start_x, point_y - start_y)

    projection = (
        (point_x - start_x) * segment_x + (point_y - start_y) * segment_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest_x = start_x + projection * segment_x
    closest_y = start_y + projection * segment_y
    return math.hypot(point_x - closest_x, point_y - closest_y)


def entry_waypoints(
    robot_pose: Pose2D,
    tag_pose: Pose2D,
    entry_distance: float,
    lateral_offset: float,
    yaw_offset: float,
    keepout_radius: float,
    arc_step_angle: float,
) -> tuple:
    """Plan the legs that place the robot on the approach axis.

    The last waypoint is always the corridor entry point. When the straight run
    to it would pass closer to the tag than ``keepout_radius``, the robot is
    first pushed radially clear and then walks an arc around the tag, so it
    never crosses the dock face on the way in.
    """

    axis_yaw = normalize_angle(tag_pose.yaw + yaw_offset)
    forward_x = math.cos(axis_yaw)
    forward_y = math.sin(axis_yaw)
    entry = Pose2D(
        x=(
            tag_pose.x
            - entry_distance * forward_x
            + lateral_offset * -forward_y
        ),
        y=(
            tag_pose.y
            - entry_distance * forward_y
            + lateral_offset * forward_x
        ),
        yaw=axis_yaw,
    )

    clearance = segment_point_distance(
        robot_pose.x,
        robot_pose.y,
        entry.x,
        entry.y,
        tag_pose.x,
        tag_pose.y,
    )
    if clearance >= keepout_radius:
        return (entry,)

    radius = max(
        entry_distance,
        math.hypot(robot_pose.x - tag_pose.x, robot_pose.y - tag_pose.y),
    )
    start_bearing = math.atan2(
        robot_pose.y - tag_pose.y,
        robot_pose.x - tag_pose.x,
    )
    end_bearing = math.atan2(entry.y - tag_pose.y, entry.x - tag_pose.x)
    sweep = normalize_angle(end_bearing - start_bearing)
    steps = max(1, int(math.ceil(abs(sweep) / max(arc_step_angle, 1e-3))))

    waypoints = []
    for index in range(steps):
        bearing = start_bearing + sweep * (index / steps)
        waypoints.append(
            Pose2D(
                x=tag_pose.x + radius * math.cos(bearing),
                y=tag_pose.y + radius * math.sin(bearing),
                yaw=normalize_angle(bearing + math.pi),
            )
        )
    waypoints.append(entry)
    return tuple(waypoints)


def scan_sector_clearances(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    forward_angle: float,
    front_half_angle: float,
    rear_half_angle: float,
):
    """Return minimum valid ranges in robot-front and robot-rear sectors."""

    front = math.inf
    rear = math.inf
    angle = angle_min
    for value in ranges:
        if math.isfinite(value) and range_min <= value <= range_max:
            front_error = normalize_angle(angle - forward_angle)
            rear_error = normalize_angle(angle - forward_angle - math.pi)
            if abs(front_error) <= front_half_angle:
                front = min(front, value)
            if abs(rear_error) <= rear_half_angle:
                rear = min(rear, value)
        angle += angle_increment
    return front, rear


def minimum_range_excluding_sector(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    forward_angle: float,
    exclude_bearing: float,
    exclude_half_angle: float,
):
    """Return the closest valid return, ignoring one robot-frame sector.

    Rotating in place sweeps the whole footprint, so the relevant clearance is
    the minimum over the whole scan. The dock the robot is deliberately parked in
    front of would dominate that minimum, so its bearing is excluded.
    """

    closest = math.inf
    angle = angle_min
    for value in ranges:
        if math.isfinite(value) and range_min <= value <= range_max:
            bearing = normalize_angle(angle - forward_angle)
            if (
                exclude_half_angle <= 0.0
                or abs(normalize_angle(bearing - exclude_bearing))
                > exclude_half_angle
            ):
                closest = min(closest, float(value))
        angle += angle_increment
    return closest
