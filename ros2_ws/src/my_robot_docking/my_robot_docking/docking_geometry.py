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


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


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
