import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from coverage_planner_archive.coverage_executor_node import (
    densify_path,
    path_length,
    requires_path_entry,
    trim_path,
)


def make_path(points):
    path = Path()
    path.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def test_path_length_sums_segments():
    path = make_path([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)])

    assert path_length(path) == 7.0


def test_trim_path_interpolates_start_and_keeps_remainder():
    path = make_path([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])

    trimmed = trim_path(path, 1.0)

    assert len(trimmed.poses) == 3
    assert trimmed.poses[0].pose.position.x == 1.0
    assert trimmed.poses[0].pose.position.y == 0.0
    assert trimmed.poses[-1].pose.position.x == 2.0
    assert trimmed.poses[-1].pose.position.y == 2.0
    assert math.isclose(path_length(trimmed), 3.0)
    assert all(pose.header.frame_id == "map" for pose in trimmed.poses)


def test_trim_path_past_end_returns_final_pose():
    path = make_path([(0.0, 0.0), (1.0, 0.0)])

    trimmed = trim_path(path, 2.0)

    assert len(trimmed.poses) == 1
    assert trimmed.poses[0].pose.position.x == 1.0


def test_densify_path_limits_controller_pose_spacing():
    path = make_path([(0.0, 0.0), (1.0, 0.0)])

    dense = densify_path(path, 0.25)

    assert len(dense.poses) == 5
    assert math.isclose(path_length(dense), 1.0)
    for start, end in zip(dense.poses, dense.poses[1:]):
        distance = math.hypot(
            end.pose.position.x - start.pose.position.x,
            end.pose.position.y - start.pose.position.y,
        )
        assert distance <= 0.25


def test_densify_path_rejects_non_positive_spacing():
    path = make_path([(0.0, 0.0), (1.0, 0.0)])

    try:
        densify_path(path, 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("Expected ValueError for zero spacing")


def test_path_entry_is_required_only_outside_tolerance():
    target = PoseStamped()
    target.pose.position.x = 1.0
    target.pose.position.y = 1.0

    assert not requires_path_entry((1.1, 1.0), target, 0.25)
    assert requires_path_entry((2.0, 1.0), target, 0.25)
    assert requires_path_entry(None, target, 0.25)
