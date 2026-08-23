from geometry_msgs.msg import PoseStamped

from test_coverage.executor_node import (
    bounded_closest_pose_index,
    rewind_pose_index,
)


def _path(count: int):
    poses = []
    for index in range(count):
        pose = PoseStamped()
        pose.pose.position.x = float(index)
        poses.append(pose)
    return poses


def test_closest_pose_search_is_bounded_by_forward_path_distance():
    poses = _path(7)

    assert bounded_closest_pose_index(poses, 0, 6, 4.1, 0.0, 2.0) == 2
    assert bounded_closest_pose_index(poses, 2, 6, 4.1, 0.0, 2.0) == 4


def test_retry_rewinds_only_within_current_segment():
    poses = _path(7)

    assert rewind_pose_index(poses, 5, 2, 2.0) == 3
    assert rewind_pose_index(poses, 3, 3, 2.0) == 3
