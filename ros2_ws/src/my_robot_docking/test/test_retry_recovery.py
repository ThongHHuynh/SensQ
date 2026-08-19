import math

import pytest

from my_robot_docking.docking_geometry import Pose2D
from my_robot_docking.docking_server import ApproachOutcome, DockingServer


class GoalHandle:
    is_cancel_requested = False


class Logger:
    def info(self, _message):
        pass


def make_server(poses):
    server = DockingServer.__new__(DockingServer)
    server.control_rate = 20.0
    server.retry_recovery_timeout = 10.0
    server.retry_backup_distance = 0.20
    server.retry_backup_speed = 0.05
    server.retry_rotation_angle = 0.35
    server.retry_rotation_speed = 0.20
    server.rotation_stop_distance = 0.30
    server.rear_stop_distance = 0.18
    server.yaw_tolerance = 0.08
    server.heading_kp = 1.20
    server._rear_clearance = math.inf
    server._rotation_clearance = lambda exclude_bearing=None: math.inf
    server._sensor_safety_reason = lambda require_scan=True: None
    server._publish_feedback = lambda *args, **kwargs: None
    server._stop_robot = lambda: None
    server.get_logger = lambda: Logger()
    pose_iterator = iter(poses)
    server._latest_odom_pose = lambda: next(pose_iterator)
    return server


def test_retry_recovery_backs_up_then_rotates(monkeypatch):
    poses = [
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(-0.10, 0.0, 0.0),
        Pose2D(-0.21, 0.0, 0.0),
        Pose2D(-0.21, 0.0, 0.0),
        Pose2D(-0.21, 0.0, 0.20),
        Pose2D(-0.21, 0.0, 0.35),
    ]
    server = make_server(poses)
    commands = []
    server._publish_dock_command = commands.append
    monkeypatch.setattr('my_robot_docking.docking_server.time.sleep', lambda _: None)
    monkeypatch.setattr('my_robot_docking.docking_server.time.monotonic', lambda: 0.0)
    monkeypatch.setattr('my_robot_docking.docking_server.rclpy.ok', lambda: True)

    outcome, message = server._recover_for_retry(GoalHandle(), 1)

    assert outcome == ApproachOutcome.SUCCEEDED
    assert message == 'Retry recovery complete'
    assert any(command.linear.x < 0.0 for command in commands)
    assert any(command.angular.z > 0.0 for command in commands)


def test_retry_rotation_direction_alternates():
    server = DockingServer.__new__(DockingServer)
    server.retry_rotation_angle = 0.35

    assert server._retry_rotation_offset(1) == pytest.approx(0.35)
    assert server._retry_rotation_offset(2) == pytest.approx(-0.35)
    assert server._retry_rotation_offset(3) == pytest.approx(0.35)


def test_retry_recovery_stops_for_rear_obstacle(monkeypatch):
    server = make_server([Pose2D(0.0, 0.0, 0.0)])
    server._rear_clearance = 0.10
    commands = []
    server._publish_dock_command = commands.append
    monkeypatch.setattr('my_robot_docking.docking_server.time.monotonic', lambda: 0.0)
    monkeypatch.setattr('my_robot_docking.docking_server.rclpy.ok', lambda: True)

    outcome, message = server._recover_for_retry(GoalHandle(), 1)

    assert outcome == ApproachOutcome.SAFETY_STOP
    assert 'Rear obstacle' in message
    assert commands == []


def test_rotation_safety_checks_swept_clearance():
    server = DockingServer.__new__(DockingServer)
    server.rotation_stop_distance = 0.30
    server._rotation_clearance = lambda exclude_bearing=None: 0.20
    server._rear_clearance = math.inf
    server.rear_stop_distance = 0.18
    server._sensor_safety_reason = lambda require_scan=True: None

    reason = server._retry_recovery_safety_reason(rotating=True)

    assert reason is not None
    assert 'rotation stop distance' in reason
