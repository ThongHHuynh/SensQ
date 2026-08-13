from types import SimpleNamespace

from my_robot_docking.docking_server import ApproachOutcome, DockingServer
from my_robot_docking_msgs.action import Dock


class Logger:
    def info(self, _message):
        pass


def make_server(outcomes):
    server = DockingServer.__new__(DockingServer)
    calls = []
    outcome_iterator = iter(outcomes)

    def approach(*args):
        calls.append({
            'distance': args[2],
            'moving_state': args[6],
            'verifying_state': args[7],
        })
        return next(outcome_iterator)

    server._approach_forward_target = approach
    server.get_logger = lambda: Logger()
    return server, calls


def test_predocking_is_verified_before_staging():
    server, calls = make_server([
        (ApproachOutcome.SUCCEEDED, 'predocking ready'),
        (ApproachOutcome.SUCCEEDED, 'staging ready'),
    ])
    dock = SimpleNamespace(
        dock_id='test_dock',
        predocking_distance=1.5,
        staging_distance=1.0,
    )

    outcome, _ = server._prepare_at_staging(
        object(),
        dock,
        lateral_offset=0.0,
        yaw_offset=0.0,
        retries=0,
    )

    assert outcome == ApproachOutcome.SUCCEEDED
    assert [call['distance'] for call in calls] == [1.5, 1.0]
    assert calls[0]['moving_state'] == Dock.Feedback.ALIGNING_PREDOCK
    assert calls[1]['moving_state'] == Dock.Feedback.MOVING_TO_STAGING
    assert calls[1]['verifying_state'] == Dock.Feedback.VERIFYING_STAGING


def test_failed_predocking_does_not_advance_to_staging():
    server, calls = make_server([
        (ApproachOutcome.CONTROL_FAILED, 'predocking failed'),
    ])
    dock = SimpleNamespace(
        dock_id='test_dock',
        predocking_distance=1.5,
        staging_distance=1.0,
    )

    outcome, message = server._prepare_at_staging(
        object(),
        dock,
        lateral_offset=0.0,
        yaw_offset=0.0,
        retries=0,
    )

    assert outcome == ApproachOutcome.CONTROL_FAILED
    assert message == 'predocking failed'
    assert [call['distance'] for call in calls] == [1.5]
