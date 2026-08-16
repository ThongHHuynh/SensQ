from types import SimpleNamespace

from my_robot_docking.docking_server import DockingServer


class NavigationGoal:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def make_server(acceptance_distance):
    server = DockingServer.__new__(DockingServer)
    server._active_dock_goal = SimpleNamespace(is_active=True)
    server._active_nav_goal = NavigationGoal()
    server.predocking_acceptance_distance = acceptance_distance
    server._navigation_feedback_valid = False
    server._predocking_acceptance_reached = False
    server._last_navigation_feedback_time = 0.0
    server.navigation_feedback_rate = 5.0
    server._navigation_feedback_state = 7
    server._publish_feedback = lambda *args, **kwargs: None
    return server


def feedback(distance):
    return SimpleNamespace(
        feedback=SimpleNamespace(distance_remaining=distance),
    )


def test_positive_acceptance_distance_cancels_after_crossing_threshold():
    server = make_server(0.25)

    server._navigation_feedback(feedback(0.50))
    server._navigation_feedback(feedback(0.24))

    assert server._predocking_acceptance_reached is True
    assert server._active_nav_goal.cancel_count == 1


def test_zero_acceptance_distance_disables_early_handoff():
    server = make_server(0.0)

    server._navigation_feedback(feedback(0.50))
    server._navigation_feedback(feedback(0.0))

    assert server._predocking_acceptance_reached is False
    assert server._active_nav_goal.cancel_count == 0


def test_initial_zero_feedback_does_not_trigger_handoff():
    server = make_server(0.25)

    server._navigation_feedback(feedback(0.0))

    assert server._predocking_acceptance_reached is False
    assert server._active_nav_goal.cancel_count == 0
