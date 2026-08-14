import threading

from my_robot_docking.docking_server import DockingServer


class Future:
    def __init__(self):
        self._callbacks = []
        self._done = False
        self._lock = threading.Lock()

    def add_done_callback(self, callback):
        with self._lock:
            if self._done:
                callback(self)
                return
            self._callbacks.append(callback)

    def done(self):
        with self._lock:
            return self._done

    def complete(self):
        with self._lock:
            self._done = True
            callbacks = tuple(self._callbacks)
        for callback in callbacks:
            callback(self)


class DockGoal:
    is_cancel_requested = False


class NavGoal:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def test_wait_for_future_wakes_on_done_callback(monkeypatch):
    server = DockingServer.__new__(DockingServer)
    server._active_nav_goal = None
    future = Future()
    timer = threading.Timer(0.01, future.complete)
    monkeypatch.setattr('my_robot_docking.docking_server.rclpy.ok', lambda: True)

    timer.start()
    try:
        assert server._wait_for_future(future, DockGoal(), timeout=1.0)
    finally:
        timer.cancel()


def test_wait_for_future_cancels_navigation_on_timeout(monkeypatch):
    server = DockingServer.__new__(DockingServer)
    server._active_nav_goal = NavGoal()
    future = Future()
    monkeypatch.setattr('my_robot_docking.docking_server.rclpy.ok', lambda: True)

    assert not server._wait_for_future(
        future,
        DockGoal(),
        timeout=0.01,
        cancel_nav=True,
    )
    assert server._active_nav_goal.cancel_count == 1
