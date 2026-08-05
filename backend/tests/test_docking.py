import math
import sys
import types
import unittest

from app.docking import DockingCatalog, DockingConfigError


database_stub = types.ModuleType("app.database")


async def save_snapshot_stub(_snapshot):
    return None


database_stub.save_snapshot = save_snapshot_stub
sys.modules.setdefault("app.database", database_stub)

websocket_stub = types.ModuleType("app.websocket_manager")
websocket_stub.ws_manager = object()
sys.modules.setdefault("app.websocket_manager", websocket_stub)

from app.ros_monitor import RosMonitor
from app.state import initial_snapshot


class DockingCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = DockingCatalog()

    def test_catalog_contains_current_docks_and_runtime_config(self):
        payload = self.catalog.get_payload()
        dock_ids = {dock["dockId"] for dock in payload["docks"]}

        self.assertEqual(dock_ids, {"home_dock", "flex_dock1"})
        self.assertEqual(payload["serverConfig"]["dock_action"], "/dock")
        self.assertEqual(
            payload["arbiterConfig"]["dock_input_topic"],
            "/cmd_vel_dock",
        )

    def test_goal_defaults_to_database_offsets(self):
        goal = self.catalog.normalize_goal({"dock_id": "home_dock"})

        self.assertTrue(goal["navigate_to_staging_pose"])
        self.assertFalse(goal["use_offset_override"])
        self.assertEqual(goal["final_distance"], 0.3)
        self.assertEqual(goal["lateral_offset"], 0.0)
        self.assertEqual(goal["yaw_offset"], 0.0)

    def test_override_respects_minimum_distance(self):
        with self.assertRaisesRegex(DockingConfigError, "at least"):
            self.catalog.normalize_goal(
                {
                    "dock_id": "home_dock",
                    "use_offset_override": True,
                    "final_distance": 0.05,
                }
            )

    def test_non_finite_override_is_rejected(self):
        with self.assertRaisesRegex(DockingConfigError, "finite"):
            self.catalog.normalize_goal(
                {
                    "dock_id": "flex_dock1",
                    "use_offset_override": True,
                    "yaw_offset": math.inf,
                }
            )

    def test_snapshot_contains_idle_docking_state(self):
        docking = initial_snapshot()["docking"]

        self.assertFalse(docking["active"])
        self.assertEqual(docking["state"], "IDLE")

    def test_ros_client_maps_every_action_goal_field(self):
        published = []
        monitor = RosMonitor(published.append, "/dock")
        client = FakeActionClient()
        monitor._ros_available = True
        monitor._dock_client = client
        monitor._dock_type = FakeDock
        goal = {
            "dock_id": "flex_dock1",
            "navigate_to_staging_pose": False,
            "use_offset_override": True,
            "final_distance": 0.4,
            "lateral_offset": -0.1,
            "yaw_offset": 0.2,
        }

        started, _ = monitor.start_docking(goal)

        self.assertTrue(started)
        self.assertEqual(client.goal.dock_id, "flex_dock1")
        self.assertFalse(client.goal.navigate_to_staging_pose)
        self.assertTrue(client.goal.use_offset_override)
        self.assertEqual(client.goal.final_distance, 0.4)
        self.assertEqual(client.goal.lateral_offset, -0.1)
        self.assertEqual(client.goal.yaw_offset, 0.2)
        self.assertEqual(published[-1]["docking"]["state"], "SENDING")


class FakeGoal:
    pass


class FakeDock:
    Goal = FakeGoal


class FakeFuture:
    def add_done_callback(self, callback):
        self.callback = callback


class FakeActionClient:
    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        self.goal = goal
        self.feedback_callback = feedback_callback
        return FakeFuture()


if __name__ == "__main__":
    unittest.main()
