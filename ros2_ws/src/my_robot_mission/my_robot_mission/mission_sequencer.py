"""Mission sequencer orchestrating undock, coverage cleaning, and re-docking.

Only ``mission_type: "coverage"`` is implemented. Nothing in the codebase
defines a waypoint source, loop count, or stopping condition for a "patrol"
mission, so patrol goals are rejected rather than guessing at motion
behaviour for a physical robot.
"""

import json
import math
import re
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from my_robot_docking.dock_database import DockDatabase, DockDatabaseError
from my_robot_docking.docking_geometry import Pose2D, target_pose_from_tag
from my_robot_docking_msgs.action import Dock, Mission, Undock

SEGMENT_PATTERN = re.compile(r'segment (\d+)/(\d+)')


class MissionOutcome:
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELLED = 'cancelled'


class MissionSequencer(Node):
    """Orchestrate undock -> coverage -> return -> dock as one action goal."""

    def __init__(self):
        super().__init__('mission_sequencer')
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        if not self.dock_database_file:
            raise RuntimeError('dock_database_file parameter is empty')
        self.database = DockDatabase(
            self.dock_database_file,
            self.legacy_predocking_offset,
        )

        self.callback_group = ReentrantCallbackGroup()

        self._goal_lock = threading.Lock()
        self._active_goal_handle = None
        self._preempt_requested = False
        self._goal_free_event = threading.Event()
        self._goal_free_event.set()

        self._execution_lock = threading.Lock()
        self._execution_state = {}
        self._planning_lock = threading.Lock()
        self._planning_metrics = {}

        self._undock_client = ActionClient(
            self, Undock, self.undock_action, callback_group=self.callback_group,
        )
        self._dock_client = ActionClient(
            self, Dock, self.dock_action, callback_group=self.callback_group,
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
            callback_group=self.callback_group,
        )
        self._coverage_generate_client = self.create_client(
            Trigger,
            self.coverage_generate_service,
            callback_group=self.callback_group,
        )
        self._coverage_start_client = self.create_client(
            Trigger,
            self.coverage_start_service,
            callback_group=self.callback_group,
        )
        self._coverage_cancel_client = self.create_client(
            Trigger,
            self.coverage_cancel_service,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            String,
            self.coverage_execution_status_topic,
            self._execution_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            self.coverage_planning_status_topic,
            self._planning_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self._events_publisher = self.create_publisher(String, self.events_topic, 10)

        self.action_server = ActionServer(
            self,
            Mission,
            self.mission_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            handle_accepted_callback=self._handle_accepted,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f'Mission sequencer ready: action={self.mission_action}, '
            f'docks={self.database.ids()}'
        )

    def _declare_parameters(self):
        declarations = {
            'mission_action': '/execute_mission',
            'undock_action': '/undock',
            'dock_action': '/dock',
            'navigate_to_pose_action': '/navigate_to_pose',
            'coverage_generate_service': '/coverage_planner/generate',
            'coverage_start_service': '/coverage_executor/start',
            'coverage_cancel_service': '/coverage_executor/cancel',
            'coverage_execution_status_topic': '/test_coverage/execution_status',
            'coverage_planning_status_topic': '/test_coverage/planning_status',
            # The planner does not report true covered area; this scales the
            # covered path length into a reporting estimate, not a planning
            # input.
            'coverage_operation_width_m': 0.25,
            'events_topic': '/mission/events',
            'dock_database_file': '',
            'legacy_predocking_offset': 0.5,
            'action_server_timeout_s': 10.0,
            'navigation_timeout_s': 120.0,
            'undock_timeout_s': 60.0,
            'dock_timeout_s': 180.0,
            'coverage_timeout_s': 1800.0,
            'coverage_poll_interval_s': 1.0,
        }
        for name, default in declarations.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        for name in (
            'mission_action',
            'undock_action',
            'dock_action',
            'navigate_to_pose_action',
            'coverage_generate_service',
            'coverage_start_service',
            'coverage_cancel_service',
            'coverage_execution_status_topic',
            'coverage_planning_status_topic',
            'coverage_operation_width_m',
            'events_topic',
            'dock_database_file',
            'legacy_predocking_offset',
            'action_server_timeout_s',
            'navigation_timeout_s',
            'undock_timeout_s',
            'dock_timeout_s',
            'coverage_timeout_s',
            'coverage_poll_interval_s',
        ):
            setattr(self, name, self.get_parameter(name).value)

    def _validate_parameters(self):
        positive = (
            'coverage_operation_width_m',
            'legacy_predocking_offset',
            'action_server_timeout_s',
            'navigation_timeout_s',
            'undock_timeout_s',
            'dock_timeout_s',
            'coverage_timeout_s',
            'coverage_poll_interval_s',
        )
        invalid = [name for name in positive if getattr(self, name) <= 0.0]
        if invalid:
            raise ValueError(
                f'Mission parameters must be positive: {", ".join(invalid)}'
            )

    # -- Action server callbacks -------------------------------------------------

    def _goal_callback(self, request):
        if request.mission_type != 'coverage':
            self.get_logger().warning(
                f"Rejecting mission: mission_type '{request.mission_type}' is "
                "not implemented (only 'coverage' is supported)"
            )
            return GoalResponse.REJECT
        try:
            self.database.get(request.dock_id)
        except DockDatabaseError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _handle_accepted(self, goal_handle):
        with self._goal_lock:
            previous = self._active_goal_handle
            self._active_goal_handle = goal_handle
        if previous is not None and previous.is_active:
            self.get_logger().info('New mission goal preempts the active one')
            self._preempt_requested = True
        goal_handle.execute()

    def _execute_callback(self, goal_handle):
        stop_wait = (
            self.coverage_timeout_s
            + self.navigation_timeout_s
            + self.undock_timeout_s
            + self.dock_timeout_s
        )
        if not self._goal_free_event.wait(timeout=stop_wait):
            result = Mission.Result()
            result.success = False
            result.message = 'Timed out waiting for the previous mission to stop'
            if goal_handle.is_active:
                goal_handle.abort()
            return result

        self._goal_free_event.clear()
        self._preempt_requested = False
        request = goal_handle.request
        result = Mission.Result()
        start_time = time.monotonic()

        try:
            dock = self.database.get(request.dock_id)
        except DockDatabaseError as error:
            return self._finish(
                goal_handle, result, MissionOutcome.FAILED, str(error), start_time,
            )

        try:
            self._publish_event('started', request.mission_type, dock.dock_id)

            outcome, message = self._run_undocking(goal_handle, dock)
            if outcome != MissionOutcome.SUCCEEDED:
                return self._finish(goal_handle, result, outcome, message, start_time)

            outcome, message = self._run_coverage(goal_handle, dock)
            if outcome != MissionOutcome.SUCCEEDED:
                return self._finish(goal_handle, result, outcome, message, start_time)

            outcome, message = self._run_return_to_dock(goal_handle, dock)
            if outcome != MissionOutcome.SUCCEEDED:
                return self._finish(goal_handle, result, outcome, message, start_time)

            if request.auto_dock_on_complete:
                outcome, message = self._run_docking(goal_handle, dock)
                if outcome != MissionOutcome.SUCCEEDED:
                    return self._finish(
                        goal_handle, result, outcome, message, start_time,
                    )

            self._publish_feedback(
                goal_handle, Mission.Feedback.COMPLETE, dock.dock_id, 100.0,
                'Mission complete',
            )
            result.success = True
            result.area_covered_m2 = self._estimate_area_covered()
            result.duration_seconds = time.monotonic() - start_time
            result.message = f'Mission complete for {dock.dock_id}'
            self._publish_event('completed', request.mission_type, dock.dock_id)
            goal_handle.succeed()
            return result
        except Exception as error:  # Keep the action from leaving a stuck state.
            self.get_logger().error(f'Mission failed: {error}')
            return self._finish(
                goal_handle, result, MissionOutcome.FAILED, str(error), start_time,
            )
        finally:
            self._goal_free_event.set()
            with self._goal_lock:
                if self._active_goal_handle is goal_handle:
                    self._active_goal_handle = None

    def _finish(self, goal_handle, result, outcome, message, start_time):
        result.success = False
        result.area_covered_m2 = self._estimate_area_covered()
        result.duration_seconds = time.monotonic() - start_time
        result.message = message
        if outcome == MissionOutcome.CANCELLED:
            self._publish_feedback(
                goal_handle, Mission.Feedback.PAUSED, '', 0.0, message,
            )
            if goal_handle.is_active:
                goal_handle.canceled()
            self._publish_event('cancelled', detail=message)
        else:
            self._publish_feedback(
                goal_handle, Mission.Feedback.ERROR, '', 0.0, message,
            )
            if goal_handle.is_active:
                goal_handle.abort()
            self._publish_event('failed', detail=message)
        return result

    # -- Mission phases -----------------------------------------------------------

    def _run_undocking(self, goal_handle, dock):
        self._publish_feedback(
            goal_handle, Mission.Feedback.UNDOCKING, dock.dock_id, 0.0,
            f'Undocking from {dock.dock_id}',
        )
        if not self._undock_client.wait_for_server(
            timeout_sec=self.action_server_timeout_s,
        ):
            return MissionOutcome.FAILED, f'{self.undock_action} is unavailable'

        goal = Undock.Goal()
        goal.dock_id = dock.dock_id
        send_future = self._undock_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, goal_handle, self.action_server_timeout_s):
            return self._cancel_or_fail(goal_handle, 'Undocking request timed out')

        undock_handle = send_future.result()
        if undock_handle is None or not undock_handle.accepted:
            return MissionOutcome.FAILED, 'Undock goal was rejected'

        result_future = undock_handle.get_result_async()
        if not self._wait_for_future(
            result_future, goal_handle, self.undock_timeout_s, cancel_handle=undock_handle,
        ):
            return self._cancel_or_fail(goal_handle, 'Undocking timed out')

        wrapped = result_future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and bool(wrapped.result.success):
            return MissionOutcome.SUCCEEDED, 'Undocked'
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return MissionOutcome.CANCELLED, 'Undocking cancelled'
        return MissionOutcome.FAILED, f'Undocking failed: {wrapped.result.message}'

    def _run_coverage(self, goal_handle, dock):
        self._publish_feedback(
            goal_handle, Mission.Feedback.NAVIGATING, dock.dock_id, 10.0,
            'Coverage executor will navigate to the entry point',
        )
        generated, message = self._call_trigger(
            self._coverage_generate_client, 'Coverage planner', self.action_server_timeout_s,
        )
        if not generated:
            return MissionOutcome.FAILED, f'Coverage planning failed: {message}'

        self._publish_feedback(
            goal_handle, Mission.Feedback.COVERING, dock.dock_id, 15.0,
            'Starting coverage execution',
        )
        started, message = self._call_trigger(
            self._coverage_start_client, 'Coverage executor', self.action_server_timeout_s,
        )
        if not started:
            return MissionOutcome.FAILED, f'Coverage execution failed to start: {message}'

        deadline = time.monotonic() + self.coverage_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested or self._preempt_requested:
                self._call_trigger(self._coverage_cancel_client, 'Coverage cancellation', 5.0)
                return MissionOutcome.CANCELLED, 'Mission cancelled during coverage'

            with self._execution_lock:
                state = self._execution_state.get('state')
                detail = self._execution_state.get('detail', '')

            self._publish_feedback(
                goal_handle, Mission.Feedback.COVERING, dock.dock_id,
                self._estimate_coverage_percent(detail), detail or 'Covering',
            )

            if state == 'SUCCEEDED':
                return MissionOutcome.SUCCEEDED, 'Coverage complete'
            if state == 'CANCELED':
                return MissionOutcome.CANCELLED, 'Coverage was cancelled externally'
            if state == 'ERROR':
                return MissionOutcome.FAILED, f'Coverage execution error: {detail}'

            time.sleep(self.coverage_poll_interval_s)

        self._call_trigger(self._coverage_cancel_client, 'Coverage cancellation', 5.0)
        return MissionOutcome.FAILED, 'Coverage execution timed out'

    def _run_return_to_dock(self, goal_handle, dock):
        self._publish_feedback(
            goal_handle, Mission.Feedback.RETURNING, dock.dock_id, 90.0,
            'Returning to dock staging pose',
        )
        if not self._navigate_client.wait_for_server(
            timeout_sec=self.action_server_timeout_s,
        ):
            return MissionOutcome.FAILED, f'{self.navigate_to_pose_action} is unavailable'

        staging_pose = target_pose_from_tag(
            Pose2D(dock.reference_x, dock.reference_y, dock.approach_yaw),
            dock.predocking_distance,
            dock.lateral_offset,
            dock.yaw_offset,
        )
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = dock.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = staging_pose.x
        goal.pose.pose.position.y = staging_pose.y
        goal.pose.pose.orientation.z = math.sin(staging_pose.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(staging_pose.yaw * 0.5)

        send_future = self._navigate_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, goal_handle, self.action_server_timeout_s):
            return self._cancel_or_fail(
                goal_handle, 'Navigation request to dock staging pose timed out',
            )

        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            return MissionOutcome.FAILED, 'Nav2 rejected the return-to-dock goal'

        result_future = nav_handle.get_result_async()
        if not self._wait_for_future(
            result_future, goal_handle, self.navigation_timeout_s, cancel_handle=nav_handle,
        ):
            return self._cancel_or_fail(
                goal_handle, 'Navigation to dock staging pose timed out',
            )

        wrapped = result_future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            return MissionOutcome.SUCCEEDED, 'Reached dock staging pose'
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return MissionOutcome.CANCELLED, 'Return-to-dock navigation cancelled'
        return (
            MissionOutcome.FAILED,
            f'Return-to-dock navigation failed with status {wrapped.status}',
        )

    def _run_docking(self, goal_handle, dock):
        self._publish_feedback(
            goal_handle, Mission.Feedback.DOCKING, dock.dock_id, 95.0, 'Docking',
        )
        if not self._dock_client.wait_for_server(timeout_sec=self.action_server_timeout_s):
            return MissionOutcome.FAILED, f'{self.dock_action} is unavailable'

        goal = Dock.Goal()
        goal.dock_id = dock.dock_id
        goal.navigate_to_staging_pose = False
        goal.use_offset_override = False
        goal.final_distance = 0.0
        goal.lateral_offset = 0.0
        goal.yaw_offset = 0.0

        send_future = self._dock_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, goal_handle, self.action_server_timeout_s):
            return self._cancel_or_fail(goal_handle, 'Docking request timed out')

        dock_handle = send_future.result()
        if dock_handle is None or not dock_handle.accepted:
            return MissionOutcome.FAILED, 'Docking goal was rejected'

        result_future = dock_handle.get_result_async()
        if not self._wait_for_future(
            result_future, goal_handle, self.dock_timeout_s, cancel_handle=dock_handle,
        ):
            return self._cancel_or_fail(goal_handle, 'Docking timed out')

        wrapped = result_future.result()
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED and bool(wrapped.result.success):
            return MissionOutcome.SUCCEEDED, 'Docked'
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return MissionOutcome.CANCELLED, 'Docking cancelled'
        return MissionOutcome.FAILED, f'Docking failed: {wrapped.result.message}'

    # -- Helpers --------------------------------------------------------------

    def _wait_for_future(self, future, goal_handle, timeout, cancel_handle=None):
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested or self._preempt_requested:
                if cancel_handle is not None:
                    cancel_handle.cancel_goal_async()
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if cancel_handle is not None:
                    cancel_handle.cancel_goal_async()
                return False
            completed.wait(timeout=min(remaining, 0.05))
        return future.done()

    def _cancel_or_fail(self, goal_handle, timeout_message):
        if goal_handle.is_cancel_requested or self._preempt_requested:
            return MissionOutcome.CANCELLED, 'Mission cancelled'
        return MissionOutcome.FAILED, timeout_message

    def _call_trigger(self, client, label, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'{label} service is unavailable'
        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=timeout):
            return False, f'{label} service call timed out'
        try:
            response = future.result()
        except Exception as error:
            return False, str(error)
        return bool(response.success), str(response.message)

    def _execution_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            payload = {}
        with self._execution_lock:
            self._execution_state = payload

    def _planning_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            payload = {}
        with self._planning_lock:
            self._planning_metrics = payload

    def _estimate_coverage_percent(self, detail):
        if not detail:
            return 0.0
        match = SEGMENT_PATTERN.search(detail)
        if not match:
            return 0.0
        current, total = int(match.group(1)), int(match.group(2))
        if total <= 0:
            return 0.0
        return round(100.0 * min(current, total) / total, 1)

    def _estimate_area_covered(self):
        with self._planning_lock:
            metrics = dict(self._planning_metrics)
        path_length = metrics.get('path_length_m')
        if not isinstance(path_length, (int, float)):
            return 0.0
        return round(float(path_length) * self.coverage_operation_width_m, 2)

    def _publish_feedback(self, goal_handle, phase, current_zone, progress_percent, detail):
        feedback = Mission.Feedback()
        feedback.phase = int(phase)
        feedback.current_zone = str(current_zone)
        feedback.progress_percent = float(progress_percent)
        feedback.detail = str(detail)
        goal_handle.publish_feedback(feedback)

    def _publish_event(self, kind, mission_type='', dock_id='', detail=''):
        payload = {
            'event': kind,
            'mission_type': mission_type,
            'dock_id': dock_id,
            'detail': detail,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
        }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self._events_publisher.publish(message)

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
