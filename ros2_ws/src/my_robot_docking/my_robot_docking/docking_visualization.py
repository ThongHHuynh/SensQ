"""RViz markers for planned and executed docking stages."""

from dataclasses import dataclass
import math
import threading
import time

from geometry_msgs.msg import Point
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_docking.docking_geometry import (
    Pose2D,
    normalize_angle,
    target_pose_from_tag,
)


class DockingStage:
    """Stable marker namespaces for the docking state machine."""

    NAVIGATION = 'navigation'
    PREDOCK_ALIGNMENT = 'predock_alignment'
    STAGING_APPROACH = 'staging_approach'
    FINAL_APPROACH = 'final_approach'
    REVERSE_FINAL = 'reverse_final'
    RETRY = 'retry'


STAGE_COLORS = {
    DockingStage.NAVIGATION: (1.0, 0.82, 0.0, 0.95),
    DockingStage.PREDOCK_ALIGNMENT: (0.0, 0.90, 0.95, 0.95),
    DockingStage.STAGING_APPROACH: (0.10, 0.35, 1.0, 0.95),
    DockingStage.FINAL_APPROACH: (1.0, 0.42, 0.0, 0.95),
    DockingStage.REVERSE_FINAL: (0.90, 0.0, 0.90, 0.95),
    DockingStage.RETRY: (0.55, 0.55, 0.55, 0.95),
}

TAG_COLOR = (0.95, 0.05, 0.05, 0.95)
SUCCESS_COLOR = (0.10, 0.95, 0.20, 0.95)
FAILURE_COLOR = (0.95, 0.10, 0.10, 0.95)


@dataclass(frozen=True)
class DockingReference:
    """Global tag, predocking, staging, and final target poses."""

    frame_id: str
    tag: Pose2D
    predocking: Pose2D
    staging: Pose2D
    final: Pose2D
    reverse: bool


def build_docking_reference(
    dock,
    final_distance: float,
    lateral_offset: float,
    yaw_offset: float,
) -> DockingReference:
    """Build the static centerline targets shown in RViz."""

    tag = Pose2D(
        dock.reference_x,
        dock.reference_y,
        dock.approach_yaw,
    )
    predocking = target_pose_from_tag(
        tag,
        dock.predocking_distance,
        lateral_offset,
        yaw_offset,
    )
    staging = target_pose_from_tag(
        tag,
        dock.staging_distance,
        lateral_offset,
        yaw_offset,
    )
    final = target_pose_from_tag(
        tag,
        final_distance,
        lateral_offset,
        yaw_offset,
    )
    if dock.reverse_docking:
        final = Pose2D(
            final.x,
            final.y,
            normalize_angle(final.yaw + math.pi),
        )
    return DockingReference(
        frame_id=dock.global_frame,
        tag=tag,
        predocking=predocking,
        staging=staging,
        final=final,
        reverse=dock.reverse_docking,
    )


def should_append_trail_point(
    previous: Pose2D,
    current: Pose2D,
    minimum_distance: float,
    minimum_yaw: float,
) -> bool:
    """Return whether a sampled pose adds useful path information."""

    if previous is None:
        return True
    distance = math.hypot(current.x - previous.x, current.y - previous.y)
    yaw_change = abs(normalize_angle(current.yaw - previous.yaw))
    return distance >= minimum_distance or yaw_change >= minimum_yaw


class DockingVisualizer:
    """Publish persistent reference markers and stage-colored robot trails."""

    def __init__(
        self,
        node,
        enabled: bool,
        topic: str,
        line_width: float,
        minimum_point_distance: float,
        publish_rate: float,
        maximum_points: int,
    ):
        self._node = node
        self._enabled = bool(enabled)
        self._line_width = float(line_width)
        self._minimum_point_distance = float(minimum_point_distance)
        self._minimum_yaw = math.radians(2.0)
        self._publish_period = 1.0 / float(publish_rate)
        self._maximum_points = int(maximum_points)
        self._lock = threading.RLock()
        self._reference = None
        self._stage = None
        self._trails = {}
        self._trail_frames = {}
        self._current_pose = None
        self._live_target = None
        self._status = None
        self._last_publish_time = 0.0

        self._publisher = None
        if self._enabled:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._publisher = node.create_publisher(
                MarkerArray,
                topic,
                qos,
            )

    def begin_goal(
        self,
        dock,
        final_distance: float,
        lateral_offset: float,
        yaw_offset: float,
    ):
        """Clear the last run and publish this goal's reference geometry."""

        if not self._enabled:
            return
        reference = build_docking_reference(
            dock,
            final_distance,
            lateral_offset,
            yaw_offset,
        )
        with self._lock:
            self._reference = reference
            self._stage = None
            self._trails = {}
            self._trail_frames = {}
            self._current_pose = None
            self._live_target = None
            self._status = None
            self._last_publish_time = 0.0
            delete = Marker()
            delete.action = Marker.DELETEALL
            self._publisher.publish(MarkerArray(markers=[delete]))
            self._publish_locked(force=True)

    def set_stage(self, stage):
        """Select the color and namespace used for new odometry samples."""

        if not self._enabled:
            return
        if stage is not None and stage not in STAGE_COLORS:
            raise ValueError(f'Unknown docking visualization stage: {stage}')
        with self._lock:
            self._stage = stage
            self._live_target = None
            self._publish_locked(force=True)

    def append_pose(self, pose: Pose2D, frame_id: str):
        """Append one odometry pose to the active stage's actual trail."""

        if not self._enabled or self._stage is None or not frame_id:
            return
        with self._lock:
            stage = self._stage
            trail = self._trails.setdefault(stage, [])
            trail_frame = self._trail_frames.get(stage)
            if trail_frame is not None and trail_frame != frame_id:
                trail.clear()
            self._trail_frames[stage] = frame_id
            if should_append_trail_point(
                trail[-1] if trail else None,
                pose,
                self._minimum_point_distance,
                self._minimum_yaw,
            ):
                trail.append(pose)
                if len(trail) > self._maximum_points:
                    del trail[:len(trail) - self._maximum_points]
            self._current_pose = (frame_id, pose)
            self._publish_locked()

    def update_live_target(
        self,
        target: Pose2D,
        frame_id: str,
        robot: Pose2D = None,
    ):
        """Show the target currently used by the visual controller."""

        if not self._enabled or not frame_id:
            return
        if robot is None:
            robot = Pose2D(0.0, 0.0, 0.0)
        with self._lock:
            self._live_target = (frame_id, robot, target)
            self._publish_locked()

    def finish(self, success: bool, message: str):
        """Stop recording and leave a persistent result label in RViz."""

        if not self._enabled:
            return
        with self._lock:
            self._stage = None
            self._live_target = None
            self._status = (bool(success), str(message))
            self._publish_locked(force=True)

    def _publish_locked(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_publish_time < self._publish_period:
            return
        self._last_publish_time = now
        self._publisher.publish(MarkerArray(markers=self._build_markers()))

    def _build_markers(self):
        stamp = self._node.get_clock().now().to_msg()
        markers = []
        if self._reference is not None:
            markers.extend(self._reference_markers(stamp))
        markers.extend(self._trail_markers(stamp))
        markers.extend(self._dynamic_markers(stamp))
        return markers

    def _reference_markers(self, stamp):
        reference = self._reference
        final_stage = (
            DockingStage.REVERSE_FINAL
            if reference.reverse
            else DockingStage.FINAL_APPROACH
        )
        markers = [
            self._line_marker(
                'reference/staging',
                reference.frame_id,
                [reference.predocking, reference.staging],
                STAGE_COLORS[DockingStage.STAGING_APPROACH],
                stamp,
            ),
            self._line_marker(
                'reference/final',
                reference.frame_id,
                [reference.staging, reference.final],
                STAGE_COLORS[final_stage],
                stamp,
            ),
            self._sphere_marker(
                'reference/tag',
                0,
                reference.frame_id,
                reference.tag,
                TAG_COLOR,
                stamp,
            ),
        ]
        targets = (
            ('Predock', reference.predocking,
             STAGE_COLORS[DockingStage.PREDOCK_ALIGNMENT]),
            ('Staging', reference.staging,
             STAGE_COLORS[DockingStage.STAGING_APPROACH]),
            ('Final', reference.final, STAGE_COLORS[final_stage]),
        )
        for marker_id, (label, pose, color) in enumerate(targets, start=1):
            markers.append(self._arrow_marker(
                'reference/poses',
                marker_id,
                reference.frame_id,
                pose,
                color,
                stamp,
            ))
            markers.append(self._text_marker(
                'reference/labels',
                marker_id,
                reference.frame_id,
                pose,
                label,
                color,
                stamp,
            ))
        return markers

    def _trail_markers(self, stamp):
        markers = []
        for marker_id, stage in enumerate(STAGE_COLORS):
            trail = self._trails.get(stage, ())
            if not trail:
                continue
            markers.append(self._line_marker(
                f'actual/{stage}',
                self._trail_frames[stage],
                trail,
                STAGE_COLORS[stage],
                stamp,
                marker_id,
            ))
        return markers

    def _dynamic_markers(self, stamp):
        markers = []
        if self._current_pose is not None:
            frame_id, pose = self._current_pose
            color = FAILURE_COLOR
            if self._status is not None and self._status[0]:
                color = SUCCESS_COLOR
            elif self._stage in STAGE_COLORS:
                color = STAGE_COLORS[self._stage]
            markers.append(self._arrow_marker(
                'current', 0, frame_id, pose, color, stamp,
            ))
        else:
            markers.append(self._delete_marker('current', 0))

        if self._live_target is not None:
            frame_id, robot, target = self._live_target
            color = STAGE_COLORS.get(
                self._stage,
                STAGE_COLORS[DockingStage.FINAL_APPROACH],
            )
            markers.append(self._arrow_marker(
                'live', 0, frame_id, target, color, stamp,
            ))
            markers.append(self._line_marker(
                'live',
                frame_id,
                [robot, target],
                color,
                stamp,
                marker_id=1,
                width=self._line_width * 0.5,
            ))
        else:
            markers.append(self._delete_marker('live', 0))
            markers.append(self._delete_marker('live', 1))

        if self._status is not None and self._reference is not None:
            success, message = self._status
            markers.append(self._text_marker(
                'status',
                0,
                self._reference.frame_id,
                self._reference.tag,
                message,
                SUCCESS_COLOR if success else FAILURE_COLOR,
                stamp,
                z=0.65,
            ))
        else:
            markers.append(self._delete_marker('status', 0))
        return markers

    def _marker(self, namespace, marker_id, marker_type, frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _line_marker(
        self,
        namespace,
        frame_id,
        poses,
        color,
        stamp,
        marker_id=0,
        width=None,
    ):
        marker = self._marker(
            namespace,
            marker_id,
            Marker.LINE_STRIP,
            frame_id,
            stamp,
        )
        marker.scale.x = self._line_width if width is None else width
        self._set_color(marker, color)
        marker.points = [self._point(pose.x, pose.y, 0.035) for pose in poses]
        return marker

    def _arrow_marker(
        self,
        namespace,
        marker_id,
        frame_id,
        pose,
        color,
        stamp,
    ):
        marker = self._marker(
            namespace,
            marker_id,
            Marker.ARROW,
            frame_id,
            stamp,
        )
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = 0.055
        marker.pose.orientation.z = math.sin(pose.yaw * 0.5)
        marker.pose.orientation.w = math.cos(pose.yaw * 0.5)
        marker.scale.x = 0.24
        marker.scale.y = 0.055
        marker.scale.z = 0.055
        self._set_color(marker, color)
        return marker

    def _sphere_marker(
        self,
        namespace,
        marker_id,
        frame_id,
        pose,
        color,
        stamp,
    ):
        marker = self._marker(
            namespace,
            marker_id,
            Marker.SPHERE,
            frame_id,
            stamp,
        )
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = 0.06
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        self._set_color(marker, color)
        return marker

    def _text_marker(
        self,
        namespace,
        marker_id,
        frame_id,
        pose,
        text,
        color,
        stamp,
        z=0.24,
    ):
        marker = self._marker(
            namespace,
            marker_id,
            Marker.TEXT_VIEW_FACING,
            frame_id,
            stamp,
        )
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = z
        marker.scale.z = 0.13
        marker.text = text
        self._set_color(marker, color)
        return marker

    @staticmethod
    def _delete_marker(namespace, marker_id):
        marker = Marker()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def _set_color(marker, color):
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
