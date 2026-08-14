"""AprilTag TF observation helper for the docking server."""

import math
import threading
from dataclasses import dataclass

from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException


@dataclass(frozen=True)
class TagObservation:
    """Fresh tag pose and surface-normal heading in the robot base frame."""

    tag_id: int
    tag_frame: str
    stamp_seconds: float
    age_seconds: float
    x: float
    y: float
    z: float
    normal_yaw: float


class TagTracker:
    """Combine AprilTag detections with their timestamp-matched TF poses."""

    def __init__(
        self,
        node,
        tf_buffer: Buffer,
        base_frame: str,
        detections_topic: str,
        tag_timeout: float,
        transform_timeout: float,
        normal_sign: float,
        callback_group=None,
    ):
        self._node = node
        self._tf_buffer = tf_buffer
        self._base_frame = base_frame
        self._tag_timeout = tag_timeout
        self._transform_timeout = transform_timeout
        self._normal_sign = 1.0 if normal_sign >= 0.0 else -1.0
        self._detections = {}
        self._lock = threading.Lock()

        self._subscription = node.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self._detections_callback,
            10,
            callback_group=callback_group,
        )

    def _detections_callback(self, message):
        with self._lock:
            for detection in message.detections:
                self._detections[detection.id] = message.header.stamp

    def latest(self, tag_id: int, tag_frame: str):
        """Return a fresh observation, or ``None`` when unavailable/stale."""

        with self._lock:
            detection = self._detections.get(tag_id)

        if detection is None:
            return None

        stamp = Time.from_msg(detection)
        now = self._node.get_clock().now()
        age = (now - stamp).nanoseconds / 1e9

        if age < -self._transform_timeout or age > self._tag_timeout:
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                tag_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Rotate the detector's tag +Z axis into base_frame. normal_sign selects
        # the physical approach direction because detector/model conventions
        # can choose opposite sides of the tag plane.
        normal_x = 2.0 * (
            rotation.x * rotation.z + rotation.w * rotation.y
        )
        normal_y = 2.0 * (
            rotation.y * rotation.z - rotation.w * rotation.x
        )
        normal_x *= self._normal_sign
        normal_y *= self._normal_sign

        horizontal_norm = math.hypot(normal_x, normal_y)
        if horizontal_norm < 0.25:
            self._node.get_logger().warning(
                f'Tag {tag_id} normal is not sufficiently horizontal',
                throttle_duration_sec=2.0,
            )
            return None

        normal_yaw = math.atan2(normal_y, normal_x)
        return TagObservation(
            tag_id=tag_id,
            tag_frame=tag_frame,
            stamp_seconds=stamp.nanoseconds / 1e9,
            age_seconds=max(0.0, age),
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            normal_yaw=normal_yaw,
        )
