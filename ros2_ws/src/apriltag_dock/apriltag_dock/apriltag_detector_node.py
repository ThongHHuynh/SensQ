"""
AprilTag Detector Node
======================
Detects AprilTag 36h11 markers from an RGB camera and broadcasts their
6-DOF poses as TF frames.  Also publishes a PoseArray of all detections.

This node supports two backends:
  1. ``apriltag_ros`` (system package) — preferred when installed.
     In this mode the node simply wraps the apriltag_ros AprilTagNode
     and republishes detected tag poses + TF.
  2. ``pupil-apriltag`` (pip package) — pure-Python fallback.

Dependencies (at least one):
  - ros-humble-apriltag-ros   (apt)
  - pupil-apriltag + scipy    (pip)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

# Try pupil-apriltag (pip fallback)
try:
    import pupil_apriltag as apriltag
except ImportError:
    apriltag = None

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    Rotation = None


class AprilTagDetectorNode(Node):
    """Detect AprilTags and broadcast their poses as TF frames.

    Uses the pupil-apriltag Python library for detection and pose
    estimation.  Camera intrinsics are obtained from the CameraInfo
    topic.
    """

    def __init__(self):
        super().__init__('apriltag_detector_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.20)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = self.get_parameter('tag_size').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        # ── Validate imports ────────────────────────────────────────────
        if apriltag is None:
            self.get_logger().fatal(
                'pupil-apriltag not installed. '
                'Run:  pip3 install pupil-apriltag'
            )
            raise RuntimeError('pupil-apriltag is required')
        if Rotation is None:
            self.get_logger().fatal(
                'scipy not installed. '
                'Run:  pip3 install scipy'
            )
            raise RuntimeError('scipy is required')

        # ── AprilTag detector ──────────────────────────────────────────
        self.detector = apriltag.Detector(families=self.tag_family)
        self.get_logger().info(
            f'AprilTag detector initialised  family={self.tag_family}  '
            f'tag_size={self.tag_size:.3f} m'
        )

        # ── Camera intrinsics (populated by CameraInfo callback) ──────
        self.camera_params = None  # [fx, fy, cx, cy]

        # ── ROS helpers ─────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── QoS for image/camera_info (sensor‑data profile) ────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ──────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_cb,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            sensor_qos,
        )

        # ── Publisher ───────────────────────────────────────────────────
        self.detections_pub = self.create_publisher(
            PoseArray, '/apriltag/detections', 10
        )

        self.get_logger().info('AprilTag detector node started')

    # ─────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        """Cache camera intrinsics from the first CameraInfo message."""
        if self.camera_params is not None:
            return
        # K is a 3×3 row-major array
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]
        if fx == 0.0 or fy == 0.0:
            return  # invalid / uninitialised
        self.camera_params = [fx, fy, cx, cy]
        self.get_logger().info(
            f'Camera intrinsics received: '
            f'fx={fx:.1f}  fy={fy:.1f}  cx={cx:.1f}  cy={cy:.1f}'
        )

    def _image_cb(self, msg: Image):
        """Detect tags in every incoming image frame."""
        if self.camera_params is None:
            return  # waiting for CameraInfo

        # Convert ROS Image → OpenCV greyscale
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        # Run the detector
        detections = self.detector.detect(
            cv_image,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        if not detections:
            return

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id or 'camera_optical_frame'

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id

        for det in detections:
            if det.pose_R is None or det.pose_t is None:
                continue

            # Translation (3×1 → flat)
            tx, ty, tz = det.pose_t.flatten()

            # Rotation matrix → quaternion
            rot = Rotation.from_matrix(det.pose_R)
            qx, qy, qz, qw = rot.as_quat()  # [x, y, z, w]

            # Build Pose
            pose = Pose()
            pose.position.x = float(tx)
            pose.position.y = float(ty)
            pose.position.z = float(tz)
            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)
            pose_array.poses.append(pose)

            # Broadcast TF  camera_optical_frame → tag_<id>
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = frame_id
            t.child_frame_id = f'tag_{det.tag_id}'
            t.transform.translation.x = float(tx)
            t.transform.translation.y = float(ty)
            t.transform.translation.z = float(tz)
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(t)

        self.detections_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
