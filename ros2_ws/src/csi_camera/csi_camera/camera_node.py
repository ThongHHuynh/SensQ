#!/usr/bin/env python3

"""Publish synchronized CSI camera images and calibration metadata."""

from copy import deepcopy
from pathlib import Path
from urllib.parse import unquote, urlparse

import cv2
import rclpy
import yaml

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def _matrix_data(calibration, key, expected_length):
    """Return a validated row-major matrix from a ROS calibration YAML."""
    matrix = calibration.get(key, {})
    values = matrix.get("data") if isinstance(matrix, dict) else None

    if not isinstance(values, list) or len(values) != expected_length:
        raise ValueError(
            f"Calibration field '{key}.data' must contain "
            f"{expected_length} values."
        )

    return [float(value) for value in values]


def _calibration_path(camera_info_url):
    """Convert a plain path or file URL into a local calibration path."""
    parsed = urlparse(camera_info_url)

    if parsed.scheme not in ("", "file"):
        raise ValueError(
            "camera_info_url must be a local path or a file:// URL."
        )

    if parsed.scheme == "file" and parsed.netloc not in ("", "localhost"):
        raise ValueError("Remote file:// calibration URLs are not supported.")

    raw_path = parsed.path if parsed.scheme == "file" else camera_info_url
    return Path(unquote(raw_path)).expanduser()


def load_camera_info(camera_info_url, expected_width, expected_height):
    """Load and validate a standard ROS monocular calibration YAML file."""
    path = _calibration_path(camera_info_url)

    try:
        with path.open("r", encoding="utf-8") as calibration_file:
            calibration = yaml.safe_load(calibration_file)
    except OSError as error:
        raise ValueError(
            f"Could not read camera calibration '{path}': {error}"
        ) from error

    if not isinstance(calibration, dict):
        raise ValueError(f"Camera calibration '{path}' is not a YAML map.")

    width = int(calibration.get("image_width", 0))
    height = int(calibration.get("image_height", 0))

    if (width, height) != (expected_width, expected_height):
        raise ValueError(
            f"Calibration resolution {width}x{height} does not match "
            f"camera output {expected_width}x{expected_height}."
        )

    distortion = calibration.get("distortion_coefficients", {})
    distortion_values = (
        distortion.get("data") if isinstance(distortion, dict) else None
    )
    if not isinstance(distortion_values, list):
        raise ValueError(
            "Calibration field 'distortion_coefficients.data' must be a list."
        )

    camera_info = CameraInfo()
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = str(
        calibration.get("distortion_model", "plumb_bob")
    )
    camera_info.d = [float(value) for value in distortion_values]
    camera_info.k = _matrix_data(calibration, "camera_matrix", 9)
    camera_info.r = _matrix_data(calibration, "rectification_matrix", 9)
    camera_info.p = _matrix_data(calibration, "projection_matrix", 12)

    if camera_info.k[0] == 0.0 or camera_info.p[0] == 0.0:
        raise ValueError("Camera calibration has zero focal length.")

    return camera_info


def uncalibrated_camera_info(width, height):
    """Create ROS-standard metadata whose zero K matrix means uncalibrated."""
    camera_info = CameraInfo()
    camera_info.width = width
    camera_info.height = height
    camera_info.distortion_model = "plumb_bob"
    return camera_info


class JetsonCsiCamera(Node):
    def __init__(self) -> None:
        super().__init__("jetson_csi_camera")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("topic_name", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("flip_180", True)


        sensor_id = int(self.get_parameter("sensor_id").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        topic_name = str(self.get_parameter("topic_name").value)
        camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        camera_info_url = str(self.get_parameter("camera_info_url").value)
        flip_180 = bool(self.get_parameter("flip_180").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.expected_width = width
        self.expected_height = height
        self.bridge = CvBridge()


        if camera_info_url:
            self.camera_info = load_camera_info(
                camera_info_url,
                expected_width=width,
                expected_height=height,
            )
            self.get_logger().info(
                f"Loaded {width}x{height} calibration from {camera_info_url}"
            )
        else:
            self.camera_info = uncalibrated_camera_info(width, height)
            self.get_logger().warning(
                "No camera_info_url provided; publishing uncalibrated "
                "CameraInfo. AprilTag pose estimation requires calibration."
            )

        self.publisher = self.create_publisher(
            Image,
            topic_name,
            qos_profile_sensor_data,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            camera_info_topic,
            qos_profile_sensor_data,
        )

        self.pipeline = self.create_pipeline(
            sensor_id=sensor_id,
            width=width,
            height=height,
            fps=fps,
            flip_180=flip_180
        )

        self.capture = cv2.VideoCapture(
            self.pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.capture.isOpened():
            raise RuntimeError(
                "Camera is not online, could not open the CSI camera with the GStreamer pipeline."
            )

        # The camera blocks inside cap.read(), so this timer runs as fast as
        # frames become available.
        self.timer = self.create_timer(0.001, self.publish_frame)

        self.get_logger().info(
            f"Publishing synchronized {width}x{height} images on "
            f"{topic_name} and metadata on {camera_info_topic} at up to "
            f"{fps} FPS"
        )

    @staticmethod
    def create_pipeline(
        sensor_id: int,
        width: int,
        height: int,
        fps: int,
        flip_180: bool,
    ) -> str:
        flip_method = 2 if flip_180 else 0
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), "
            f"width={width}, height={height}, "
            f"framerate={fps}/1, format=NV12 ! "
            f"nvvidconv flip-method={flip_method} ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true sync=false"
        )

    def publish_frame(self) -> None:
        success, frame = self.capture.read()

        if not success:
            self.get_logger().error(
                "Failed to capture a frame.",
                throttle_duration_sec=2.0,
            )
            return

        frame_height, frame_width = frame.shape[:2]
        if (frame_width, frame_height) != (
            self.expected_width,
            self.expected_height,
        ):
            self.get_logger().error(
                f"Captured {frame_width}x{frame_height}, expected "
                f"{self.expected_width}x{self.expected_height}; dropping "
                "frame because its calibration would be invalid.",
                throttle_duration_sec=2.0,
            )
            return

        message = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        camera_info = deepcopy(self.camera_info)
        camera_info.header = message.header

        self.camera_info_publisher.publish(camera_info)
        self.publisher.publish(message)

    def destroy_node(self) -> None:
        if hasattr(self, "capture") and self.capture.isOpened():
            self.capture.release()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = JetsonCsiCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"Camera startup failed: {error}")
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
