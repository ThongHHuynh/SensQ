#!/usr/bin/env python3

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class JetsonCsiCamera(Node):
    def __init__(self) -> None:
        super().__init__("jetson_csi_camera")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("topic_name", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_link")

        sensor_id = int(self.get_parameter("sensor_id").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        topic_name = str(self.get_parameter("topic_name").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            topic_name,
            qos_profile_sensor_data,
        )

        self.pipeline = self.create_pipeline(
            sensor_id=sensor_id,
            width=width,
            height=height,
            fps=fps,
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
            f"Publishing {width}x{height} at up to {fps} FPS on {topic_name}"
        )

    @staticmethod
    def create_pipeline(
        sensor_id: int,
        width: int,
        height: int,
        fps: int,
    ) -> str:
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), "
            f"width={width}, height={height}, "
            f"framerate={fps}/1, format=NV12 ! "
            "nvvidconv ! "
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

        message = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

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