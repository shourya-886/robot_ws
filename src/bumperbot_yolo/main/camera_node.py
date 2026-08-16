#!/usr/bin/env python3
"""
Custom camera node -- captures from the webcam and publishes both a raw
(sensor_msgs/Image) and compressed (sensor_msgs/CompressedImage) stream
in one node, at a fixed 15Hz.

Replaces the earlier two-node design (webcam_compressed.py +
compressed_to_image.py), which is no longer used.
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('video_device', 0)
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('raw_topic', '/camera/color/image_raw')
        self.declare_parameter('compressed_topic', '/camera/color/compressed')

        video_device = self.get_parameter('video_device').value
        frame_rate = self.get_parameter('frame_rate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        self.frame_id = self.get_parameter('frame_id').value
        raw_topic = self.get_parameter('raw_topic').value
        compressed_topic = self.get_parameter('compressed_topic').value

        self.bridge = CvBridge()
        self.raw_pub = self.create_publisher(Image, raw_topic, 10)
        self.compressed_pub = self.create_publisher(CompressedImage, compressed_topic, 10)

        self.cap = cv2.VideoCapture(video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {video_device}!")
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.get_logger().info(
                f"Camera node started: device={video_device}, "
                f"{width}x{height} @ {frame_rate}Hz -> '{raw_topic}' and '{compressed_topic}'"
            )

        self._consecutive_failures = 0
        self._max_consecutive_failures = 10  # log an escalated warning if the camera stalls

        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self._consecutive_failures += 1
            if self._consecutive_failures == self._max_consecutive_failures:
                self.get_logger().error(
                    f"Camera has failed to deliver a frame for "
                    f"{self._consecutive_failures} consecutive attempts -- "
                    f"check the device connection."
                )
            else:
                self.get_logger().warn("Failed to capture frame from camera")
            return

        self._consecutive_failures = 0
        stamp = self.get_clock().now().to_msg()

        # Raw image
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        self.raw_pub.publish(raw_msg)

        # Compressed image, encoded from the same captured frame (not re-decoded
        # from the raw message, so this does one JPEG encode per frame, not two
        # conversions).
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        if success:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = self.frame_id
            compressed_msg.format = 'jpeg'
            compressed_msg.data = np.array(encoded_image).tobytes()
            self.compressed_pub.publish(compressed_msg)
        else:
            self.get_logger().error("Failed to encode image to JPEG")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()