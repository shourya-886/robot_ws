#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from datetime import datetime

class Camera(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("camera_usb", 0)
        self.declare_parameter("hardware", "pc")

        self.camera_usb_raw = self.get_parameter("camera_usb").get_parameter_value().integer_value
        self.hardware = self.get_parameter("hardware").get_parameter_value().string_value

        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_usb_raw)

        self.get_logger().info("Camera Node started")
    def timer_callback(self):
        if not self.cap.isOpened():
            print(f"Error: Could not open USB camera at index {self.camera_usb_raw}")
            return

        ret, frame = self.cap.read()
        if ret:
            self.get_logger().info(f"captured frame")
            self.captured_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_publisher.publish(self.captured_image)
        else:
            self.get_logger().error(f"could not capture frame from usb webcamera")
            return


def main():
    rclpy.init()
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()