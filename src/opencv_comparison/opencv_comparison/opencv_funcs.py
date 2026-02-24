#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

import cv2
import os


class OpenCvFuncs(Node):
    def __init__(self):
        super().__init__("opencv_func_node")

        self.declare_parameter("save_converted_images", True)
        self.declare_parameter("camera_usb", 0)

        self.save_converted_images = self.get_parameter("save_converted_images").get_parameter_value().bool_value
        self.save_path = "/home/shourya/robot_ws/src/opencv_comparison/received_images"
        self.camera_usb_raw = self.get_parameter("camera_usb").get_parameter_value().integer_value

        self.image_capture_sub = self.create_subscription(Bool, "/capture_frame", self.image_capture_callback, 10)
        self.greyscale_image_pub = self.create_publisher(Image, "/camera/grey_image", 10)

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.camera_usb_raw, cv2.CAP_V4L2)

        os.makedirs(os.path.join(self.save_path, "grey_images"), exist_ok=True)

        self.frame = None
        self.grey_image = None
        self.img_count = 0
        self.get_logger().info("opencv_comparison is ready")

    def capture_and_save_image(self):
        if not self.cap.isOpened():
            self.get_logger().error(f"Camera index {self.camera_usb_raw} not open")
            return False

        ret, frame = self.cap.read()
        if ret:
            self.img_count += 1
            self.frame = frame
            self.get_logger().info(f"Captured frame #{self.img_count}")

            filename_colour = os.path.join(self.save_path, "colour_images", f"captured_image{self.img_count}.png")
            cv2.imwrite(filename_colour, self.frame)
            return True
        else:
            self.get_logger().error("Could not capture frame")
            return False

    def convert_to_greyscale(self):
        # Always convert from the raw NumPy 'self.frame'
        self.grey_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        if self.save_converted_images:
            filename_grey = os.path.join(self.save_path, "grey_images", f'grey_frame_{self.img_count}.png')
            cv2.imwrite(filename_grey, self.grey_image)

    def image_capture_callback(self, msg):
        if msg.data:
            if self.capture_and_save_image():
                self.convert_to_greyscale()

                message_to_send = self.bridge.cv2_to_imgmsg(self.grey_image, "mono8")
                self.greyscale_image_pub.publish(message_to_send)


def main():
    rclpy.init()
    node = OpenCvFuncs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()