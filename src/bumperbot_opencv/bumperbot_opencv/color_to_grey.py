#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2
from sensor_msgs.msg import Image

class ColorToGrey(Node):
    def __init__(self):
        super().__init__('pub_center_frame')

        self.bridge = cv_bridge.CvBridge()
        self.color_image_sub = self.create_subscription(Image,
                                                         '/image_raw',
                                                         self.color_to_grey_callback,
                                                         10)
        self.grey_image_pub = self.create_publisher(Image,
                                                    '/image_grey',
                                                    10
                                                    )

    def color_to_grey_callback(self, msg):
        self.get_logger().info("Received color image")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.grey_image_pub.publish(self.bridge.cv2_to_imgmsg(grey_image, encoding='mono8'))
        self.get_logger().info("Published grey image")

def main():
    rclpy.init()
    color_to_grey = ColorToGrey()
    rclpy.spin(color_to_grey)


if __name__ == "__main__":
    main()