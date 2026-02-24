#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageCommander(Node):
    def __init__(self):
        super().__init__('image_commander')

        self.declare_parameter("image_path", "/home/shourya/robot_ws/src/opencv_comparison/picture_to_send/1.jpg")
        self.image_path = self.get_parameter("image_path").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, '/camera_image', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.bridge = CvBridge()
            
    def timer_callback(self):
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error(f"Could not find image at {self.image_path}")
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        self.get_logger().info(f"Sending {self.image_path}...")
        self.publisher_.publish(msg)
        self.get_logger().info("Done!")

def main():
    rclpy.init()
    node = ImageCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()