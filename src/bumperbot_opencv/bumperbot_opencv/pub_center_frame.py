#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PubCenterFrame(Node):
    def __init__(self):
        super().__init__('pub_center_frame')

        self.declare_parameter("cicle_radius", 10)
        self.declare_parameter("circle_color", (0, 0, 255))
        self.declare_parameter("circle_thickness", -1)

        self.radius = self.get_parameter("cicle_radius").get_parameter_value().integer_value
        self.color = self.get_parameter("circle_color").get_parameter_value().string_value
        self.thickness = self.get_parameter("circle_thickness").get_parameter_value().integer_value
        
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image,
                                                  '/image_raw',
                                                  self.procedure,
                                                  10)
        self.image_pub = self.create_publisher(Image,
                                               '/center_frame',
                                               10
                                               )
        self.center_pub = self.create_publisher(String,
                                                '/center_frame_coor',
                                                10
                                                )
        self.center = (0, 0)


    def procedure(self, msg):
        self.get_logger().info("Received image")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        

        height = cv_image.shape[0]
        width = cv_image.shape[1]

        self.center = (int(width / 2), int(height / 2))

        cv2.circle(cv_image, self.center, self.radius, self.color, self.thickness)

        self.publish_center_frame()
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

        self.get_logger().info("Published center frame and image")

    def publish_center_frame(self):
        center_frame_msg = String()
        center_frame_msg.data = f"{self.center[0]}, {self.center[1]}"
        self.center_pub.publish(center_frame_msg)

def main():
    rclpy.init()
    pub_center_frame = PubCenterFrame()
    rclpy.spin(pub_center_frame)


if __name__ == "__main__":
    main()