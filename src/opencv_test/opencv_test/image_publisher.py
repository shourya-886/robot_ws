#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge

class ImageCommander(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter("image_path", "/home/shourya/robot_ws/src/opencv_test/picture_to_send/1.jpg")
        self.declare_parameter("live_stream", True)
        self.declare_parameter("camera_index", 1)

        self.live_stream = self.get_parameter("live_stream").get_parameter_value().bool_value
        self.camera_index = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.image_path = self.get_parameter("image_path").get_parameter_value().string_value
        self.img = None

        if self.live_stream == True:
            self.source = "camera" #camera
            self.cam = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        else:
            self.source = self.image_path
            self.img = cv2.imread(self.image_path)
            if self.img is None:
                self.get_logger().error(f"Could not find image at {self.image_path}")
                return

        self.publisher_ = self.create_publisher(Image, '/camera/colour', 10)
        self.cmp_pub = self.create_publisher(CompressedImage, '/camera/colour/compressed', 10)
        self.cap_img_sub = self.create_subscription(Bool, "/capture_image", self.timer_callback, 10)
        
        self.bridge = CvBridge()
            
    def timer_callback(self, msg):
        if msg.data:
            if self.live_stream == True:
                ret, self.img = self.cam.read()
                if not ret:
                    self.get_logger().info("No image can be captured from camera")
                    return

            msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
            cmp_msg = self.bridge.cv2_to_compressed_imgmsg(self.img)

            self.get_logger().info(f"Sending {self.source}...")
            self.publisher_.publish(msg)
            self.cmp_pub.publish(cmp_msg)
            self.get_logger().info("Done!")

    def destroy_node(self):
        if self.live_stream and hasattr(self, 'cam'):
            self.cam.release()
            self.get_logger().info("Camera hardware released.")
        super().destroy_node()



def main():
    rclpy.init()
    node = ImageCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()