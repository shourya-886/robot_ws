from platform import node

import rclpy
from rclpy.node import Node
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory


class ConvertColourToGreyConversion(Node):
    def __init__(self):
        super().__init__("colour_to_grey")

        # 1. Parameter and Path Setup (Do this once at startup)
        self.declare_parameter("save_images", True)
        self.save_images = self.get_parameter("save_images").value

        package_share = get_package_share_directory("opencv_comparison")
        self.base_colour_path = os.path.join(package_share, "saved_images", "colour")
        self.base_grey_path = os.path.join(package_share, "saved_images", "grey")

        if self.save_images:
            os.makedirs(self.base_colour_path, exist_ok=True)
            os.makedirs(self.base_grey_path, exist_ok=True)

        self.bridge = CvBridge()
        self.img_count = 0

        self.colour_sub = self.create_subscription(CompressedImage, "/camera/colour", self.colour_conversion_callback, 10)
        self.grey_pub = self.create_publisher(CompressedImage, "/camera/grey", 10)

    def colour_conversion_callback(self, msg):
        # 3. Fast Decoding
        input_cv = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        grey_img_cv = cv2.cvtColor(input_cv, cv2.COLOR_BGR2GRAY)

        output_msg = self.bridge.cv2_to_compressed_imgmsg(grey_img_cv, dst_format='jpg')
        output_msg.header.stamp = msg.header.stamp
        self.grey_pub.publish(output_msg)

        if self.save_images:
            c_path = os.path.join(self.base_colour_path, f"frame_{self.img_count}.jpg")
            g_path = os.path.join(self.base_grey_path, f"frame_{self.img_count}.jpg")

            cv2.imwrite(c_path, input_cv)
            cv2.imwrite(g_path, grey_img_cv)

            self.img_count += 1

def main(args=None):
    rclpy.init(args=args)
    conversion = ConvertColourToGreyConversion()
    rclpy.spin(conversion)
    conversion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()