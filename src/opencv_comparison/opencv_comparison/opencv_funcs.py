#!/usr/bin/env python3
import cv2
import os
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from robot_msgs.srv import ImageToImage


class OpenCvFuncs(Node):
    def __init__(self):
        super().__init__("opencv_func_node")
        #this param asks for whether to save the images requested
        self.declare_parameter("save_requested_images", True)

        #assigning value from param
        self.save_requested_images = self.get_parameter("save_requested_images").get_parameter_value().bool_value

        if self.save_requested_images == True: #checking if the user has asked to save the requested files
            #run this if user's choice is yes
            self.declare_parameter("save_img_path", "/home/shourya/robot_ws/src/opencv_comparison/received_images") #declaring parameter if the user has asked to save the images
            self.save_path = self.get_parameter("save_img_path").get_parameter_value().string_value

        #creating server and initializing variables
        self.convert_colour_to_greyscale_server = self.create_service(ImageToImage, "/convert_colour_to_greyscale", self.convert_colour_to_greyscale_callback)
        self.bridge = CvBridge()
        self.img_count = 0
        self.filename_colour = ""
        self.filename_grey = ""
        self.get_logger().info("opencv_func_server is ready")
       
    def convert_colour_to_greyscale_callback(self, request, response):
        received_image = None
        self.img_count += 1

        try:
            received_image = self.bridge.imgmsg_to_cv2(request, "bgr8")
        except Exception as e:
            self.get_logger().error(f"error while converting colour to greyscale: {e}")

        if received_image is None:
            print("Error: Could not decode image. Check if format is supported.")
            return response

        grey_image = cv2.cvtColor(received_image, cv2.COLOR_BGR2GRAY)
        self.filename_colour = os.path.join(self.save_path, "colour_images", f'colour_frame_{self.img_count}.jpg')
        self.filename_grey = os.path.join(self.save_path, "grey_images", f'grey_frame_{self.img_count}.jpg')

        if self.save_requested_images:
            cv2.imwrite(self.filename_colour, received_image)
            cv2.imwrite(self.filename_grey, grey_image)

        response = self.bridge.cv2_to_imgmsg(grey_image, "rgb8")
        return response


def main():
    rclpy.init()
    node = OpenCvFuncs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()