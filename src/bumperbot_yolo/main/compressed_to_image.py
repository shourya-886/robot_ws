#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class CompressedToImageNode(Node):
    def __init__(self):
        super().__init__('compressed_to_image_node')
        
        # Initialize CvBridge for converting between OpenCV and ROS image types
        self.bridge = CvBridge()
        
        # Subscriber for incoming compressed images
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/colour/compressed',
            self.listener_callback,
            10
        )
        
        # Publisher for raw decompressed images
        self.publisher_ = self.create_publisher(
            Image,
            '/camera/colour/image_raw',
            10
        )
        
        self.get_logger().info("Compressed to Image Node has started. Listening to /camera/colour/compressed and publishing to /camera/colour/image_raw")

    def listener_callback(self, msg: CompressedImage):
        try:
            # Convert compressed image data (bytes) back into an OpenCV image matrix
            # cv2.imdecode handles formats like JPEG and PNG automatically based on the raw bytes
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image payload.")
                return

            # Convert the OpenCV image into a standard ROS 2 sensor_msgs/Image message
            raw_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            
            # Preserve the original header timestamp and frame ID
            raw_msg.header = msg.header
            
            # Publish the raw image
            self.publisher_.publish(raw_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    # Import numpy locally inside main or at top level (needed for frombuffer)
    global np
    import numpy as np
    
    node = CompressedToImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()