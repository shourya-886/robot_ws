#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraCompressNode(Node):
    def __init__(self):
        super().__init__('camera_compress_node')
        
        # Publisher for compressed images
        self.publisher_ = self.create_publisher(
            CompressedImage, 
            '/camera/colour/compressed', 
            10
        )
    
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device!")
        else:
            self.get_logger().info("Camera Compressed Node has started, publishing to /camera/colour/compressed")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from camera")
            return

        # Create the CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        
        # Format specifier: 'jpeg' (or 'png') followed by optional parameters
        msg.format = 'jpeg'
        
        # Compress the OpenCV image matrix into JPEG format with 80% quality
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        
        if success:
            msg.data = np.array(encoded_image).tobytes()
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Failed to encode image to JPEG")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()