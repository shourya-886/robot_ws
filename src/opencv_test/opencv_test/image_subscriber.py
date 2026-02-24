#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # 1. Subscribe to the same topic used in image_publisher_node.py
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.listener_callback,
            10)
        
        # 2. Initialize CvBridge
        self.bridge = CvBridge()
        
        # 3. Create a directory to save images if it doesn't exist
        self.save_path = '/opencv_comparison/saved_images'
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            
        self.img_count = 0
        self.get_logger().info('Image Saver Node started. Waiting for images...')

    def listener_callback(self, msg):
        try:
            # 4. Convert ROS2 Image message back to OpenCV (NumPy) format
            # We use "bgr8" because that's what the publisher used
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 5. Save the image to disk
            filename = os.path.join(self.save_path, f'frame_{self.img_count:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            
            self.get_logger().info(f'Saved: {filename}')
            self.img_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Could not convert or save image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    
    try:
        rclpy.spin(image_saver)
    except KeyboardInterrupt:
        pass
    finally:
        image_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()