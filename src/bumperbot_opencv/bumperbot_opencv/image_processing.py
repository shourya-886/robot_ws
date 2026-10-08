#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Subscribe to the input image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publishers for processed images
        self.contours_img_pub = self.create_publisher(Image, 'contours/image_raw', 1)
        self.gray_img_pub = self.create_publisher(Image, 'gray/image_raw', 1)
        self.blur_img_pub = self.create_publisher(Image, 'blur/image_raw', 1)
        self.thresholded_img_pub = self.create_publisher(Image, 'thresholded/image_raw', 1)
        self.perspective_img_pub = self.create_publisher(Image, 'prespective/image_raw', 1)
        self.corner_img_pub = self.create_publisher(Image, 'corner/image_raw', 1)
        self.keypoints_img_pub = self.create_publisher(Image, 'keypoints/image_raw', 1)

        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        try:
            cv_ptr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f"cv_bridge exception: {e}")
            return

        processed_image = cv_ptr.copy()
        contours_image = cv_ptr.copy()

        # 0. Convert to grayscale
        gray_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)

        # 1. Blurring
        blur_image = cv2.blur(processed_image, (15, 15))

        # 2. Thresholding
        _, thresholded_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)

        # 3. Contour Detection
        contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(contours_image, contours, -1, (0, 255, 0), 2)

        # 4. Feature Detection and Description (ORB)
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(gray_image, None)
        keypoints_img = cv2.drawKeypoints(
            processed_image, keypoints, None, color=(0, 0, 255), flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS
        )

        # 5. Harris Corner Detection
        blockSize = 2
        apertureSize = 3
        k = 0.04
        dst = cv2.cornerHarris(gray_image, blockSize, apertureSize, k)
        dst_norm = cv2.normalize(dst, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_32FC1 if hasattr(cv2, 'CV_32FC1') else cv2.CV_32FC1)
        dst_norm_scaled = cv2.convertScaleAbs(dst_norm)
        
        corner_image = processed_image.copy()
        for i in range(dst_norm.shape[0]):
            for j in range(dst_norm.shape[1]):
                if int(dst_norm[i, j]) > 200:
                    cv2.circle(corner_image, (j, i), 5, (0, 0, 255), 2, 8, 0)

        # Perspective placeholder (matching empty matrix behavior in C++)
        perspective_img = np.zeros_like(processed_image)

        # Publish all processed topics
        self.contours_img_pub.publish(self.bridge.cv2_to_imgmsg(contours_image, encoding='bgr8'))
        self.gray_img_pub.publish(self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8'))
        self.blur_img_pub.publish(self.bridge.cv2_to_imgmsg(blur_image, encoding='bgr8'))
        self.thresholded_img_pub.publish(self.bridge.cv2_to_imgmsg(thresholded_image, encoding='mono8'))
        self.corner_img_pub.publish(self.bridge.cv2_to_imgmsg(corner_image, encoding='bgr8'))
        self.perspective_img_pub.publish(self.bridge.cv2_to_imgmsg(perspective_img, encoding='bgr8'))
        self.keypoints_img_pub.publish(self.bridge.cv2_to_imgmsg(keypoints_img, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()