import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

from yolo_detector import YOLODetector
import os
from ament_index_python.packages import get_package_share_directory



class YOLO(Node):
    def __init__(self):
        super().__init__("yolo_inference")

        self.declare_parameter("model_path", "models/my_model_2.onnx")
        self.declare_parameter("save_images", True)
        self.declare_parameter("camera_index", 2)
        self.declare_parameter("conf_thres", 0.4)

        self.CAMERA_INDEX = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.MODEL_PATH = self.get_parameter("model_path").get_parameter_value().string_value
        self.CONF_THRESH = self.get_parameter("conf_thres").get_parameter_value().float_value
        self.save_images = self.get_parameter("save_images").get_parameter_value().string_value

        if self.save_images:
            self.base_input_path = os.path.join(get_package_share_directory("yolo"), "received_images", "input") #the folder where the images from topic: /camera/colour are stored and also acts as the input for inference
            self.results_path = os.path.join(get_package_share_directory("yolo"), "received_images", "results")#the folder where the resultant images are stored

            self.final_input_path = os.path.join(self.base_input_path, "input.jpg")
            self.final_results_path = os.path.join(self.results_path, "results.jpg")

        self.get_logger().info("Loading YOLO model into RAM...")
        self.detector = YOLODetector(self.MODEL_PATH, conf_thres=self.CONF_THRESH)

        self.colour_sub = self.create_subscription(CompressedImage, "/camera/colour", self.image_callback, 10)
        self.cracks_found_pub = self.create_publisher(Bool, "/cracks_found", 10)
        self.bridge = CvBridge()

        self.CLASSES = ["Fist bump", "Hello", "Thumbs down", "Thumbs up"]
        self.input_cv = None

    def image_callback(self, msg):
        try:
            input_cv = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(self.final_input_path, input_cv)

            self.run_inference(input_cv)
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")


    def run_inference(self, frame):
        detections = self.detector.detect(frame)

        self.get_logger().info(f"detections = {detections}")
        cracks = Bool()

        if len(detections) >= 1:
            self.get_logger().info(f"Detected {len(detections)} cracks")
            cracks.data = True
        else:
            self.get_logger().info(f"Detected no cracks")
            cracks.data = False

        self.cracks_found_pub.publish(cracks)
        self.get_logger().info(f"drawing boxes")

        for det in detections:
            x1, y1, x2, y2 = det["box"]
            label = f"{self.CLASSES[det['class_id']]}: {det['conf']:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imwrite(self.final_results_path, frame)


def main():
    rclpy.init()
    inference = YOLO()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()