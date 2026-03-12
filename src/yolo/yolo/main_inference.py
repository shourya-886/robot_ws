#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

from yolo_detector import YOLODetector
import os
#commit
#importing all the pkgs required
class YOLO(Node):
    def __init__(self):
        super().__init__("yolo_inference")

        #defining the args's descriptions
        conf_thresh_arg_desc = ParameterDescriptor(description="Confidence threshold (0.0 to 1.0)")
        model_path_arg_desc = ParameterDescriptor(description="Path to YOLO model in .ONNX format... also it should be relative to the yolo pkg folder")
        save_images_arg_desc = ParameterDescriptor(description="true or false... whether to save images in yolo_pkg_folder/received_images")

        #declaring args
        self.declare_parameter("model_path", "models/my_model_2.onnx", descriptor=model_path_arg_desc)
        self.declare_parameter("save_images", True, descriptor=save_images_arg_desc)
        self.declare_parameter("conf_thres", 0.5, descriptor=conf_thresh_arg_desc)

        #passing arg's values to self variables
        self.MODEL_PATH = self.get_parameter("model_path").get_parameter_value().string_value
        self.CONF_THRESH = self.get_parameter("conf_thres").get_parameter_value().double_value
        self.save_images = self.get_parameter("save_images").get_parameter_value().bool_value

        #checking if the user has requested to live stream as well
        if self.save_images:
            #defining base paths for saving images
            self.base_input_path = os.path.join("/home/shourya/robot_ws/src/yolo", "received_images", "input") #the folder where the images from topic: /camera/colour are stored and also acts as the input for inference
            self.results_path = os.path.join("/home/shourya/robot_ws/src/yolo", "received_images", "results")#the folder where the resultant images are stored
            #defining the final path
            self.final_input_path = os.path.join(self.base_input_path, "input.jpg")
            self.final_results_path = os.path.join(self.results_path, "results.jpg")

        #loading the model in RAM so that we don't have to initialize it again in image callback
        self.get_logger().info("Loading YOLO model into RAM...")
        self.detector = YOLODetector(self.MODEL_PATH, conf_thres=self.CONF_THRESH)

        #creating subs and pubs and bridges
        self.colour_sub = self.create_subscription(CompressedImage, "/camera/colour/compressed", self.image_callback, 10)
        self.cracks_found_pub = self.create_publisher(Bool, "/cracks_found", 10)
        self.bridge = CvBridge()

        #defining variables
        self.CLASSES = ["Fist bump", "Hello", "Thumbs down", "Thumbs up"]
        self.input_cv = None

        self.get_logger().info("waiting for image....")

    def image_callback(self, msg):
        #this func will be called when an image is received
        self.get_logger().info("received image")

        try:#writing in try block to catch exceptions
            input_cv = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')#converting the ros2 msg to cv2 img
            cv2.imwrite(self.final_input_path, input_cv)#writing it for debugging

            self.run_inference(input_cv)#calling helper function while passing the cv2 img

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    def run_inference(self, frame):
        detections = self.detector.detect(frame)#running inference

        detected_names = []#creating list for appending the detections
        for d in detections:#looping through each detections
            name = self.CLASSES[d['class_id']]
            detected_names.append(name)#appending them

        cracks = Bool() #initializing the msg to be sent in /cracks_found
        if len(detections) >= 1: #if there are more than or equal to 1 objects then print it in  console
            self.get_logger().info(f"-----------objects detected: {', '.join(detected_names)}------")
            cracks.data = True
        else:
            self.get_logger().info(f"Detected no objects")
            cracks.data = False

        self.cracks_found_pub.publish(cracks)#publishing the data

        for det in detections: #actual drawing boxes part
            x1, y1, x2, y2 = det["box"]
            class_name = self.CLASSES[det['class_id']]
            label = f"{class_name}: {det['conf']:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imwrite(self.final_results_path, frame)#writing the results image to path


def main():
    rclpy.init()
    inference = YOLO()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()