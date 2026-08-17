<<<<<<< HEAD
import os
import sys
import argparse
import time

# Image Processing
import cv2

# Serial
import serial

# YOLO pkgs
from ultralytics import YOLO

# Firebase
import firebase_admin
from firebase_admin import db, credentials

# Cloudinary
import cloudinary
import cloudinary.uploader

# Logging
=======
#!/usr/bin/env python3
"""
main_ros.py -- YOLO Node (node 4 of the bumperbot pipeline).

Does NOT open the camera or move the robot. WP NAV2 (waypoint_follower +
PhotoAtWaypoint) and the custom camera_node already handle navigation and
image capture independently. This node's only job: watch images_taken/ for
new files saved by PhotoAtWaypoint, run YOLO inference on each new image as
soon as it appears, and upload results to Cloudinary/Firebase.

Runs continuously. Each waypoint's photo is processed exactly once, the
moment it's detected -- not on a fixed timer, and not re-processed if seen
again.
"""
import os
import sys
import time

import cv2
from ultralytics import YOLO

import firebase_admin
from firebase_admin import db, credentials

import cloudinary
import cloudinary.uploader

>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)
from datetime import datetime

import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


# CONSTANTS
OUTPUT_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/clicked_images_inference"
CLICKED_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/clicked_images"
CLOUD_NAME = os.getenv("CLOUDINARY_CLOUD_NAME")
API_KEY = os.getenv("CLOUDINARY_API_KEY")
API_SECRET = os.getenv("CLOUDINARY_API_SECRET")
PORT = '/dev/arduino'
BAUD_RATE = 115200
TIME_TO_MOVE_FORWARD = 0.25
TIME_TO_MOVE_FORWARD_TURN = 3.5
TIME_TO_MOVE_FORWARD_TURN_FORWARD = 1.6
SLEEP_TIME = 2.0
=======


# CONSTANTS
IMAGES_TAKEN_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/images_taken"
OUTPUT_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/clicked_images_inference"
CLOUD_NAME = os.getenv("CLOUDINARY_CLOUD_NAME")
API_KEY = os.getenv("CLOUDINARY_API_KEY")
API_SECRET = os.getenv("CLOUDINARY_API_SECRET")
POLL_INTERVAL_SEC = 1.0
MIN_DETECTION_CONFIDENCE = 0.5   # matches min_thresh usage in main_hardcoded.py

>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)

def log_to_file(message: str, severity: str = "d"):
    """
    Appends a timestamped message to debug_log.txt.
    Severity 'e': ERROR with dashes.
    Severity 'w': WARN.
    Default 'd': Standard log.
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if severity.lower() == "e":
        log_entry = f"[{timestamp}] ERROR ----------{message}----------\n"
    elif severity.lower() == "w":
        log_entry = f"[{timestamp}] WARN {message}\n"
    else:
        log_entry = f"[{timestamp}] {message}\n"

    with open("/home/shourya/robot_ws/src/bumperbot_yolo/logs/debug_log.txt", "a") as f:
        f.write(log_entry)

<<<<<<< HEAD
class SerialOperation:
    def __init__(self, port, baudrate):
        #self.arduino = serial.Serial(port, baudrate)
        log_to_file(f"initialised serial at {port} with {baudrate}")

    def send_serial_data(self, direction: str):
        commands = {
            "forward": "rp10.00,lp10.00,\r\n",
            "backward": "rn10.00,ln10.00,\r\n",
            "left": "rp10.00,ln10.00,\r\n",
            "right": "rn10.00,lp10.00\r\n",
            "stop": "rp0.00,lp0.00,\r\n"
        }
        message = commands.get(direction)
        if message:
            self.arduino.write(message.encode('utf-8'))
            log_to_file(f"sent message to arduino with msg: {message}")
        else:
            self.get_logger().info("Direction argument fits no options, please recheck")
            log_to_file("wrong direction passed in send_serial_data(), ignoring command", "w")

class Updatation:
=======

class Updatation:
    """Unchanged from main_hardcoded.py -- same Firebase/Cloudinary calls,
    reused as-is so both nodes stay consistent with each other."""

>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)
    def __init__(self):
        self.initalise_cloudinary()
        self.initialise_firebase()

    def initalise_cloudinary(self):
        cloudinary.config(cloud_name=CLOUD_NAME, api_key=API_KEY, api_secret=API_SECRET)
        log_to_file("initialised cloudinary")

    def initialise_firebase(self):
        cred = credentials.Certificate("/home/shourya/robot_ws/src/bumperbot_yolo/firebase/firebase_new_new.json")
        firebase_admin.initialize_app(cred, {"databaseURL": "https://testing-65588-default-rtdb.firebaseio.com/"})
        log_to_file("initialised firebase")

    def update_firebase_objects_detect(self, desired_state, desired_number, iteration):
        db.reference(f"/objects/object_detected{iteration}").set(desired_state)
        db.reference(f"/objects/no_objects{iteration}").set(desired_number)
        log_to_file(f"updated object's number to firebase {desired_state} and {desired_number}")

    def update_firebase_url(self, image_raw, infer_image, iteration):
        result1 = cloudinary.uploader.upload(image_raw)
        result2 = cloudinary.uploader.upload(infer_image)
        db.reference(f"/images/input{iteration}").set(result1["secure_url"])
        db.reference(f"/images/inference{iteration}").set(result2["secure_url"])
<<<<<<< HEAD
        log_to_file(f"uploaded image to cloudinary and stored url in firebase with each url being: {result1['secure_url']} and {result2['secure_url']}")

class YoloInference():
=======
        log_to_file(
            f"uploaded image to cloudinary and stored url in firebase with each url being: "
            f"{result1['secure_url']} and {result2['secure_url']}"
        )


class YoloInference:
>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)
    def __init__(self):
        log_to_file("initialised YOLO inference class")

    def load_model(self, model_path):
        if not os.path.exists(model_path):
            log_to_file("could not find path to model as specified", "e")
            sys.exit(1)

        log_to_file("model exists in path specified, returning YOLO object")
        return YOLO(model_path, task='detect')

<<<<<<< HEAD
    def determine_source_type(self, img_source):
        # Support both index (e.g., '0') and path (e.g., '/dev/camera')
        if img_source.isdigit() or img_source == '/dev/camera':
            log_to_file("source is camera")
            return 'camera'
        elif os.path.isdir(img_source):
            log_to_file("source is folder")
            return 'folder'
        elif os.path.isfile(img_source):
            log_to_file("source is image")
            return 'image'
        else:
            self.get_logger(f"Error: '{img_source}' is not a valid camera index, folder, or file.")
            log_to_file("source is not valid in determine_source_type()", "e")
            sys.exit(1)

class ImageProcessing:
    def __init__(self, updater):
        self.updater = updater
        log_to_file("initialised image processing class with updater")

    def get_and_increment_counter(self):
        counter_file = "/home/shourya/robot_ws/src/bumperbot_yolo/setup/counter_inference.txt"
        os.makedirs(os.path.dirname(counter_file), exist_ok=True)

        current_val = 1
        if os.path.exists(counter_file):
            try:
                with open(counter_file, "r") as f:
                    val = int(f.read().strip())
                    if val in (1, 2):
                        current_val = val
            except ValueError:
                current_val = 1

        # Next iteration toggles strictly between 1 and 2
        next_val = 2 if current_val == 1 else 1

        with open(counter_file, "w") as f:
            f.write(str(next_val))

        return current_val

    def open_camera(self, camera_input):
        cap = cv2.VideoCapture(camera_input, cv2.CAP_V4L2)
        if not cap.isOpened():
            log_to_file(f"could not open camera, check source specified {camera_input}", "e")
            sys.exit(1)

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        log_to_file("opened camera successfully")
        return cap

    def take_picture_from_camera(self, cap, model, min_thresh):
        pic_count = self.get_and_increment_counter()  # toggles between 1 and 2

        os.makedirs(CLICKED_DIR, exist_ok=True)
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        for _ in range(5): 
            cap.read()
            ret, frame = cap.read()

        if not ret:
            log_to_file("raised a IOerror error, failed to capture image", "e")
            raise IOError(f"Failed to capture image from camera at iteration {pic_count}")

        raw_save_path = os.path.join(CLICKED_DIR, f"pic{pic_count}.jpeg")
        cv2.imwrite(raw_save_path, frame)
        log_to_file(f"wrote image of raw image to path {raw_save_path}")

        results = model(frame, verbose=False)
=======

class WaypointImageWatcher(Node):
    """
    Polls IMAGES_TAKEN_DIR for new files written by PhotoAtWaypoint, in
    filename order (PhotoAtWaypoint names files "<waypoint_index>_<unix_ts>.png",
    so lexical/numeric ordering by waypoint index also gives capture order).

    Each filename is processed exactly once. A simple on-disk "seen" set
    (backed by an in-memory set, rebuilt from directory contents on startup)
    prevents re-processing images from a previous run still sitting in the
    folder when this node starts.
    """

    def __init__(self, model_path):
        super().__init__('yolo_inference_node')

        os.makedirs(IMAGES_TAKEN_DIR, exist_ok=True)
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        self.updater = Updatation()
        self.yolo_handler = YoloInference()
        self.model = self.yolo_handler.load_model(model_path)

        # Anything already in the folder when this node starts is treated as
        # already-processed -- avoids re-running inference on old captures
        # from a previous test run every time this node restarts.
        self.seen_files = set(os.listdir(IMAGES_TAKEN_DIR))
        log_to_file(
            f"YOLO node starting, ignoring {len(self.seen_files)} pre-existing "
            f"file(s) already in {IMAGES_TAKEN_DIR}"
        )

        self.timer = self.create_timer(POLL_INTERVAL_SEC, self.check_for_new_images)
        self.get_logger().info(f"Watching {IMAGES_TAKEN_DIR} for new waypoint images...")

    def check_for_new_images(self):
        try:
            current_files = set(os.listdir(IMAGES_TAKEN_DIR))
        except FileNotFoundError:
            log_to_file(f"{IMAGES_TAKEN_DIR} does not exist yet, will retry", "w")
            return

        new_files = current_files - self.seen_files
        if not new_files:
            return

        # If multiple files landed between polls, process oldest-first by
        # modification time so waypoint order is respected even under a
        # burst (e.g. this node was briefly down and two waypoints queued up).
        new_files_sorted = sorted(
            new_files,
            key=lambda f: os.path.getmtime(os.path.join(IMAGES_TAKEN_DIR, f))
        )

        for filename in new_files_sorted:
            filepath = os.path.join(IMAGES_TAKEN_DIR, filename)

            # Guard against picking up a file mid-write (PhotoAtWaypoint
            # writes then closes quickly, but be defensive on a slow SD
            # card/USB write): confirm file size is stable across a short
            # gap before treating it as complete.
            size_a = os.path.getsize(filepath)
            time.sleep(0.2)
            size_b = os.path.getsize(filepath)
            if size_a != size_b or size_b == 0:
                log_to_file(f"{filename} appears to still be writing, will pick it up next poll", "w")
                continue

            self.seen_files.add(filename)
            self.process_image(filepath, filename)

    def process_image(self, filepath, filename):
        log_to_file(f"processing new waypoint image: {filepath}")

        frame = cv2.imread(filepath)
        if frame is None:
            log_to_file(f"failed to read {filepath} with cv2.imread, skipping", "e")
            return

        # Use the filename (minus extension) as the iteration key for
        # Firebase/Cloudinary, e.g. "0_1786885674" -- keeps each waypoint's
        # result distinctly addressable without needing a separate counter.
        iteration = os.path.splitext(filename)[0]

        results = self.model(frame, verbose=False)
>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)
        detections = results[0].boxes

        any_object_detected = False
        valid_detections_count = 0
        for i in range(len(detections)):
<<<<<<< HEAD
            if detections[i].conf.item() > min_thresh:
=======
            if detections[i].conf.item() > MIN_DETECTION_CONFIDENCE:
>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)
                any_object_detected = True
                valid_detections_count += 1

        annotated_frame = results[0].plot()
<<<<<<< HEAD
        infer_save_path = os.path.join(OUTPUT_DIR, f"pic{pic_count}.jpeg")
        cv2.imwrite(infer_save_path, annotated_frame)
        log_to_file(f"wrote image of inferenced image to {infer_save_path}")

        # Always update object counts to Firebase regardless of detection count
        self.updater.update_firebase_objects_detect(any_object_detected, valid_detections_count, pic_count)
        log_to_file(f"updated firebase object's no in take_picture_from_camera()")

        # Always upload images to Cloudinary and update Firebase URLs regardless of detection count
        self.updater.update_firebase_url(raw_save_path, infer_save_path, pic_count)
        log_to_file("updated firebase url for images in take_picture_from_camera()")


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.declare_parameter('model', '/path/to/default/model.pt')
        self.declare_parameter('source', '0')
        self.declare_parameter('thresh', 0.5)

        self.qos_profile_pub = QoSProfile(depth=5)
        self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/input_joy/cmd_vel_stamped", 10)


        model_path = self.get_parameter('model').get_parameter_value().string_value
        img_source = self.get_parameter('source').get_parameter_value().string_value
        self.min_thresh = self.get_parameter('thresh').get_parameter_value().double_value

        self.updater = Updatation()
        self.serial_op = SerialOperation(PORT, BAUD_RATE)
        self.yolo_handler = YoloInference()
        self.img_proc = ImageProcessing(self.updater)

        self.model = self.yolo_handler.load_model(model_path)
        self.source_type = self.yolo_handler.determine_source_type(img_source)

        self.cap = None
        if self.source_type == 'camera':
            if img_source == '/dev/camera':
                camera_input = img_source
            else:
                camera_input = int(img_source)
            self.cap = self.img_proc.open_camera(camera_input)

        self.timer_ = self.create_timer(1.0, self.timer_callback)
        log_to_file("ROS2 node initialized with params")

    def send_command_movement(self, direction):
        direction = direction.lower()
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "key_teleop"

        if direction == "forward":
            message.twist.linear.x = 0.7
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.0
            self.get_logger().info("in forward")
            

        elif direction == "backward":
            message.twist.linear.x = -0.5
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.0
            self.get_logger().info("in backward")

        elif direction == "left":
            message.twist.linear.x = 0.0
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = -1.0
            self.get_logger().info("in left")

        elif direction == "left_minor":
            message.twist.linear.x = 0.0
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = -0.2
            self.get_logger().info("in left_minor")

        elif direction == "right":
            message.twist.linear.x = 0.0
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 1.0
            self.get_logger().info("in right")

        elif direction == "right_minor":
            message.twist.linear.x = 0.0
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.2
            self.get_logger().info("in right_minor")

        elif direction == "stop":
            message.twist.linear.x = 0.0
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.0
            self.get_logger().info("in stop")

        else:
            self.get_logger().info("wrong argument passed to send_command+message()")

        self.cmd_vel_pub.publish(message)
        self.get_logger().info("publishing message")


    def timer_callback(self):
        log_to_file("----------------------------CODE EXECUTION START----------------------------")

        self.get_logger().info("-------------------number for n is : 1------------------------")
        log_to_file("-------------------number for n is : 1------------------------")

        #----------------------A starts-------------------------
        log_to_file("starting movement sequence A")
        start_time = time.time()
        while time.time() - start_time < TIME_TO_MOVE_FORWARD:
            self.send_command_movement("forward")
            time.sleep(0.1)

        time.sleep(2.0)

        self.send_command_movement("left")
        time.sleep(2.0)
        self.send_command_movement("left_minor")
        time.sleep(2.0)

        try:
            self.img_proc.take_picture_from_camera(self.cap, self.model, self.min_thresh)
        except IOError as e:
            self.get_logger().error(f"error in take_picture_from_camera: {e}")
            log_to_file(f"error in take_picture_from_camera: {e}", "e")
            sys.exit(1)

        self.send_command_movement("right")
        time.sleep(2.0)
        self.send_command_movement("right_minor")
        time.sleep(2.0)

        start_time = time.time()
        while time.time() - start_time < TIME_TO_MOVE_FORWARD_TURN:
            self.send_command_movement("forward")
            time.sleep(0.1)

        time.sleep(2.0)
        log_to_file("ending movement sequence A")
        #----------------------A ends-------------------------
        
        #----------------------B starts-------------------------
        log_to_file("starting movement sequence B")
        self.send_command_movement("left")
        time.sleep(2.0)
        self.send_command_movement("left_minor")
        time.sleep(2.0)

        start_time = time.time()
        while time.time() - start_time < TIME_TO_MOVE_FORWARD_TURN_FORWARD:
            self.send_command_movement("forward")
            time.sleep(0.1)

        time.sleep(2.0)

        self.send_command_movement("left")
        time.sleep(2.0)
        self.send_command_movement("left_minor")
        time.sleep(2.0)

        try:
            self.img_proc.take_picture_from_camera(self.cap, self.model, self.min_thresh)
        except IOError as e:
            self.get_logger().error(f"error in take_picture_from_camera: {e}")
            log_to_file(f"error in take_picture_from_camera: {e}", "e")
            sys.exit(1)

        self.send_command_movement("right")
        time.sleep(2.0)
        self.send_command_movement("right_minor")
        time.sleep(2.0)

        start_time = time.time()
        while time.time() - start_time < TIME_TO_MOVE_FORWARD:
            self.send_command_movement("forward")
            time.sleep(0.1)

        time.sleep(2.0)
        log_to_file("ending movement sequence B")
        #----------------------B ENDS-------------------------

        log_to_file("----------------------------CODE EXECUTION END----------------------------")
        self.timer_.cancel()

    def destroy_node(self):
        if getattr(self, "cap", None) is not None:
            self.cap.release()
            log_to_file("closed camera connection")
        super().destroy_node()


def main():
    rclpy.init()
    main_node = MainNode()
    rclpy.spin(main_node)
    main_node.destroy_node()
    rclpy.shutdown()
=======
        infer_save_path = os.path.join(OUTPUT_DIR, f"{iteration}_inference.jpeg")
        cv2.imwrite(infer_save_path, annotated_frame)
        log_to_file(f"wrote inference result to {infer_save_path}")

        try:
            self.updater.update_firebase_objects_detect(any_object_detected, valid_detections_count, iteration)
            self.updater.update_firebase_url(filepath, infer_save_path, iteration)
        except Exception as e:
            log_to_file(f"failed to update firebase/cloudinary for {filename}: {e}", "e")
            return

        self.get_logger().info(
            f"Processed {filename}: {valid_detections_count} detection(s) above threshold"
        )


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('yolo_param_loader')
    node.declare_parameter('model', '')
    model_path = node.get_parameter('model').value
    node.destroy_node()

    if not model_path:
        print("ERROR: 'model' parameter is required, e.g. --ros-args -p model:=/path/to/model.pt")
        sys.exit(1)

    watcher = WaypointImageWatcher(model_path)
    try:
        rclpy.spin(watcher)
    except KeyboardInterrupt:
        pass
    finally:
        watcher.destroy_node()
        rclpy.shutdown()

>>>>>>> a650301 (add YOLO node to launch file, update camera node parameters, and implement main ROS functionality for image processing)

if __name__ == '__main__':
    main()