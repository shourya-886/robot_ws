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

from datetime import datetime

import rclpy
from rclpy.node import Node


# CONSTANTS
IMAGES_TAKEN_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/images_taken"
OUTPUT_DIR = "/home/shourya/robot_ws/src/bumperbot_yolo/clicked_images_inference"
CLOUD_NAME = os.getenv("CLOUDINARY_CLOUD_NAME")
API_KEY = os.getenv("CLOUDINARY_API_KEY")
API_SECRET = os.getenv("CLOUDINARY_API_SECRET")
POLL_INTERVAL_SEC = 1.0
MIN_DETECTION_CONFIDENCE = 0.5   # matches min_thresh usage in main_hardcoded.py


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


class Updatation:
    """Unchanged from main_hardcoded.py -- same Firebase/Cloudinary calls,
    reused as-is so both nodes stay consistent with each other."""

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
        log_to_file(
            f"uploaded image to cloudinary and stored url in firebase with each url being: "
            f"{result1['secure_url']} and {result2['secure_url']}"
        )


class YoloInference:
    def __init__(self):
        log_to_file("initialised YOLO inference class")

    def load_model(self, model_path):
        if not os.path.exists(model_path):
            log_to_file("could not find path to model as specified", "e")
            sys.exit(1)

        log_to_file("model exists in path specified, returning YOLO object")
        return YOLO(model_path, task='detect')


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
        detections = results[0].boxes

        any_object_detected = False
        valid_detections_count = 0
        for i in range(len(detections)):
            if detections[i].conf.item() > MIN_DETECTION_CONFIDENCE:
                any_object_detected = True
                valid_detections_count += 1

        annotated_frame = results[0].plot()
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


if __name__ == '__main__':
    main()