# BumperBot — Autonomous Wall Crack Inspection Robot

A ROS 2 Humble workspace for **BumperBot**, a custom differential-drive robot designed to autonomously navigate an environment, capture images at inspection waypoints, detect wall cracks using a YOLO model, and store selected results in Firebase and Cloudinary.

The project combines custom hardware, ROS 2 navigation, SLAM/localization, waypoint-based inspection, computer vision, and cloud logging into a single robot workspace.

## Features

### Currently implemented

- Differential-drive robot with wheel encoders and Arduino Mega motor control
- `ros2_control` hardware interface communicating with the robot over UART
- RPLIDAR A1 integration
- MPU-6050 IMU driver
- SLAM using `slam_toolbox`
- AMCL localization using saved maps
- Nav2 autonomous navigation
- Waypoint-based autonomous inspection
- Automatic image capture during waypoint inspection
- YOLO-based crack/object detection using an ONNX model
- CUDA/ONNX Runtime inference support on the Jetson Orin Nano
- Uploading selected raw and annotated images to Cloudinary
- Detection status and image URLs stored in Firebase Realtime Database
- Gazebo simulation support
- Joystick teleoperation and custom controller implementations
- Custom A* and Dijkstra Nav2 planner implementations available for experimentation

### Planned / in progress

- Battery Monitoring System (BMS) integration — tracked in GitHub Issue #4
- Continued calibration of waypoints and robot navigation
- Further refinement of the inspection and cloud-upload pipeline
- Improved portability by removing remaining machine-specific paths

---

## System Overview

```text
 RPLIDAR ───────────────┐
 MPU-6050 ──────────────┤
 Camera ────────────────┤
                         ▼
                 ┌───────────────┐
                 │ Jetson Orin   │
                 │ Nano Super    │
                 └───────┬───────┘
                         │ UART
                         ▼
                 ┌───────────────┐
                 │ Arduino Mega  │
                 └───────┬───────┘
                         ▼
                    Motor Driver
                         ▼
                 Differential Drive

 Jetson pipeline:
 SLAM / AMCL → Nav2 → Waypoints → Camera Capture → YOLO ONNX
                                                   ↓
                                      Firebase + Cloudinary
```

---

## Workspace Structure

The workspace currently contains the following ROS 2 packages:

```text
robot_ws/
├── README.md
├── .gitignore
└── src/
    ├── bumperbot_bringup/        # Top-level real and simulated robot launch files
    ├── bumperbot_controller/     # Controllers, odometry, teleoperation and twist tools
    ├── bumperbot_cpp_examples/   # ROS 2 C++ learning examples
    ├── bumperbot_description/    # URDF/Xacro robot description and simulation assets
    ├── bumperbot_firmware/       # Arduino firmware, hardware interface and IMU driver
    ├── bumperbot_localization/   # AMCL and localization-related nodes
    ├── bumperbot_mapping/        # SLAM and map-related launch/configuration
    ├── bumperbot_motion/         # Custom Nav2 motion/controller plugins
    ├── bumperbot_msgs/           # Custom ROS messages and services
    ├── bumperbot_navigation/     # Nav2 configuration and waypoint navigation
    ├── bumperbot_planning/       # Custom A* and Dijkstra planner plugins
    ├── bumperbot_py_examples/    # ROS 2 Python learning examples
    ├── bumperbot_utils/          # Utility and safety-related nodes
    └── bumperbot_yolo/           # Camera, YOLO inference and cloud logging pipeline
        ├── main/                 # Python ROS nodes
        ├── models/               # YOLO/ONNX model files
        ├── images_taken/         # Images captured during inspection
        ├── sample_images/        # Test images
        ├── logs/                 # Runtime logs
        └── setup.py              # Python package entry points
```

The repository contains both the production robot packages and several learning/experimental packages. The latter are useful references but are not necessarily launched during a normal robot run.

---

## Requirements

### Operating system and ROS

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3
- `colcon` and `rosdep`

### ROS packages

The workspace depends on ROS packages including:

- Navigation2 / Nav2
- `slam_toolbox`
- `rplidar_ros`
- `ros2_control` and `ros2_controllers`
- `robot_localization` for localization experiments
- Gazebo / ROS-Gazebo integration for simulation

Install dependencies declared by the workspace first using `rosdep`. Some hardware- or environment-specific dependencies may need to be installed separately.

### Computer vision and cloud pipeline

The current inspection pipeline uses:

- OpenCV (`cv2`)
- Ultralytics YOLO
- ONNX Runtime / CUDA-capable inference environment
- Firebase Admin SDK
- Cloudinary Python SDK

A compatible ONNX crack-detection model must be available locally and passed to the launch file or ROS node.

### Hardware for the real robot

- NVIDIA Jetson Orin Nano Super
- Arduino Mega 2560
- Differential-drive motors with encoders
- Motor driver
- RPLIDAR A1
- MPU-6050
- USB camera

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/shourya-886/robot_ws.git
cd robot_ws
```

### 2. Install ROS dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` reports packages that are not available through the system dependency database, install the corresponding ROS or Python dependencies manually.

### 3. Install Python dependencies for the YOLO pipeline

Install the Python packages required by the inspection node in the environment used to run ROS:

```bash
python3 -m pip install ultralytics opencv-python firebase-admin cloudinary
```

For the Jetson, use an inference stack compatible with the installed CUDA and JetPack version. The current project uses ONNX models to avoid loading a PyTorch `.pt` model at runtime.

### 4. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

After modifying packages, rebuild and source again:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## How to Run

### Real robot — navigation/localization

The main real-robot bringup supports optional SLAM, YOLO, and waypoint inspection:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch bumperbot_bringup real_robot.launch.py
```

Useful launch options:

```bash
# Run SLAM instead of normal map-based localization
ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true

# Enable the camera and YOLO inspection pipeline
ros2 launch bumperbot_bringup real_robot.launch.py use_yolo:=true

# Enable waypoint following
ros2 launch bumperbot_bringup real_robot.launch.py use_waypoint:=true
```

They can be combined:

```bash
ros2 launch bumperbot_bringup real_robot.launch.py \
  use_slam:=false \
  use_yolo:=true \
  use_waypoint:=true
```

### Specify a YOLO ONNX model

The current launch file accepts a `yolo_model` parameter:

```bash
ros2 launch bumperbot_bringup real_robot.launch.py \
  use_yolo:=true \
  yolo_model:=/path/to/model.onnx
```

The current inference node is designed specifically for `.onnx` models.

### Simulation

The repository includes a simulated robot bringup:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

### Map saving

After creating a map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/my_map
```

### Arduino firmware

Upload the production robot firmware from the `bumperbot_firmware` package to the Arduino Mega before starting the real robot bringup. Ensure the serial device configuration matches the hardware connected to the Jetson.

---

## Inspection Pipeline

The current autonomous inspection workflow is structured as follows:

1. Nav2 moves the robot through configured inspection waypoints.
2. The camera pipeline captures images during the inspection process.
3. Images are written to `bumperbot_yolo/images_taken/`.
4. The YOLO node watches for newly captured images.
5. Each new image is processed with the configured ONNX model.
6. An annotated inference image is saved locally.
7. Selected waypoint results are uploaded to Cloudinary.
8. Detection information and Cloudinary URLs are stored in Firebase.

The inference node deliberately processes newly created images once rather than continuously running inference on every camera frame. This keeps the inspection workflow tied to navigation and reduces unnecessary GPU and memory usage.

---

## Configuration Notes

### Cloud credentials

Cloudinary credentials are read from environment variables:

```bash
export CLOUDINARY_CLOUD_NAME="..."
export CLOUDINARY_API_KEY="..."
export CLOUDINARY_API_SECRET="..."
```

Firebase credentials are required for the cloud logging pipeline. Do not commit private credentials or API secrets to the repository.

### Device paths

The real robot uses serial devices for the Arduino and RPLIDAR. Stable udev rules are recommended so device names do not change after rebooting or reconnecting hardware.

---

## Recent Development Activity

Recent commits show a clear shift toward completing the full autonomous inspection pipeline. Work in August 2026 included:

- Adding and integrating the YOLO ROS node
- Adding conditional YOLO launch support and model configuration
- Reducing camera frame rate to control Jetson memory usage
- Calibrating waypoint positions and navigation behavior
- Moving and testing model files within the workspace
- Switching the inference workflow to working ONNX models
- Integrating waypoint navigation with the inspection workflow

The latest repository commit indicates that the current code is working and that battery monitoring is the next planned subsystem.

---

## Known Issues and Limitations

This is an actively developed robotics project. The main limitations currently visible from the codebase and development history are:

- **Machine-specific paths:** Parts of the YOLO pipeline currently use absolute paths under `/home/shourya/robot_ws`. These should eventually be replaced with package-share paths, ROS parameters, or configurable locations.
- **Cloud configuration:** Firebase credentials and Cloudinary configuration are environment/deployment specific and must be configured before running the upload pipeline.
- **Package metadata:** Some newer package metadata still contains placeholder descriptions or license fields and should be cleaned up for a production-quality public repository.
- **Hardware calibration:** Wheel parameters, waypoints, and navigation behavior require calibration on the physical robot.
- **BMS integration:** Battery monitoring has not yet been integrated and is tracked as a future task.

---

## Development Notes

This repository includes custom implementations of planners, controllers, localization components, and ROS examples alongside the production pipeline. Some of these components are experimental or educational and may not be active in the default launch configuration.

When modifying the robot, a practical workflow is:

```bash
git pull
colcon build --symlink-install
source install/setup.bash
```

Test changes in simulation where possible before deploying them to the physical robot.

---

## Roadmap

- [x] Differential-drive robot control and odometry
- [x] RPLIDAR integration
- [x] SLAM and map-based localization
- [x] Nav2 autonomous navigation
- [x] Waypoint-based inspection
- [x] Camera integration
- [x] YOLO-based ONNX inference
- [x] Firebase and Cloudinary logging pipeline
- [ ] Battery Monitoring System integration
- [ ] Remove remaining hardcoded paths and improve deployment configuration
- [ ] Continue navigation and inspection calibration

---

## Repository

**GitHub:** https://github.com/shourya-886/robot_ws

This project is under active development. Issues, experiments, and implementation details may change as the physical robot and inspection pipeline continue to evolve.
