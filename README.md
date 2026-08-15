# BumperBot — Autonomous Wall Crack Inspection Robot

**ROS 2 Humble workspace for a differential-drive robot built for autonomous crack detection on heritage monument walls.**
Built for WRO 2026 (Future Engineers). Also referred to as *Project Heritage Shield* / *Project Sentinel* in competition documentation.

- **Team:** Shourya, Prisha Prasad, Medhansh Yogesh Jagtap
- **Mentor:** Santosh Kumar
- **Schools:** Akshara International School, Phoenix Greens School of Learning
- **Maintainer contact:** pihushourya100@gmail.com

---

## Table of Contents

1. [What this robot does](#what-this-robot-does)
2. [Hardware](#hardware)
3. [System architecture](#system-architecture)
4. [TF tree](#tf-tree)
5. [Package-by-package breakdown](#package-by-package-breakdown)
6. [Installation](#installation)
7. [Building the workspace](#building-the-workspace)
8. [Running the robot](#running-the-robot)
9. [Key configuration values](#key-configuration-values)
10. [Design decisions & known quirks](#design-decisions--known-quirks)
11. [Known issues / active investigations](#known-issues--active-investigations)
12. [Development workflow](#development-workflow)
13. [Troubleshooting](#troubleshooting)
14. [Roadmap](#roadmap)
15. [License](#license)

---

## What this robot does

BumperBot autonomously navigates around heritage monument interiors/exteriors using a pre-built map, and uses a YOLO/VLM-based vision pipeline to detect cracks and structural damage in walls as it moves. Detected cracks (with location context) are logged to a Firebase backend, with image storage on Cloudinary, for later review by conservators/engineers. The robot is built from scratch — chassis designed in Fusion 360, full sensor and compute stack integrated onto a 3-tier acrylic frame.

The competition-facing narrative: a robot that can be sent into a monument, roam autonomously along walls, flag areas of concern, and produce a navigable digital record of surface condition — without requiring a human to physically inspect every wall segment.

---

## Hardware

| Component | Spec |
|---|---|
| **Onboard compute** | NVIDIA Jetson Orin Nano Super Developer Kit |
| **Development PC** | ASUS AiO V470, Intel Core i5-13420H (13th gen), dual-boot Windows 11 + Ubuntu 22.04 LTS |
| **Motor controller / MCU** | Arduino Mega 2560 R3 |
| **Drive motors** | N20 GA12-N20 12V motors with quadrature encoders |
| **Motor driver** | L298N dual H-bridge |
| **LIDAR** | RPLIDAR A1M8 (running ~7.5Hz on standard USB 5V bus power — see [Design decisions](#design-decisions--known-quirks)) |
| **IMU** | MPU-6050 |
| **Power** | 3x 21700 Li-ion cells via Waveshare UPS Module C; 11.1V rail for drive domain |
| **Arduino <-> Jetson link** | Hardware UART, `Serial2` @ 115200 baud, with a voltage divider on Arduino TX -> Jetson RX |
| **Chassis** | Custom 3D-printed/acrylic base plate, designed in Fusion 360 |
| **Design tools** | Fusion 360, KiCad (Windows) - VS Code, PyCharm (both OS) |
| **Simulation** | Ignition Gazebo 6, ROS 2 Humble, Nav2 (Ubuntu) |

A prior-generation Jetson Nano (carrier board TPS25944L overheating issue) was retired from this project after power-delivery problems; the Orin Nano Super is the current and only onboard compute target.

---

## System architecture

```
                    +-------------------------------------------+
                    |              Jetson Orin Nano              |
                    |  +-------------+    +------------------+   |
   RPLIDAR A1M8 --->|  |  Nav2 Stack |    |  YOLO/VLM crack  |   |
     (USB)          |  | SLAM/AMCL   |    |  detection node  |   |
                    |  +------+------+    +--------+---------+   |
                    |         |                     |             |
                    |  +------v---------------------v---------+  |
                    |  |        ros2_control / hardware_interface| |
                    |  +------------------+---------------------+ |
                    +---------------------|-----------------------+
                                           | UART (Serial2, 115200 baud)
                                    +------v-------+
                                    | Arduino Mega |--> L298N --> N20 motors + encoders
                                    |    2560 R3   |
                                    +--------------+
                                           |
                                    MPU-6050 (I2C, direct to Jetson via mpu6050_driver.py)

                    Crack detections --> Firebase (data) + Cloudinary (images)
```

- **Wheel odometry** is computed on the Arduino (fixed 100ms loop) and streamed as velocity over UART; the Jetson-side `bumperbot_interface.cpp` hardware interface integrates this into wheel position for `ros2_control`.
- **`diff_drive_controller`** (stock ROS 2 control plugin) is the active controller on both real and simulated robot -- the workspace also contains custom `simple_controller`/`simple_velocity_controller` alternatives for teaching/comparison purposes, selectable via the `use_simple_controller` launch argument.
- **Localization**: either `slam_toolbox` (`async`/`sync_slam_toolbox_node`) while mapping, or `map_server` + `amcl` once a saved map exists, selected via the `use_slam` launch argument. An EKF-based IMU/odometry fusion pipeline (`bumperbot_localization`) exists in the workspace but is **not currently wired into either bringup path** -- see [Known issues](#known-issues--active-investigations).
- **Navigation**: standard Nav2 stack -- `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`, each with its own `lifecycle_manager`.

---

## TF tree

```
map --> odom --> base_footprint --> base_link -+--> wheel_left_link
                                                |--> wheel_right_link
                                                |--> caster_front_link
                                                |--> caster_rear_link
                                                |--> laser_link
                                                +--> imu_link
```

- `map -> odom`: published by AMCL (post-mapping) or `slam_toolbox` (while mapping).
- `odom -> base_footprint`: published by `diff_drive_controller` (`enable_odom_tf: true`).
- `base_footprint -> base_link`: fixed joint (`base_joint`), defined in `bumperbot.urdf.xacro`.
- `caster_front_link` and `caster_rear_link` are geometrically identical (same mass, inertia, and mesh) -- the robot uses two passive casters.
- Wheel joint declaration order in `bumperbot_ros2_control.xacro`: **`wheel_left_joint` (index 0), `wheel_right_joint` (index 1)** -- this ordering is what every controller and the firmware's UART `'l'`/`'r'` index mapping must agree with.

---

## Package-by-package breakdown

| Package | Purpose | Key nodes/executables |
|---|---|---|
| **bumperbot_bringup** | Top-level entry points that tie every other package together for a full run | `real_robot.launch.py`, `simulated_robot.launch.py` |
| **bumperbot_description** | URDF/xacro robot model, meshes, Gazebo/`ros_gz` integration | `gazebo.launch.py`, `display.launch.py`; xacro files under `urdf/` |
| **bumperbot_firmware** | Jetson<->Arduino hardware interface, Arduino sketches, IMU driver | `hardware_interface.launch.py` (spawns `ros2_control_node` using `bumperbot_interface.cpp`), `mpu6050_driver.py`; Arduino sketch: `firmware/robot_control/robot_control.ino` |
| **bumperbot_controller** | Diff-drive control, odometry (real + intentionally-noisy variants for EKF testing), joystick teleop, twist relaying | `simple_controller`, `noisy_controller`, `twist_relay`; launch: `controller.launch.py`, `joystick_teleop.launch.py` |
| **bumperbot_localization** | AMCL-based global localization, and a (currently unwired) EKF-based local localization pipeline fusing wheel odom + IMU | `imu_republisher`; launch: `global_localization.launch.py`, `local_localization.launch.py`; dead code: `kalman_filter.cpp`, `odometry_motion_model.cpp` (tutorial exercises, not referenced by any launch file) |
| **bumperbot_mapping** | SLAM (`slam_toolbox`) and map-saving | `mapping_with_known_poses` (custom, standalone); launch: `slam.launch.py` |
| **bumperbot_navigation** | Nav2 configuration -- costmaps, controller/planner/behavior/smoother/BT-navigator params | launch: `navigation.launch.py`; configs: `costmap.yaml`, `controller_server.yaml`, `planner_server.yaml`, `behavior_server.yaml`, `smoother_server.yaml`, `bt_navigator.yaml` |
| **bumperbot_motion** | Custom Nav2 `FollowPath` controller plugins (alternatives to the stock RPP controller) | `bumperbot_motion::PurePursuit`, `bumperbot_motion::PDMotionPlanner` -- both implemented, currently **dormant** (commented out in `controller_server.yaml` in favor of `RegulatedPurePursuitController`) |
| **bumperbot_planning** | Custom Nav2 global planner plugins (alternatives to the stock Smac planner) | `bumperbot_planning::AStarPlanner`, `bumperbot_planning::DijkstraPlanner` -- both implemented, currently **dormant** (commented out in `planner_server.yaml` in favor of `SmacPlanner2D`) |
| **bumperbot_utils** | Safety-stop node (twist_mux lock pattern) | `safety_stop` (currently commented out in bringup) |
| **bumperbot_msgs** | Custom message/service definitions | -- |
| **bumperbot_py_examples** / **bumperbot_cpp_examples** | ROS 2 concept tutorials (publishers, subscribers, TF, lifecycle nodes, QoS, services) written while learning ROS 2 | Not used by the robot itself |

---

## Installation

Tested target: **ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS**.

### 1. Install ROS 2 Humble

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep build-essential
sudo rosdep init
rosdep update
```

### 2. Clone and install dependencies

```bash
git clone https://github.com/shourya-886/robot_ws.git
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Additional packages not always covered by rosdep

```bash
sudo apt install -y ros-humble-rplidar-ros
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-ros-gz
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-robot-localization   # for the (currently unwired) EKF pipeline
```

### 4. udev rules for stable device names

The workspace expects fixed device paths (`/dev/rplidar`, `/dev/arduino`) rather than `/dev/ttyUSB0`-style names that can shift between reboots. Set up udev rules mapping these by vendor/product ID (or serial number) for the RPLIDAR and Arduino Mega, and ensure your user is in the `dialout` group:

```bash
sudo usermod -a -G dialout $USER   # then log out/in
```

---

## Building the workspace

```bash
source /opt/ros/humble/setup.bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the robot

### Simulated robot (Gazebo)

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true
```

### Real robot

```bash
ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true
```

Set `use_slam:=false` once a map has been saved, to run AMCL against a static map instead of building a new one.

### Uploading Arduino firmware

Flash `src/bumperbot_firmware/firmware/robot_control/robot_control.ino` to the Arduino Mega 2560 R3 before running `real_robot.launch.py` -- this is the sketch that reads encoders, closes the PID velocity loop, and talks `Serial2` to the Jetson. The other sketches in `firmware/` (`simple_encoder_reader`, `simple_motor_control`, `simple_serial_receiver`, `simple_serial_transmitter`) are standalone bring-up/debug sketches, not the production firmware.

### Saving a map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/my_map
```

### Individual nodes

```bash
ros2 run bumperbot_firmware mpu6050_driver.py
ros2 launch bumperbot_controller joystick_teleop.launch.py
```

---

## Key configuration values

| Parameter | Value | Location |
|---|---|---|
| Wheel separation | 0.31265 m | `bumperbot_controller/config/bumperbot_controllers.yaml` |
| Wheel radius | 0.055 m | same |
| Max linear velocity | 0.7 m/s | same |
| Max angular velocity | 8.5 rad/s | same |
| `controller_manager` update rate | 100 Hz | same |
| Arduino control loop interval | 100 ms (fixed) | `firmware/robot_control/robot_control.ino` |
| UART baud rate | 115200 | `bumperbot_description/urdf/bumperbot_ros2_control.xacro`, `bumperbot_bringup/config/rplidar_a1.yaml` |
| Arduino serial device | `/dev/arduino` | `bumperbot_ros2_control.xacro` |
| RPLIDAR serial device | `/dev/rplidar` | `rplidar_a1.yaml` |
| RPLIDAR actual scan rate | ~7.5Hz (not 10Hz -- see below) | -- |
| Costmap resolution | 0.05 m/cell | `bumperbot_navigation/config/costmap.yaml` |
| Robot radius (costmap) | 0.1 m | same |
| Active global planner | `nav2_smac_planner/SmacPlanner2D` | `planner_server.yaml` |
| Active local controller | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | `controller_server.yaml` |

---

## Design decisions & known quirks

- **RPLIDAR A1M8 runs at ~7.5Hz, not the nameplate 10Hz.** This is expected, not a bug: per Slamtec's datasheet, reaching the full 5.5-10Hz range requires a dedicated 9V supply to the motor, separate from standard 5V USB bus power. Nav2/`slam_toolbox` parameters have been tuned to work correctly at this rate; `scan_mode` in `rplidar_a1.yaml` does not control rotation Hz (only sample rate/measurement mode).
- **Two controller implementations exist side by side.** `diff_drive_controller` (stock) is the actual active controller on both real and sim bringups (`use_simple_controller:="False"` in `real_robot.launch.py`/`simulated_robot.launch.py`). The custom `simple_controller`/`simple_velocity_controller` path exists for comparison/teaching and can be enabled via the `use_simple_controller` launch argument.
- **`noisy_controller` runs unconditionally** alongside whichever main controller is active, publishing to `/bumperbot_controller/odom_noisy` -- a deliberately-noise-injected odometry source intended as the `odom0` input for the (currently unwired) EKF pipeline.
- **Two full sets of Nav2 planner/controller plugins exist**: the stock Nav2 ones (currently active) and custom `bumperbot_planning`/`bumperbot_motion` implementations (currently dormant, commented out in config). Both custom sets are functional and were written as learning exercises / potential fallbacks.
- **`caster_front_link` and `caster_rear_link` are exact geometric copies** (same mass, inertia, mesh) -- intentional, not a copy-paste bug.

---

## Known issues / active investigations

This section is kept current as issues are found and fixed during development -- treat it as a living log, not a final state.

| Issue | Status |
|---|---|
| Rotational odometry drift (suspected toe-in/toe-out from 3D-printed base plate, plus prior software integration bugs) | Two software contributors fixed (heading-integration midpoint fix in `simple_controller.cpp`/`noisy_controller.cpp`; left/right wheel index mapping corrected in `bumperbot_interface.cpp`). Physical wheel-separation measurement (27-33cm spread observed) still under investigation. |
| `bumperbot_interface.cpp` uses host-measured `dt` (subject to USB/scheduling jitter) instead of the Arduino's known fixed 100ms loop interval | Identified, fix pending |
| `slam.launch.py` `map_saver_server` threshold parameters use malformed Python set-literal syntax instead of dict syntax | Identified; a corrected version was tested and reportedly triggered lifecycle-manager activation errors -- root cause not yet confirmed, left on the working (malformed, defaults-only) version pending further diagnosis |
| EKF/IMU-fusion localization pipeline (`local_localization.launch.py`) is implemented but not included in any bringup file -- Nav2 currently runs on raw wheel odometry only | Deferred pending resolution of MPU-6050 electrical noise / lack of dedicated battery monitoring for trigger logic |
| `AStarPlanner`/`DijkstraPlanner::poseToCell()` use `getOriginX()` instead of `getSizeInCellsX()` -- wrong cell-index formula | Dead code (plugins currently dormant), fix pending before either is ever enabled |
| Arduino Mega USB (ATmega16U2) enumeration failure on a separate unit, suspected hardware damage | Isolated from the `Serial2` UART path used for runtime motor control; does not affect robot operation, only firmware flashing/debugging on the affected board |

---

## Development workflow

1. Add new packages under `src/` following standard `ament_cmake`/`ament_python` layout.
2. Update `package.xml` dependencies, then `rosdep install --from-paths src --ignore-src -r -y`.
3. `colcon build --symlink-install` and re-source `install/setup.bash`.
4. For new Nav2 plugins (planner/controller/task-executor), export via `pluginlib` XML and ensure `CMakeLists.txt` installs both the plugin XML and the built library to the package's share/lib directories -- follow the pattern already used in `bumperbot_planning`/`bumperbot_motion`.
5. Firmware changes: edit `firmware/robot_control/robot_control.ino`, re-flash the Arduino Mega, and confirm the UART message format (`l`/`r` + `p`/`n` + value, comma-separated) still matches what `bumperbot_interface.cpp` expects.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `colcon build` fails on missing deps | `rosdep install --from-paths src --ignore-src -r -y` |
| `rclcpp`/`rclpy` not found | `source /opt/ros/humble/setup.bash` before building/running |
| Node can't find custom message types | `source install/setup.bash` after building |
| Permission denied on `/dev/arduino` or `/dev/rplidar` | Add user to `dialout` group, re-login; confirm udev rules created the expected symlink |
| Gazebo spawn errors | Confirm `ros-humble-ros-gz` and Ignition Gazebo 6 versions match what `gazebo.launch.py` expects |
| RPLIDAR reports <10Hz on `ros2 topic hz /scan` | Expected behavior on standard USB bus power -- see [Design decisions](#design-decisions--known-quirks) |
| Robot doesn't drive straight on a pure `linear.x` command | Check the `'l'`/`'r'` UART index mapping in `bumperbot_interface.cpp` against the `wheel_left_joint`(0)/`wheel_right_joint`(1) order in `bumperbot_ros2_control.xacro` |
| `slam_toolbox` map corrupted / `odom -> base_footprint` TF appears frozen | Confirm only one localization source (AMCL or `slam_toolbox`) is publishing `map -> odom` at a time -- check `use_slam` argument and that `global_localization.launch.py`/`slam.launch.py` aren't both active |

---

## Roadmap

- [ ] Resolve physical wheel-separation/toe alignment (chassis-level fix)
- [ ] Apply pending `bumperbot_interface.cpp` `dt` fix (hardcode to match Arduino's 100ms loop)
- [ ] Diagnose `slam.launch.py` map_saver_server lifecycle-manager error with corrected parameter dict syntax
- [ ] Fix dormant `AStarPlanner`/`DijkstraPlanner::poseToCell()` cell-index bug
- [ ] Investigate MPU-6050 noise mitigation, then wire `local_localization.launch.py` (EKF) into bringup
- [ ] Integrate YOLO/VLM crack-detection pipeline with Nav2 Waypoint Follower via a custom `WaypointTaskExecutor` plugin, so the robot autonomously stops and inspects at each wall-scan point
- [ ] Firebase live dashboard (SSE streaming) for crack-detection results
- [ ] Costmap keepout-zone filter masks for protected/fragile monument areas
- [ ] Command-triggered Nav2 Docking Server demo (fixed-pose, non-charging dock -- no battery monitoring hardware currently installed)

---

## License

Package `package.xml` files currently contain placeholder license fields. Set an explicit license (e.g. MIT, BSD-3-Clause) before any public redistribution, and add a `LICENSE` file at the repository root.
