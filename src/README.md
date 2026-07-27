# BumperBot ROS2 Workspace

This repository is a ROS 2 workspace for the "BumperBot" robot platform. It contains multiple packages covering bringup, controllers, firmware integration, mapping & localization, navigation (Nav2), utilities, examples, and message definitions.

This README is intentionally comprehensive: it describes the repository layout, dependencies, recommended system setup, build instructions, runtime and simulation instructions, testing, troubleshooting tips, and contribution guidance.

---

Table of Contents
- Project overview
- Supported ROS 2 distributions (recommended)
- System and ROS dependencies
- Hardware & simulator prerequisites
- Repository file tree (high-level)
- Installation (step-by-step)
  - Ubuntu / ROS 2 Humble (recommended)
  - rosdep dependency install
  - Python packages
- Building the workspace (colcon)
- Running the robot (simulation and real hardware)
  - Simulated robot (Gazebo / ros_gz_sim)
  - Real robot / bringup
  - Running individual nodes
- Launch files and common run scenarios
- Testing (colcon test)
- Development workflow and adding packages
- Troubleshooting and tips
- Contributing
- License & maintainers

---

Project overview

This workspace contains a multi-package ROS 2 project for BumperBot. It integrates firmware/hardware interfaces, controllers (C++ and Python examples), localization, mapping (SLAM), navigation (Nav2), message definitions, and example code. The repository uses standard ROS 2 ament package layout (package.xml format 3, ament_cmake, and ament_cmake_python in places).

Supported ROS 2 distributions (recommended)
- Primary tested target (recommended): ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS
- Likely compatible with later ROS 2 distributions (Rolling, Iron, etc.) but instructions and apt package names may differ. If using a different distro (Galactic, Foxy, etc.), adapt the apt package names and rosdep sources accordingly.

System and ROS dependencies

High-level dependencies used across the workspace (extracted from package.xml files):
- ROS 2 core packages: rclcpp, rclpy, std_msgs, geometry_msgs, nav_msgs
- TF2 stack: tf2, tf2_ros, tf2_geometry_msgs
- Navigation2 stack: nav2_core, nav2_costmap_2d, nav2_util
- Pluginlib
- ros_gz_sim / Gazebo integration (for simulation)
- rplidar_ros (for RPLIDAR driver integration)
- tf_transformations (runtime dependency listed in package.xml)

Notes:
- Some firmware code may depend on Python runtime scripts and system libraries for serial/UART access.
- Several packages use ament_cmake (C++) and ament_cmake_python (Python nodes) build systems.

Hardware & simulator prerequisites
- For simulation: ros_gz (ROS 2 Gazebo bridge) or gazebo/ignition packages as required by the launch files.
- For the real robot: a microcontroller/embedded controller running firmware that interfaces with ROS topics/services (see bumperbot_firmware package for details). Specific hardware (RPLIDAR A1, IMU MPU6050) is referenced in launch files.

Repository file tree (high-level)

Top-level:
- .git/
- .gitignore
- README.md (this file)
- src/  (ROS 2 workspace source folder)

Contents of src/ (packages):
- bumperbot_bringup/        # top-level launch files for bringing up the robot (real and simulated)
- bumperbot_controller/    # controller packages and teleop/joystick launchers
- bumperbot_cpp_examples/  # C++ example nodes and helper code
- bumperbot_description/    # URDF/XACRO robot description and Gazebo model integration
- bumperbot_firmware/       # firmware bridge, drivers (IMU driver, hardware_interface launch)
- bumperbot_localization/   # localization nodes and launch files
- bumperbot_mapping/        # SLAM and mapping packages and launch files
- bumperbot_motion/         # motion planner plugin(s) for Nav2
- bumperbot_msgs/           # custom message and service definitions
- bumperbot_navigation/    # navigation related launch files and configurations (Nav2)
- bumperbot_planning/       # planning nodes and configs
- bumperbot_py_examples/    # Python example nodes and scripts
- bumperbot_utils/          # utility nodes (e.g., safety_stop) and helpers

Note: Each package typically contains a package.xml and either CMakeLists.txt (C++) or setup.py/setup.cfg (Python) as appropriate.

Installation (step-by-step)

The instructions below assume a Debian-based system (Ubuntu 22.04 LTS) and ROS 2 Humble. If you're using another platform or ROS release, adapt these instructions accordingly.

1) Install system dependencies and ROS 2 (Humble) - Ubuntu 22.04

- Setup sources and keys for ROS 2 (official guide):
  - See: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Minimal summary of commands (run as a user with sudo privileges):

sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install -y ros-humble-desktop

- Install development tools often required for building ROS 2 workspaces:
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete build-essential

- Initialize rosdep:
sudo rosdep init
rosdep update

2) Clone this repository (if not already):

git clone <this-repo-url>
cd <this-repo-folder>

(For local development here, the workspace root is the repository root which contains src/.)

3) Install package dependencies using rosdep

From the workspace root (the directory containing src/):

rosdep install --from-paths src --ignore-src -r -y

This will attempt to install system packages required by the packages in src via apt. If some packages are not available as Debian packages, rosdep will indicate them and you may need to install or build them manually (for example, third-party drivers or custom packages).

4) Additional dependencies

- rplidar_ros: If the RPLIDAR driver package is not available via rosdep for your environment, you can install or build it separately. On Ubuntu/Humble:

sudo apt install -y ros-humble-rplidar-ros || echo "rplidar_ros not available via apt; see package source"

- Nav2 (Navigation2): Many systems will have nav2 packages available via apt. If not installed you can install nav2 packages or build Nav2 from source. Example apt install:

sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

- ros_gz (bridge between Ignition/Gazebo and ROS 2) if using simulation with ros_gz:

sudo apt install -y ros-humble-ros-gz

5) Python package dependencies

Some packages may include Python scripts that require extra pip packages. If present, install them in your system or virtualenv. Common commands:

python3 -m pip install --user -r src/bumperbot_py_examples/requirements.txt || true

(Replace path and requirements file with actual package requirements if present.)

Building the workspace

1) Source ROS 2 environment

Before building, source the ROS 2 installation:

source /opt/ros/humble/setup.bash

2) Build with colcon from workspace root

colcon build --symlink-install

Common options:
- --symlink-install: useful for Python packages during development
- --parallel-workers <N>: control parallel build jobs

3) Source the workspace overlay

After a successful build, source the workspace install overlay to use built packages:

source install/setup.bash

Running the robot (simulation and real hardware)

The workspace includes top-level bringup launch files. Use ros2 launch to start full stacks.

1) Simulated robot

- Example: Launch the simulated robot (Gazebo/ros_gz + bringup launch that loads simulated components):

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bumperbot_bringup simulated_robot.launch.py

This launch file includes other launch descriptions such as gazebo.launch.py, controller.launch.py, joystick_teleop.launch.py, slam.launch.py, navigation.launch.py, and global_localization.launch.py depending on configuration.

To run a specific simulation setup (e.g., start Gazebo, then separate launchers), inspect the launch directory: src/bumperbot_bringup/launch/

2) Real robot bringup

- Example: Launch the real robot bringup (connects to hardware drivers and firmware bridge):

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bumperbot_bringup real_robot.launch.py

This will include the hardware_interface launch located in bumperbot_firmware and starts hardware drivers (rplidar driver node, imu driver node, controllers, localization or SLAM based on arguments).

Optional launch arguments
- The bringup launch files define LaunchArguments; for example real_robot.launch.py defines a boolean launch argument use_slam. Use like:

ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true

3) Running individual nodes

Nodes can be run individually using ros2 run or by launching their launch files. Example:

ros2 run bumperbot_firmware mpu6050_driver.py

(Exact executable names depend on the package CMakeLists/setup.py; consult each package's executable definitions.)

Launch files and common run scenarios

Commonly used top-level launches (examples present in repo):
- bumperbot_bringup/simulated_robot.launch.py - Launch stack for running the robot in simulation
- bumperbot_bringup/real_robot.launch.py - Launch stack for bringing up the real robot and hardware nodes
- bumperbot_description/launch/gazebo.launch.py - Launch Gazebo models and spawn robot entity
- controller/joystick_teleop.launch.py - Launch joystick teleop and twist_mux
- mapping/slam.launch.py - Start SLAM stack (e.g., slam_toolbox or other SLAM node)
- navigation/navigation.launch.py - Start Nav2 with provided configs

To see available launch files, list the launch folder for each package. For example:

ls src/bumperbot_bringup/launch

Testing

Standard ROS 2 workspace tests can be run with colcon:

colcon test
colcon test-result --verbose

Note: Not all packages may include unit tests. Some integration behaviours require running in simulation or on hardware.

Development workflow and adding packages

- Add new packages to the src/ directory following ROS 2 package templates (ament_cmake for C++ and ament_python for Python).
- Update package.xml and CMakeLists.txt or setup.py accordingly.
- Run rosdep install --from-paths src --ignore-src -r -y after adding dependencies.
- Build with colcon build and source the overlay.

Tips for plugin-based packages (Nav2 plugin example)
- If a package exports plugin XML (e.g., motion_planner_plugins.xml), ensure install rules in CMakeLists.txt place plugin xml and library in the package share and lib directories respectively.

Troubleshooting and tips

- "colcon build" fails with missing dependencies: Run rosdep install --from-paths src --ignore-src -r -y and inspect the missing dependencies; install required apt packages or clone missing ROS packages into src.
- "rclcpp/rclpy not found" errors: Ensure you sourced the correct ROS 2 distribution installation: source /opt/ros/humble/setup.bash
- Runtime node cannot find message types: Ensure workspace overlay is sourced: source install/setup.bash
- Permissions for hardware serial ports: Add your user to dialout group if accessing serial devices on Linux: sudo usermod -a -G dialout $USER and re-login.
- Gazebo spawn errors: Ensure ros_gz (or the appropriate gazebo bridge) and matching Gazebo/Ignition versions are installed.

Contributing

Contributions are welcome. Suggested workflow:
- Fork and create a feature branch
- Run existing tests and linting
- Submit a PR with a clear description of changes

Maintainers and contact

Maintainer: shourya <pihushourya100@gmail.com>

License

The repository's packages include a placeholder license in package.xml. Before redistribution or production use, set an explicit license (e.g., MIT, BSD-3-Clause) in package.xml and add a LICENSE file at the repository root.

Appendix: Useful commands summary

# Setup environment and install dependencies (Ubuntu 22.04 + ROS 2 Humble)
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep build-essential
sudo rosdep init || true
rosdep update

# From workspace root
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch simulated robot
ros2 launch bumperbot_bringup simulated_robot.launch.py

# Launch real robot bringup (with optional SLAM)
ros2 launch bumperbot_bringup real_robot.launch.py use_slam:=true

---

If anything in this README doesn't match the current repository layout or package content, run:

ls -R src | sed -n '1,200p'

and inspect package.xml and CMakeLists.txt/setup.py for each package to confirm executables and launch file names.

Thanks for using BumperBot workspace! If you'd like, README can be tailored to include specific:
- ROS 2 distro variants (Galactic/I-sim)
- Platform-specific build instructions (Jetson/arm64)
- A full auto-generated file tree with file counts and sizes



