cmake_minimum_required(VERSION 3.8)
project(pcl_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find C++ dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

# Include header directories
include_directories(include)

# --- C++ Node Configuration ---
add_executable(pcl_processing_cpp src/pcl_processing.cpp)
ament_target_dependencies(pcl_processing_cpp rclcpp sensor_msgs pcl_ros)

# Install C++ executable
install(TARGETS
  pcl_processing_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# --- Python Node Configuration ---
# Install the Python executable script directly into lib/pcl_processing
install(PROGRAMS
  pcl_processing/pcl_processing.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()