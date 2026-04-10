cmake_minimum_required(VERSION 3.8)
project(yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # Required for ament_python_install_package
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. Install the Python module folder (the folder with __init__.py)
# This allows 'import yolo_detector' to work inside your node
ament_python_install_package(${PROJECT_NAME})

# 3. Install your node scripts as executables
install(PROGRAMS
  ${PROJECT_NAME}/main_inference.py
  ${PROJECT_NAME}/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install models and image folders to the share directory
# This is where get_package_share_directory() looks
# install(
#   DIRECTORY models received_images
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
