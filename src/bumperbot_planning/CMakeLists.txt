cmake_minimum_required(VERSION 3.8)
project(bumperbot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

add_library(dijkstra_planner SHARED src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner ${dependencies})

add_library(a_star_planner SHARED src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner ${dependencies})

add_library(crack_detection_executor SHARED src/crack_detection_executor.cpp)
ament_target_dependencies(crack_detection_executor ${dependencies})

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)
pluginlib_export_plugin_description_file(nav2_core waypoint_task_executor_plugins.xml)

install(TARGETS
  dijkstra_planner
  a_star_planner
  crack_detection_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(FILES global_planner_plugins.xml waypoint_task_executor_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/a_star_planner.py
  ${PROJECT_NAME}/dijkstra_planner.py
  ${PROJECT_NAME}/send_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  dijkstra_planner
  a_star_planner
  crack_detection_executor
)
ament_export_dependencies(${dependencies})
ament_package()