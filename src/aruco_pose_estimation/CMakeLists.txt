cmake_minimum_required(VERSION 3.8)
project(aruco_pose_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Marker map library (shared with standalone executable)
add_library(marker_map_lib
  src/marker_map.cpp
)
target_link_libraries(marker_map_lib ${OpenCV_LIBS})
ament_target_dependencies(marker_map_lib)

# ROS2 node for ArUco pose estimation
add_executable(aruco_pose_estimation_node
  src/aruco_pose_estimation_node.cpp
)
target_link_libraries(aruco_pose_estimation_node
  marker_map_lib
  ${OpenCV_LIBS}
)
ament_target_dependencies(aruco_pose_estimation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  aruco_pose_estimation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
