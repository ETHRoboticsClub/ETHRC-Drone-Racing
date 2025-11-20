cmake_minimum_required(VERSION 3.8)
project(aruco_pose_estimation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include Directories
include_directories(include ${OpenCV_INCLUDE_DIRS})

# Add Executable (Combining your node and the helper cpp file)
add_executable(aruco_pose_estimation_node
  src/aruco_pose_estimation_node.cpp
  src/pose_estimation_utils.cpp
)
target_link_libraries(aruco_pose_estimation_node ${OpenCV_LIBS})

# Link ROS Dependencies
ament_target_dependencies(aruco_pose_estimation_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install Target (Essential for ROS2 to find the node)
install(TARGETS aruco_pose_estimation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config/launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()