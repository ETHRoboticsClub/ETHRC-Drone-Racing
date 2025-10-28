cmake_minimum_required(VERSION 3.8)
project(ekf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Receiver node
add_executable(ekf_node src/nodes/ekf_node.cpp)
ament_target_dependencies(ekf_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# Publisher node
add_executable(mock_publisher src/nodes/mock_publisher.cpp)
ament_target_dependencies(mock_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mock_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
  DESTINATION include/)

ament_package()
