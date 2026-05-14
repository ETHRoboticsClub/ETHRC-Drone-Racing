cmake_minimum_required(VERSION 3.8)
project(betaflight_blackbox_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(blackbox_driver_node src/blackbox_driver_node.cpp)

ament_target_dependencies(blackbox_driver_node
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
)

find_library(SERIALPORT_LIBRARY NAMES serialport REQUIRED)
target_link_libraries(blackbox_driver_node ${SERIALPORT_LIBRARY})

install(TARGETS
    blackbox_driver_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
