cmake_minimum_required(VERSION 3.8)
project(betaflight_blackbox_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(msp REQUIRED msp)
include_directories(${msp_INCLUDE_DIRS})
link_directories(${msp_LIBRARY_DIRS})

add_executable(blackbox_driver_node src/blackbox_driver_node.cpp)

ament_target_dependencies(blackbox_driver_node
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
  diagnostic_msgs
)

find_library(SERIALPORT_LIBRARY NAMES serialport REQUIRED)
target_link_libraries(blackbox_driver_node ${SERIALPORT_LIBRARY})

add_executable(msp_imu_driver_node src/msp_imu_driver_node.cpp)

ament_target_dependencies(msp_imu_driver_node
  rclcpp
  builtin_interfaces
  sensor_msgs
  std_msgs
)
target_link_libraries(msp_imu_driver_node ${msp_LIBRARIES})

install(TARGETS
    blackbox_driver_node
    msp_imu_driver_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
