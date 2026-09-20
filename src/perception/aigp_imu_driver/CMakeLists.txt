cmake_minimum_required(VERSION 3.8)
project(aigp_imu_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O2 -Wall -Wextra -Wno-address-of-packed-member)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavlink REQUIRED)

add_library(imu_bridge_component SHARED src/imu_bridge_node.cpp)
target_include_directories(imu_bridge_component PRIVATE ${mavlink_INCLUDE_DIRS})
ament_target_dependencies(imu_bridge_component rclcpp rclcpp_components sensor_msgs)

rclcpp_components_register_node(imu_bridge_component
  PLUGIN "aigp_imu_driver::ImuBridgeNode"
  EXECUTABLE imu_bridge_node)

install(TARGETS imu_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
