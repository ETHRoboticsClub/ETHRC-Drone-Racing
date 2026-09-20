cmake_minimum_required(VERSION 3.8)
project(aigp_camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-O2 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(vision_driver_component SHARED src/vision_driver_node.cpp)
target_include_directories(vision_driver_component PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_driver_component ${OpenCV_LIBS})
ament_target_dependencies(vision_driver_component rclcpp rclcpp_components sensor_msgs)

rclcpp_components_register_node(vision_driver_component
  PLUGIN "aigp_camera_driver::VisionDriverNode"
  EXECUTABLE vision_driver_node)

install(TARGETS vision_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
