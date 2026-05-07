cmake_minimum_required(VERSION 3.8)
project(jetson_h264_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

include_directories(
  ${GST_INCLUDE_DIRS}
)

add_executable(encoder_node src/encoder_node.cpp)
target_link_libraries(encoder_node
  ${GST_LIBRARIES}
)
ament_target_dependencies(encoder_node
  rclcpp
)

install(TARGETS
  encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
