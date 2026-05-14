cmake_minimum_required(VERSION 3.8)
project(imx219_argus_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

  add_executable(argus_camera_node src/argus_camera_node.cpp)
  target_include_directories(argus_camera_node PRIVATE ${GST_INCLUDE_DIRS})
  target_link_libraries(argus_camera_node ${GST_LIBRARIES})
  ament_target_dependencies(argus_camera_node rclcpp sensor_msgs)

  install(TARGETS argus_camera_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Skipping imx219_argus_camera (Jetson only)")
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/test_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
