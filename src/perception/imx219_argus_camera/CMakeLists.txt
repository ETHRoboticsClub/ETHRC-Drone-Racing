cmake_minimum_required(VERSION 3.8)
project(imx219_argus_camera)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/test_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
