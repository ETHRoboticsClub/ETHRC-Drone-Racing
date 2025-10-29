cmake_minimum_required(VERSION 3.12)
project(ov_bringup)

find_package(ament_cmake REQUIRED)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


