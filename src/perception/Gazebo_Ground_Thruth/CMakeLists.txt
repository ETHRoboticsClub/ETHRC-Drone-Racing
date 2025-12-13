cmake_minimum_required(VERSION 3.8)
project(Gazebo_Ground_Thruth)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable
add_executable(Gazebo_Ground_Thruth src/Gazebo_Ground_Thruth.cpp)

ament_target_dependencies(Gazebo_Ground_Thruth
  rclcpp
  nav_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  Gazebo_Ground_Thruth
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
