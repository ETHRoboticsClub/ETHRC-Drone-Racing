cmake_minimum_required(VERSION 3.8)
project(ekf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Mock publisher for scenario testing ──
add_executable(mock_publisher src/nodes/mock_publisher.cpp)
target_include_directories(mock_publisher PRIVATE include)
ament_target_dependencies(mock_publisher rclcpp nav_msgs geometry_msgs)
install(TARGETS mock_publisher DESTINATION lib/${PROJECT_NAME})

# Install launch files and configs
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(robot_localization REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(angles REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_add_gtest(test_drone_ekf test/test_drone_ekf.cpp)
  target_include_directories(test_drone_ekf PRIVATE
    ${robot_localization_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_drone_ekf
    rclcpp robot_localization angles tf2 tf2_geometry_msgs
    tf2_ros sensor_msgs nav_msgs geometry_msgs)
  target_link_libraries(test_drone_ekf
    ${robot_localization_LIBRARIES} yaml-cpp)
endif()

ament_package()
