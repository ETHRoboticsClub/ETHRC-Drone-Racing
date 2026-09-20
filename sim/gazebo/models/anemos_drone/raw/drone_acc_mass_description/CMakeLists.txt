cmake_minimum_required(VERSION 3.8)
project(drone_acc_mass_description)

# Find ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Install URDF, Meshes, and Launch files so ROS 2 can find them in 'install'
install(DIRECTORY
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
