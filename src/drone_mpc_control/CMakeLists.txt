cmake_minimum_required(VERSION 3.0.2)
project(drone_mpc_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs
    sensor_msgs
    mavros 
    mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Installa gli script Python
catkin_install_python(PROGRAMS
  scripts/drone_mpc_node.py
  scripts/drone_model.py
  scripts/test_acados.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Installa i file launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Installa i file config
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)