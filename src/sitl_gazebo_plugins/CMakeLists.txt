cmake_minimum_required(VERSION 3.8)
project(sitl_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim QUIET)
find_package(yaml-cpp REQUIRED)

if(gz-sim_FOUND)
  find_package(gz-plugin REQUIRED)
  find_package(gz-math REQUIRED)
  find_package(gz-transport REQUIRED)
  find_package(gz-msgs REQUIRED)
  set(GZ_SIM_TARGET gz-sim::gz-sim)
  set(GZ_PLUGIN_TARGET gz-plugin::gz-plugin)
  set(GZ_MATH_TARGET gz-math::gz-math)
  set(GZ_TRANSPORT_TARGET gz-transport::gz-transport)
  set(GZ_MSGS_TARGET gz-msgs::gz-msgs)
else()
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  find_package(gz-math7 REQUIRED)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  set(GZ_SIM_TARGET gz-sim8::gz-sim8)
  set(GZ_PLUGIN_TARGET gz-plugin2::gz-plugin2)
  set(GZ_MATH_TARGET gz-math7::gz-math7)
  set(GZ_TRANSPORT_TARGET gz-transport13::gz-transport13)
  set(GZ_MSGS_TARGET gz-msgs10::gz-msgs10)
endif()

add_library(boreas_dynamics STATIC src/boreas_dynamics.cpp)
target_compile_features(boreas_dynamics PUBLIC cxx_std_17)
target_include_directories(boreas_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(EthrcBetaflightSITLPlugin SHARED src/ethrc_betaflight_sitl_plugin.cpp)

target_compile_features(EthrcBetaflightSITLPlugin PUBLIC cxx_std_17)
target_link_libraries(EthrcBetaflightSITLPlugin
  boreas_dynamics
  yaml-cpp
  ${GZ_SIM_TARGET}
  ${GZ_PLUGIN_TARGET}
  ${GZ_MATH_TARGET}
  ${GZ_TRANSPORT_TARGET}
  ${GZ_MSGS_TARGET}
)

target_include_directories(EthrcBetaflightSITLPlugin PRIVATE include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_boreas_dynamics test/test_boreas_dynamics.cpp)
  target_link_libraries(test_boreas_dynamics boreas_dynamics)
endif()

install(TARGETS EthrcBetaflightSITLPlugin
  LIBRARY DESTINATION lib
)

install(TARGETS boreas_dynamics
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
