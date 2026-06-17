cmake_minimum_required(VERSION 3.8)
project(sitl_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim QUIET)

if(gz-sim_FOUND)
  find_package(gz-plugin REQUIRED)
  find_package(gz-math REQUIRED)
  set(GZ_SIM_TARGET gz-sim::gz-sim)
  set(GZ_PLUGIN_TARGET gz-plugin::gz-plugin)
  set(GZ_MATH_TARGET gz-math::gz-math)
else()
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED)
  find_package(gz-math7 REQUIRED)
  set(GZ_SIM_TARGET gz-sim8::gz-sim8)
  set(GZ_PLUGIN_TARGET gz-plugin2::gz-plugin2)
  set(GZ_MATH_TARGET gz-math7::gz-math7)
endif()

add_library(EthrcBetaflightSITLPlugin SHARED
  src/ethrc_betaflight_sitl_plugin.cpp
)

target_compile_features(EthrcBetaflightSITLPlugin PUBLIC cxx_std_17)
target_link_libraries(EthrcBetaflightSITLPlugin
  ${GZ_SIM_TARGET}
  ${GZ_PLUGIN_TARGET}
  ${GZ_MATH_TARGET}
)

install(TARGETS EthrcBetaflightSITLPlugin
  LIBRARY DESTINATION lib
)

ament_package()
