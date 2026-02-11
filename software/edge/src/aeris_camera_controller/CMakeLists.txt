# CMake build configuration for the Aeris camera controller Gazebo plugin.
# Builds a ROS 2 service-enabled plugin for cinematic camera control in simulation.
cmake_minimum_required(VERSION 3.16)
project(aeris_camera_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aeris_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-transport11 REQUIRED)

set(plugin_sources
  src/CameraControllerPlugin.cpp
)

add_library(${PROJECT_NAME} SHARED ${plugin_sources})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  aeris_msgs
  ignition-gazebo6
  ignition-math6
  ignition-transport11)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp aeris_msgs ignition-gazebo6 ignition-math6 ignition-transport11)

ament_package()
