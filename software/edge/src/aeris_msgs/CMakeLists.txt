# Build configuration for aeris_msgs ROS 2 interface package.
# Defines message, service, and action types for Aeris drone ecosystem.
cmake_minimum_required(VERSION 3.8)
project(aeris_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

set(interface_files
  "msg/GeoPosition.msg"
  "msg/VehicleOrientation.msg"
  "msg/Telemetry.msg"
  "msg/MapTile.msg"
  "msg/ThermalHotspot.msg"
  "msg/AcousticBearing.msg"
  "msg/GasIsopleth.msg"
  "msg/FusedDetection.msg"
  "msg/CameraWaypoint.msg"
  "msg/MissionState.msg"
  "msg/MissionProgress.msg"
  "srv/MapTile.srv"
  "srv/GetMapTileBytes.srv"
  "srv/ThermalHotspot.srv"
  "srv/AcousticBearing.srv"
  "srv/GasIsopleth.srv"
  "srv/SetCameraView.srv"
  "srv/MissionCommand.srv"
  "srv/VehicleCommand.srv"
  "action/MapTile.action"
  "action/ThermalHotspot.action"
  "action/AcousticBearing.action"
  "action/GasIsopleth.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
