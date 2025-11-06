cmake_minimum_required(VERSION 3.8)
project(aeris_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aeris_msgs REQUIRED)

add_executable(map_tile_publisher src/map_tile_publisher.cpp)
ament_target_dependencies(map_tile_publisher rclcpp aeris_msgs)

install(TARGETS map_tile_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
