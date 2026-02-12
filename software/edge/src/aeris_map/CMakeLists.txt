cmake_minimum_required(VERSION 3.8)
project(aeris_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aeris_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PNG REQUIRED)
find_package(SQLite3 REQUIRED)

add_library(tile_contract src/tile_contract.cpp)
target_include_directories(tile_contract PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(tile_contract PUBLIC cxx_std_17)
ament_target_dependencies(tile_contract aeris_msgs)
target_link_libraries(tile_contract OpenSSL::Crypto)

add_library(slam_backend src/slam_backend.cpp)
target_include_directories(slam_backend PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(slam_backend PUBLIC cxx_std_17)

add_executable(map_tile_publisher src/map_tile_publisher.cpp)
target_include_directories(map_tile_publisher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(map_tile_publisher rclcpp aeris_msgs nav_msgs sensor_msgs)
target_link_libraries(map_tile_publisher tile_contract slam_backend PNG::PNG SQLite::SQLite3)

install(TARGETS
  map_tile_publisher
  tile_contract
  slam_backend
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tile_contract test/test_tile_contract.cpp)
  if(TARGET test_tile_contract)
    target_include_directories(test_tile_contract PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_tile_contract tile_contract)
    ament_target_dependencies(test_tile_contract aeris_msgs)
  endif()

  ament_add_gtest(test_slam_backend test/test_slam_backend.cpp)
  if(TARGET test_slam_backend)
    target_include_directories(test_slam_backend PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_slam_backend slam_backend)
  endif()

  ament_add_gtest(test_map_tile_publisher test/test_map_tile_publisher.cpp)
  if(TARGET test_map_tile_publisher)
    target_include_directories(test_map_tile_publisher PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_features(test_map_tile_publisher PRIVATE cxx_std_17)
    target_link_libraries(
      test_map_tile_publisher
      tile_contract
      slam_backend
      PNG::PNG
      SQLite::SQLite3
    )
    ament_target_dependencies(
      test_map_tile_publisher
      rclcpp
      aeris_msgs
      nav_msgs
      sensor_msgs
    )
  endif()
endif()

ament_package()
