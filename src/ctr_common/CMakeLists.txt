cmake_minimum_required(VERSION 3.22)
project(ctr_common VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(ctr_common src/runtime_paths.cpp)
add_library(ctr_common::ctr_common ALIAS ctr_common)
target_include_directories(ctr_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ctr_common PUBLIC rclcpp ament_index_cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ctr_common
  EXPORT export_ctr_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_ctr_common HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ament_index_cpp)

ament_package()
