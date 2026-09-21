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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_conventions test/test_joint_conventions.cpp)
  target_link_libraries(test_joint_conventions ctr_common)
  ament_add_gtest(test_csv_io test/test_csv_io.cpp)
  target_link_libraries(test_csv_io ctr_common)
  ament_add_gtest(test_diag_csv test/test_diag_csv.cpp)
  target_link_libraries(test_diag_csv ctr_common)
  ament_add_gtest(test_finite_guard test/test_finite_guard.cpp)
  target_link_libraries(test_finite_guard ctr_common)
  ament_add_gtest(test_home_pose test/test_home_pose.cpp)
  target_link_libraries(test_home_pose ctr_common)
endif()

ament_package()
