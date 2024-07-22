cmake_minimum_required(VERSION 3.22)
project(manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED) 

# Define executables
add_executable(mange src/manager_node.cpp)
add_executable(record src/recorder_node.cpp)

# Sets and enables OpenMP parallel optimization in case OpenMP is available
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ament_target_dependencies(mange rclcpp std_msgs rclcpp_action interfaces)

ament_target_dependencies(record rclcpp std_msgs interfaces)

install(TARGETS
mange
record
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()