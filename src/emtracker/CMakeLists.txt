cmake_minimum_required(VERSION 3.22)
project(emtracker)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED) 

# EM tracker libraries
add_subdirectory(lib_emtracker)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_emtracker/include)

# open IGTLink libraries
set(OpenIGTLink_DIR "/usr/local/lib/igtl/cmake/igtl-3.1")
find_package(OpenIGTLink REQUIRED PATHS /usr/local)
include(${OpenIGTLink_USE_FILE})

add_executable(track src/EMtracker_node.cpp)
target_link_libraries(track EMTlib)
ament_target_dependencies(track rclcpp std_msgs interfaces OpenIGTLink)

install(TARGETS
  track
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()