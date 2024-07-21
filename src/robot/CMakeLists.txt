cmake_minimum_required(VERSION 3.22)
project(robot)

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
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED) 

# Add SDL2
find_package(SDL2 REQUIRED)
if (SDL2_FOUND)
    message(STATUS ">>> SDL2 has been found!")
    include_directories(${SDL2_INCLUDE_DIRS})
    target_link_libraries(run ${SDL2_LIBRARIES})
else (NOT SDL2_FOUND)
    error(STATUS ">>> SDL2 has not been found")
endif()

add_subdirectory(lib_robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_robot/include)

add_executable(run src/robot_node.cpp)
target_link_libraries(run ROBOTlib)
ament_target_dependencies(run rclcpp std_msgs std_srvs interfaces)

install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()