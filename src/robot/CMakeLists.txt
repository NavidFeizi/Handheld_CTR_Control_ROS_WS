cmake_minimum_required(VERSION 3.22)
project(robot VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set the default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set RPATH to avoid library conflicts
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED) 

add_subdirectory(lib_robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_robot/include)

add_executable(ctr_robot src/robot_node.cpp)
target_link_libraries(ctr_robot ROBOTlib)
ament_target_dependencies(ctr_robot rclcpp std_msgs std_srvs rclcpp_action interfaces)

add_executable(key_input src/keyboard_node.cpp)
ament_target_dependencies(key_input rclcpp std_msgs interfaces)

# Add SDL2
find_package(SDL2 REQUIRED)
if (SDL2_FOUND)
    message(STATUS ">>> SDL2 has been found!")
    include_directories(${SDL2_INCLUDE_DIRS})
    target_link_libraries(key_input ${SDL2_LIBRARIES})
else (NOT SDL2_FOUND)
    error(STATUS ">>> SDL2 has not been found")
endif()

install(TARGETS
  ctr_robot
  key_input
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()