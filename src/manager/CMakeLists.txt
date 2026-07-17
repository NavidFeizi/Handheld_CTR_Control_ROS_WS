cmake_minimum_required(VERSION 3.22)
project(manager VERSION 1.0 LANGUAGES CXX)

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

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED) 
find_package(ament_index_cpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent)

set(CMAKE_AUTOMOC ON)   # Enable automatic MOC processing

# Define and link executables
add_executable(manage src/manager_node.cpp)
ament_target_dependencies(manage rclcpp std_msgs rclcpp_action interfaces ament_index_cpp)

add_executable(record src/recorder_node.cpp)
ament_target_dependencies(record rclcpp std_msgs interfaces ament_index_cpp 
                            tf2 tf2_ros geometry_msgs tf2_eigen Eigen3)

add_executable(procedure src/procedure_node.cpp)
ament_target_dependencies(procedure rclcpp std_msgs interfaces ament_index_cpp)

add_executable(joint_path src/joint_path_node.cpp)
ament_target_dependencies(joint_path rclcpp std_msgs interfaces ament_index_cpp)

add_executable(master 
    src/master_node.cpp
    src/master_qt_gui.cpp
    include/manager/master_node.hpp
    include/manager/master_qt_gui.hpp)
target_include_directories(master PRIVATE include)
target_link_libraries(master Qt5::Widgets Qt5::Core Qt5::Concurrent)
ament_target_dependencies(master rclcpp std_msgs std_srvs interfaces ament_index_cpp 
                            tf2 tf2_ros geometry_msgs tf2_eigen Eigen3)

install(TARGETS
manage
record
procedure
joint_path
master
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()