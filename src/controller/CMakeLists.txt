cmake_minimum_required(VERSION 3.22)
project(controller VERSION 1.0 LANGUAGES CXX)

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
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED) # Make sure this package correctly exports its dependencies

# Define executable
add_executable(control src/controller_node.cpp)

ament_target_dependencies(control rclcpp std_msgs rclcpp_action interfaces)

# Adding the CTR static Library to the project
add_subdirectory(ctr_library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ctr_library/include)
target_link_libraries(control CTRlib)

# OpenMP
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    add_compile_options(${OpenMP_CXX_FLAGS})
endif()

# Set number of threads for OpenBLAS
if(DEFINED ENV{OPENBLAS_NUM_THREADS})
    set(OPENBLAS_NUM_THREADS $ENV{OPENBLAS_NUM_THREADS})
else()
    set(OPENBLAS_NUM_THREADS 64) # Default to 64 if not set
endif()

# Optionally, set the number of OpenMP threads
if(DEFINED ENV{OMP_NUM_THREADS})
    set(OMP_NUM_THREADS $ENV{OMP_NUM_THREADS})
else()
    set(OMP_NUM_THREADS 4) # Default to 4 if not set
endif()

add_compile_definitions(OMP_NUM_THREADS=${OMP_NUM_THREADS})

install(TARGETS
control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()