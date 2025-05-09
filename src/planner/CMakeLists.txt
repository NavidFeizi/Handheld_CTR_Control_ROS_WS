cmake_minimum_required(VERSION 3.22)
project(planner VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Option to enable/disable ASan
option(ENABLE_ASAN "Build with AddressSanitizer" OFF)

# Global or per-target application
if (ENABLE_ASAN)
  message(STATUS "Building WITH AddressSanitizer")
  add_compile_options(-fsanitize=address -g)       # compile flags
  add_link_options(-fsanitize=address)             # linker flags
else()
  message(STATUS "Building WITHOUT AddressSanitizer")
endif()  

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

# Define executable
add_executable(plan src/planner_node.cpp)

# Find necessary ros2 packages packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED) 
find_package(ament_index_cpp REQUIRED)
ament_target_dependencies(plan rclcpp std_msgs std_srvs interfaces ament_index_cpp)

# Find necessary planner packages
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Blaze REQUIRED)
find_package(TBB REQUIRED)
find_package(NLopt REQUIRED)      # For CTRlib
find_package(ompl REQUIRED)       # For CTR_MPL
find_package(fcl REQUIRED)        # For executable
find_package(assimp REQUIRED)     # For executable
find_package(pugixml REQUIRED)    # For executable
ament_target_dependencies(plan OpenMP Boost BLAS LAPACK Blaze TBB NLopt ompl fcl assimp pugixml)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
ament_target_dependencies(plan tf2 tf2_ros geometry_msgs tf2_eigen Eigen3)

# Adding the CTR static Library to the project
add_subdirectory(ctr_library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ctr_library/include)
target_link_libraries(plan CTRlib)

# Adding the motion_planner Library to the project
add_subdirectory(motion_planning)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/motion_planning/include)
target_link_libraries(plan CTR_MPL)

# OpenMP
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    add_compile_options(${OpenMP_CXX_FLAGS})
endif()

add_compile_definitions(OMP_NUM_THREADS=${OMP_NUM_THREADS})

install(TARGETS
plan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()