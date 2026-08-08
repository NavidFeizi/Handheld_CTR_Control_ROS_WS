cmake_minimum_required(VERSION 3.22)
project(emtracker VERSION 1.0 LANGUAGES CXX)

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
find_package(interfaces REQUIRED) 

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# EM tracker libraries
add_subdirectory(lib_emtracker)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_emtracker/include)

# open IGTLink libraries
set(OpenIGTLink_DIR "/usr/local/lib/igtl/cmake/igtl-3.1")
find_package(OpenIGTLink REQUIRED PATHS /usr/local)
include(${OpenIGTLink_USE_FILE})

add_executable(track src/EMtracker_node.cpp)
target_link_libraries(track EMTlib)
ament_target_dependencies(track rclcpp std_msgs std_srvs interfaces OpenIGTLink geometry_msgs tf2 tf2_ros ament_index_cpp)

install(TARGETS
  track
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Read-only seeds (config.yaml, SROMs, landmark/transform CSVs, beeps). At
# runtime the node copies them once into the writable emtracker_data_dir.
install(DIRECTORY
  lib_emtracker/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Node parameters (kept out of config/ so the seed copy stays pure).
install(FILES
  config/emtracker_params.yaml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_butterworth test/test_butterworth.cpp)
  target_include_directories(test_butterworth PRIVATE lib_emtracker/include)
  ament_add_gtest(test_rigid_transformation test/test_rigid_transformation.cpp)
  target_include_directories(test_rigid_transformation PRIVATE lib_emtracker/include)
  target_link_libraries(test_rigid_transformation EMTlib)
endif()

ament_package()