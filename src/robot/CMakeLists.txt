cmake_minimum_required(VERSION 3.5)
project(robot VERSION 2.0 LANGUAGES CXX)

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
find_package(ctr_cosserat REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent)

# Find necessary planner packages
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Blaze REQUIRED)
find_package(TBB REQUIRED)
find_package(NLopt REQUIRED)      # For cosserat_fk

# Find necessary pinn packages
find_package(Torch REQUIRED)
find_package(nlohmann_json REQUIRED)  

set(CMAKE_AUTOMOC ON)   # Enable automatic MOC processing

add_subdirectory(lib_robot)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_robot/include)

# Adding the CTR pinn library to the project
add_subdirectory(ctr_pinn_infer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ctr_pinn_infer/include)

add_executable(ctr_robot src/robot_node.cpp)
target_link_libraries(ctr_robot ROBOTlib Qt5::Widgets Qt5::Core Qt5::Concurrent)
ament_target_dependencies(ctr_robot rclcpp std_msgs std_srvs rclcpp_action interfaces)

add_executable(qt_gui src/qt_node.cpp)
target_link_libraries(qt_gui Qt5::Widgets Qt5::Core Qt5::Concurrent)
ament_target_dependencies(qt_gui rclcpp std_msgs std_srvs rclcpp_action interfaces)

add_executable(cosserat_fk src/cosserat_fk_node.cpp)
target_link_libraries(cosserat_fk ctr_cosserat::ctr_cosserat)
ament_target_dependencies(cosserat_fk rclcpp std_msgs std_srvs interfaces)
ament_target_dependencies(cosserat_fk OpenMP Boost BLAS LAPACK Blaze TBB NLopt)

add_executable(pinn_fk src/pinn_fk_node.cpp)
target_link_libraries(pinn_fk CtrPinnInference)
ament_target_dependencies(pinn_fk rclcpp std_msgs std_srvs interfaces)
ament_target_dependencies(pinn_fk LAPACK Blaze nlohmann_json Torch)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node CtrPinnInference)
ament_target_dependencies(ekf_node rclcpp std_msgs std_srvs interfaces)
ament_target_dependencies(ekf_node LAPACK Blaze nlohmann_json Torch)

install(TARGETS
  ctr_robot
  qt_gui
  cosserat_fk
  pinn_fk
  ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()