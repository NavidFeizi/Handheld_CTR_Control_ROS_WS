cmake_minimum_required(VERSION 3.22)
project(ctr_robot_driver VERSION 1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBLELY_COAPP REQUIRED liblely-coapp)

add_library(ctr_robot_driver STATIC
  src/Robot.cpp
  src/CiA301node.cpp
  src/CanEssentials.cpp
)
add_library(ctr_robot_driver::ctr_robot_driver ALIAS ctr_robot_driver)

target_include_directories(ctr_robot_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(ctr_robot_driver PRIVATE ${LIBLELY_COAPP_INCLUDE_DIRS})

# Plain paths (not the PkgConfig imported target) so the exported link
# interface of this static lib stays resolvable for downstream packages.
target_link_libraries(ctr_robot_driver
  PRIVATE
    ${LIBLELY_COAPP_LINK_LIBRARIES}
    spdlog::spdlog
)

target_compile_options(ctr_robot_driver PRIVATE -Wall -Wextra -pedantic)

# --- CANopen master configuration -------------------------------------------
# dcfgen generates master.dcf/master.bin from config/master.yml in the BUILD
# tree, only when the inputs change (the old POST_BUILD version regenerated on
# every build and wrote into the source tree). Generated files and their inputs
# are installed to share/ and read from there at runtime via the canopen_dir
# runtime path (see CTRobot::RuntimePaths).
set(CANOPEN_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/canopen)
file(MAKE_DIRECTORY ${CANOPEN_GEN_DIR})
add_custom_command(
  OUTPUT ${CANOPEN_GEN_DIR}/master.dcf ${CANOPEN_GEN_DIR}/master.bin
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_SOURCE_DIR}/config/master.yml
          ${CMAKE_CURRENT_SOURCE_DIR}/config/605.0141.01-L.eds
          ${CANOPEN_GEN_DIR}/
  COMMAND dcfgen -r master.yml
  WORKING_DIRECTORY ${CANOPEN_GEN_DIR}
  DEPENDS config/master.yml config/605.0141.01-L.eds
  COMMENT "Generating CANopen master.dcf/master.bin with dcfgen"
  VERBATIM
)
add_custom_target(canopen_dcf ALL
  DEPENDS ${CANOPEN_GEN_DIR}/master.dcf ${CANOPEN_GEN_DIR}/master.bin)

install(FILES
  ${CANOPEN_GEN_DIR}/master.dcf
  ${CANOPEN_GEN_DIR}/master.bin
  ${CMAKE_CURRENT_SOURCE_DIR}/config/master.yml
  ${CMAKE_CURRENT_SOURCE_DIR}/config/605.0141.01-L.eds
  DESTINATION share/${PROJECT_NAME}/canopen)
# -----------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ctr_robot_driver
  EXPORT export_ctr_robot_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_ctr_robot_driver HAS_LIBRARY_TARGET)
# STATIC-lib PRIVATE deps surface in the export as $<LINK_ONLY:spdlog::spdlog>;
# consumers must find_package it before the export file loads.
ament_export_dependencies(spdlog)

ament_package()
