cmake_minimum_required(VERSION 3.22)
project(ctr_kinematics_pinn VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(ament_cmake REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/torch_pin.cmake)
find_package(Blaze REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(LAPACK REQUIRED)

add_library(ctr_kinematics_pinn INTERFACE)
add_library(ctr_kinematics_pinn::ctr_kinematics_pinn ALIAS ctr_kinematics_pinn)

target_include_directories(ctr_kinematics_pinn INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(ctr_kinematics_pinn INTERFACE
  ${LAPACK_LIBRARIES}
  blaze::blaze
  ${TORCH_LIBRARIES}
  nlohmann_json::nlohmann_json)

separate_arguments(_torch_cxx_flags UNIX_COMMAND "${TORCH_CXX_FLAGS}")
target_compile_options(ctr_kinematics_pinn INTERFACE ${_torch_cxx_flags} -O3)
unset(_torch_cxx_flags)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(TARGETS ctr_kinematics_pinn
  EXPORT export_ctr_kinematics_pinn)

ament_export_targets(export_ctr_kinematics_pinn HAS_LIBRARY_TARGET)
# The exported link interface names blaze::blaze / nlohmann_json::nlohmann_json;
# consumers must find_package them before the export file loads.
ament_export_dependencies(Blaze nlohmann_json LAPACK)

# Downstream find_package(ctr_kinematics_pinn) re-runs the Torch pin so
# TORCH_LIBRARIES/imported torch targets resolve identically for consumers.
ament_package(CONFIG_EXTRAS cmake/torch_pin.cmake)
