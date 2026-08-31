cmake_minimum_required(VERSION 3.22)
project(ctr_cosserat VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(NLopt REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

add_library(ctr_cosserat STATIC
  src/CTR.cpp
  src/Observer.cpp
  src/ODESystem.cpp
  src/Segment.cpp
  src/Tube.cpp
)
add_library(ctr_cosserat::ctr_cosserat ALIAS ctr_cosserat)

target_include_directories(ctr_cosserat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(ctr_cosserat
  PRIVATE
    NLopt::nlopt
    m
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    gfortran
)

# Opt-in PINN-vs-Cosserat FK cross-check (benchmark/fk_xcheck.cpp). OFF by
# default: it links LibTorch (through ctr_kinematics_pinn) purely as a
# diagnostic tool; the exported ctr_cosserat library stays Torch-free.
option(CTR_COSSERAT_BUILD_XCHECK "Build the PINN-vs-Cosserat FK cross-check tool" OFF)
if(CTR_COSSERAT_BUILD_XCHECK)
  find_package(ctr_kinematics_pinn REQUIRED)  # re-runs the Torch pin (CONFIG_EXTRAS)
  find_package(Blaze REQUIRED)
  find_package(nlohmann_json REQUIRED)
  add_executable(fk_xcheck benchmark/fk_xcheck.cpp)
  target_link_libraries(fk_xcheck PRIVATE ctr_cosserat ctr_kinematics_pinn::ctr_kinematics_pinn)
  set_target_properties(fk_xcheck PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ctr_cosserat
  EXPORT export_ctr_cosserat
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_ctr_cosserat HAS_LIBRARY_TARGET)
# STATIC-lib PRIVATE deps surface in the export as $<LINK_ONLY:NLopt::nlopt> etc.;
# consumers must find_package them before the export file loads.
ament_export_dependencies(NLopt BLAS LAPACK)

ament_package()
