# ctr_mpc/CMakeLists.txt

project(CtrMpc 
        VERSION 1.0 
        LANGUAGES CXX)

cmake_minimum_required(VERSION 3.18)

# Toolchain / defaults
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # ensure PIC for all targets        

# ---- Dependencies ----
# find_package(LAPACK REQUIRED)
find_package(Blaze REQUIRED)          # provides blaze::blaze
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)    # provides OsqpEigen::OsqpEigen

add_library(CtrMpc INTERFACE)

target_include_directories(CtrMpc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Optimization flags for anything that links to CtrMpc
target_compile_options(CtrMpc INTERFACE
    $<$<CONFIG:Release>:-O3 -march=native  -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O3 -march=native  -g -DNDEBUG>)

# Link deps to the core library (PUBLIC so the Python module inherits)
target_link_libraries(CtrMpc
  INTERFACE
    # ${LAPACK_LIBRARIES}
    gfortran    # the linker search -lgfortran | order of linking matters
    blaze::blaze
    Eigen3::Eigen
    OsqpEigen::OsqpEigen)

