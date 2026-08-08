# Create header-only (INTERFACE) library
add_library(CTR_MPL INTERFACE)

# Require C++17
target_compile_features(CTR_MPL INTERFACE cxx_std_17)

# Include directories (no trailing slash on BUILD_INTERFACE)
target_include_directories(CTR_MPL INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# OMPL configuration (find first, then link)
list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/ompl/cmake/")
find_package(ompl CONFIG REQUIRED)
if(OMPL_FOUND)
    message(STATUS "OMPL has been found!")
else()
    message(FATAL_ERROR "OMPL has not been found!")
endif()

# Link dependencies
target_link_libraries(CTR_MPL INTERFACE
    ctr_kinematics_pinn::ctr_kinematics_pinn
    Boost::serialization
    Boost::filesystem
    TBB::tbb
    ompl::ompl
)