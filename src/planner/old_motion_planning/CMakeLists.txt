# Source files
set(MOTION_PLANNING_LIB_SOURCES
    src/CTR_BackboneLengthObjective.cpp
    src/CTR_ClearanceObjective.cpp
    src/CTR_DiscreteMotionValidator.cpp
    src/CTR_StateSampler.cpp
    src/CTR_StateValidityChecker.cpp
    src/Planner.cpp
)

# Static library
add_library(CTR_MPL STATIC ${MOTION_PLANNING_LIB_SOURCES})

# Include directories
target_include_directories(CTR_MPL
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies (PUBLIC to propagate includes)
target_link_libraries(CTR_MPL
    PUBLIC
        CTRlib    # Public if headers depend on CTRlib
        Boost::serialization
        Boost::filesystem
        Boost::system
    PRIVATE
        TBB::tbb
)

# OMPL as imported target
list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/ompl/cmake/")
find_package(ompl CONFIG REQUIRED)
if(OMPL_FOUND)
    message(STATUS "OMPL has been found!")
    # target_include_directories(CTR_MPL PUBLIC ${OMPL_INCLUDE_DIRS})
    # target_link_libraries(CTR_MPL PUBLIC ${OMPL_LIBRARIES})
    target_link_libraries(CTR_MPL PUBLIC ompl::ompl)
else()
    message(FATAL_ERROR "OMPL has not been found!")
endif()

# Propagate C++17 requirement
target_compile_features(CTR_MPL PUBLIC cxx_std_17)