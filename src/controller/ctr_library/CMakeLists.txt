# Define project name and version
project(CTRlib VERSION 1.0 LANGUAGES CXX)

# List the CTR library source files
set(CTR_LIB_SOURCES
    src/collisionCheck.cpp
    src/CTR.cpp
    src/Observer.cpp
    src/ODESystem.cpp
    src/Segment.cpp
    src/Tube.cpp
)

# Create the CTR static library
add_library(CTRlib STATIC ${CTR_LIB_SOURCES})

# Include directories for the CTR library
target_include_directories(CTRlib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link NLOPT and math libraries
target_link_libraries(CTRlib
    PRIVATE
        nlopt
        m
)

# Find VTK
set(VTK_DIR "/usr/local/lib/")
find_package(VTK REQUIRED COMPONENTS CommonCore FiltersCore HINTS ${VTK_DIR})

if(VTK_FOUND)
    message(STATUS "VTK has been found!")
    # target_link_libraries(CTRlib PUBLIC ${VTK_LIBRARIES} vtkIOCore)
    target_link_libraries(CTRlib PUBLIC ${VTK_LIBRARIES})
else()
    message(FATAL_ERROR "VTK library has not been found")
endif()

# Find Eigen3
find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
    message(STATUS "Eigen3 has been found!")
    target_link_libraries(CTRlib PUBLIC Eigen3::Eigen)
else()
    message(FATAL_ERROR "Eigen3 has not been found!")
endif()

# Find FCL
find_package(fcl REQUIRED)

if(fcl_FOUND)
    message(STATUS "FCL has been found!")
    include_directories(${FCL_INCLUDE_DIRS})
    target_link_libraries(CTRlib PUBLIC ${FCL_LIBRARIES})
else()
    message(FATAL_ERROR "FCL has not been found!")
endif()