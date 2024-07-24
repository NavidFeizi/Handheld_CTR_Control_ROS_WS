# Define project name and version
project(CTRlib VERSION 1.0 LANGUAGES CXX)

# Set the CMP0167 policy to find the upstream BoostConfig.cmake directly
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# List the CTR library source files
set(CTR_LIB_SOURCES
    # src/collisionCheck.cpp
    # src/collisionCheck.cpp
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

# Boost
find_package(Boost REQUIRED)

if(Boost_FOUND)
    message(STATUS "Boost found")
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost not found")
endif()

# BLAS
set(BLA_VENDOR OpenBLAS)
set(BLA_STATIC ON)
find_package(BLAS REQUIRED)

if(BLAS_FOUND)
    message(STATUS "BLAS found")
else()
    message(FATAL_ERROR "BLAS not found")
endif()

# LAPACK
find_package(LAPACK REQUIRED)

if(LAPACK_FOUND)
    message(STATUS "LAPACK found")
else()
    message(FATAL_ERROR "LAPACK not found")
endif()

# Blaze
find_package(Blaze REQUIRED)

if(Blaze_FOUND)
    message(STATUS "Blaze found")
    target_link_libraries(${PROJECT_NAME} INTERFACE blaze::blaze)
else()
    message(FATAL_ERROR "Blaze not found")
endif()

# TBB
find_package(TBB REQUIRED)

if(TBB_FOUND)
    message(STATUS "TBB found")
    target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
else()
    message(FATAL_ERROR "TBB not found")
endif()

# OpenMP
find_package(OpenMP REQUIRED)

if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    add_compile_options(${OpenMP_CXX_FLAGS})
endif()

# Set number of threads for OpenBLAS
if(DEFINED ENV{OPENBLAS_NUM_THREADS})
    set(OPENBLAS_NUM_THREADS $ENV{OPENBLAS_NUM_THREADS})
else()
    set(OPENBLAS_NUM_THREADS 64) # Default to 64 if not set
endif()

# Add the environment variable for OpenBLAS
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS}")

# Optionally, set the number of OpenMP threads
if(DEFINED ENV{OMP_NUM_THREADS})
    set(OMP_NUM_THREADS $ENV{OMP_NUM_THREADS})
else()
    set(OMP_NUM_THREADS 4) # Default to 4 if not set
endif()

add_compile_definitions(OMP_NUM_THREADS=${OMP_NUM_THREADS})

# # PugiXML
# set(pugi_DIR "/usr/lib/cmake/pugixml/")
# find_package(pugixml REQUIRED HINTS pugi_DIR)

# if(pugixml_FOUND)
#     message(STATUS "PugiXML found")
#     include_directories(${pugixml_INCLUDE_DIRS})
# else()
#     message(FATAL_ERROR "PugiXML not found")
# endif()

# # Assimp
# find_package(assimp REQUIRED)

# if(assimp_FOUND)
#     message(STATUS "Assimp found")
#     include_directories(${ASSIMP_INCLUDE_DIRS})
#     target_link_libraries(${PROJECT_NAME} PRIVATE assimp::assimp)
# else()
#     message(FATAL_ERROR "Assimp not found")
# endif()

# Link all necessary libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${Boost_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    # ${pugixml_LIBRARIES}
)

# linking gfortran library
target_link_libraries(${PROJECT_NAME} PRIVATE gfortran)

# Optionally enable CPack if needed
# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)











# # Find VTK
# set(VTK_DIR "/usr/local/lib/")
# find_package(VTK REQUIRED COMPONENTS CommonCore FiltersCore HINTS ${VTK_DIR})

# if(VTK_FOUND)
#     message(STATUS "VTK has been found!")
#     # target_link_libraries(CTRlib PUBLIC ${VTK_LIBRARIES} vtkIOCore)
#     target_link_libraries(CTRlib PUBLIC ${VTK_LIBRARIES})
# else()
#     message(FATAL_ERROR "VTK library has not been found")
# endif()

# # Find Eigen3
# find_package(Eigen3 REQUIRED)

# if(Eigen3_FOUND)
#     message(STATUS "Eigen3 has been found!")
#     target_link_libraries(CTRlib PUBLIC Eigen3::Eigen)
# else()
#     message(FATAL_ERROR "Eigen3 has not been found!")
# endif()

# # Find FCL
# find_package(fcl REQUIRED)
# Find FCL
# find_package(fcl REQUIRED)

# if(fcl_FOUND)
#     message(STATUS "FCL has been found!")
#     include_directories(${FCL_INCLUDE_DIRS})
#     target_link_libraries(CTRlib PUBLIC ${FCL_LIBRARIES})
# else()
#     message(FATAL_ERROR "FCL has not been found!")
# endif()