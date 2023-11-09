cmake_minimum_required(VERSION 3.20)
project(Handheld_CTR_Control VERSION 2.1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Generated binary files will be directed to "bin" folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# addind the library and main executable directories to the project tree
add_subdirectory(ctr_library) # static CTR Library
# add_subdirectory(ndi_api) # NDI API as dynamic/shared library
# add_subdirectory(em_tracker) # static EM Tracker library
add_subdirectory(executable) # main executable
add_subdirectory(lib_robot)         # static robot 
add_subdirectory(lib_emtracker)     # static emtracker library
add_subdirectory(lib_keyinput)     # static emtracker library

# LAPACK libraries
find_package(LAPACK REQUIRED)

# Blaze library
find_package(blaze CONFIG REQUIRED)

# Boost Libraries
set(Boost_USE_STATIC_LIBS ON) # only find static libs
set(Boost_USE_DEBUG_LIBS OFF) # ignore debug libs and
set(Boost_USE_RELEASE_LIBS ON) # only find release libs
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED)

# # BLAS libraries
# set(BLA_VENDOR OpenBLAS)

# # set(BLA_STATIC                   ON)  # use static linkage
# find_package(OpenBLAS CONFIG REQUIRED)

set(BLA_VENDOR OpenBLAS)
set(BLA_STATIC                   ON)  # use static linkage
find_package(BLAS)

# Check for the Boost libray, if found print message, include dirs and link libraries.
if(Boost_FOUND)
    message(STATUS "Boost has been found!")
    include_directories(${Boost_INCLUDE_DIRS})
    link_libraries(${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
    error(STATUS "Boost has not been found!")
endif(Boost_FOUND)

# if(OpenBLAS_FOUND)
#     message(STATUS "OpenBLAS has been found!")
#     include_directories(${OpenBLAS_INCLUDE_DIRS})
#     target_link_libraries(${PROJECT_NAME} PRIVATE OpenBLAS::OpenBLAS)
#     target_link_libraries(TRACK PRIVATE OpenBLAS::OpenBLAS)
# elseif(NOT OpenBLAS_FOUND)
#     error(STATUS "openBLAS library has not been found")
# endif(OpenBLAS_FOUND)

if (BLAS_FOUND)
    message(STATUS "BLAS has been found!")
    include_directories(${BLAS_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${BLAS_LIBRARIES})
elseif(NOT BLAS_FOUND)
    error(STATUS "BLAS library has not been found")
endif()


# Check for the LAPACK libray, if found print message, include dirs and link libraries.
if(LAPACK_FOUND)
    message("LAPACK has been found!")
    include_directories(${LAPACK_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LAPACK_LIBRARIES})
    target_link_libraries(TRACK PRIVATE ${LAPACK_LIBRARIES})
elseif(NOT LAPACK_FOUND)
    error("LAPACK API has not been found")
endif(LAPACK_FOUND)

# Check for the Blaze libray, if found print message, include dirs and link libraries.
if(blaze_FOUND)
    message(STATUS "Blaze has been found!")
    target_link_libraries(${PROJECT_NAME} INTERFACE blaze::blaze)
    target_link_libraries(TRACK INTERFACE blaze::blaze)
elseif(NOT Blaze_FOUND)
    error(STATUS "Blaze has not been found!")
endif(blaze_FOUND)

# Check for the TBB libray. If found, print message, include dirs, and link libraries.
# find_package(TBB REQUIRED)
# if( TBB_FOUND )
# message(STATUS "TBB has been found!")
# target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
# elseif(NOT TBB_FOUND)
# error(STATUS "TBB has not been found!")
# endif()

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)