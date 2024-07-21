cmake_minimum_required(VERSION 3.22)
project(publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED) 

# Define executables
# add_executable(simulator_cosserat src/Catheter_sim_node.cpp)
add_executable(simulator_koopman src/Koopman_model_node.cpp)
add_executable(publish src/publisher_node.cpp)
add_executable(record src/recorder_node.cpp)

# Sets and enables OpenMP parallel optimization in case OpenMP is available
find_package(OpenMP)
if(OpenMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# set(Boost_USE_STATIC_LIBS ON) # only find static libs
# set(Boost_USE_DEBUG_LIBS OFF) # ignore debug libs and
# set(Boost_USE_RELEASE_LIBS ON) # only find release libs
# set(Boost_USE_MULTITHREADED ON)
# set(Boost_USE_STATIC_RUNTIME ON)

# find_package(Boost REQUIRED)
# find_package(LAPACK REQUIRED)

# set(BLA_VENDOR OpenBLAS)
# set(BLA_STATIC ON) # use static linkage
# find_package(BLAS)

# # Check for the Boost libray, if found print message, include dirs and link libraries.
# if(Boost_FOUND)
#     message(STATUS "Using Boost version: ${Boost_VERSION}")
#     include_directories(${Boost_INCLUDE_DIRS})
#     link_libraries(${Boost_LIBRARIES})

#     if(WIN32)
#         # disable autolinking in boost
#         add_definitions(-DBOOST_ALL_NO_LIB)

#         # asks Boost to report information about automatic linking requests on Visual Studio
#         add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})

#         # force all boost libraries to dynamic link
#         add_definitions(-DBOOST_ALL_DYN_LINK)
#     endif()
# elseif(NOT Boost_FOUND)
#     error(STATUS "Boost has not been found!")
# endif()

# if(BLAS_FOUND)
#     message(STATUS "Using BLAS version: ${Blas_VERSION}")
#     include_directories(${BLAS_INCLUDE_DIRS})
#     target_link_libraries(simulator_cosserat ${BLAS_LIBRARIES})
# elseif(NOT BLAS_FOUND)
#     error(STATUS "BLAS library has not been found")
# endif()

# # Check for the LAPACK libray, if found print message, include dirs and link libraries.
# if(LAPACK_FOUND)
#     message(STATUS "Using LAPACK version: ${LAPACK_VERSION}")
#     include_directories(${LAPACK_INCLUDE_DIRS})
#     target_link_libraries(simulator_cosserat ${LAPACK_LIBRARIES})
# elseif(NOT LAPACK_FOUND)
#     error("LAPACK API has not been found!")
# endif()

# # Check for the Blaze libray, if found print message, include dirs and link libraries.
# find_package(blaze REQUIRED)
# if(blaze_FOUND)
#     message(STATUS "Using Blaze version: ${blaze_VERSION}")
#     target_link_libraries(simulator_cosserat blaze::blaze)
# elseif(NOT Blaze_FOUND)
#     error(STATUS "Blaze has not been found!")
# endif()

# # Check for the TBB libray. If found, print message, include dirs, and link libraries.
# find_package(TBB REQUIRED)
# if(TBB_FOUND)
#     message(STATUS "Using TBB version: ${TBB_VERSION}")
#     target_link_libraries(simulator_cosserat TBB::tbb)
# elseif(NOT TBB_FOUND)
#     error(STATUS "TBB has not been found!")
# endif()



# # Add subdirectories and link
# add_subdirectory(tendon_library)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tendon_library/include)
# target_link_libraries(simulator_cosserat TENDONDRIVENlib)
# ament_target_dependencies(simulator_cosserat rclcpp std_msgs interfaces)

# Add subdirectories and link
add_subdirectory(lib_predictor)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib_predictor/include)
target_link_libraries(simulator_koopman KOOPMANlib)
ament_target_dependencies(simulator_koopman rclcpp std_msgs interfaces)

ament_target_dependencies(publish rclcpp std_msgs interfaces)

ament_target_dependencies(record rclcpp std_msgs interfaces)

install(TARGETS
publish
record
# simulator_cosserat
simulator_koopman
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()