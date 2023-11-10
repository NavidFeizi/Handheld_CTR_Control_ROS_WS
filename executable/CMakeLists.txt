add_executable(${PROJECT_NAME} Main.cpp) # ${CMAKE_SOURCE_DIR}/ndi_api/src/UdpSocket.cpp)

# include header files' directory
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC CTR TRACK ROBOTlib)

# Check for the TBB libray. If found, print message, include dirs, and link libraries.
find_package(TBB REQUIRED)

if(TBB_FOUND)
    message(STATUS "TBB has been found!")
    target_link_libraries(${PROJECT_NAME} PUBLIC TBB::tbb)
elseif(NOT TBB_FOUND)
    error(STATUS "TBB has not been found!")
endif()