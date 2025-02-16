# cmake_minimum_required(VERSION 3.22)
# set (CMAKE_CXX_STANDARD 14)

project(ROBOTlib VERSION 1.1.0)

# Set the release date manually
set(PROJECT_RELEASE_DATE "2025-02-11")

set(SRC_FILES 
    src/Robot.cpp
    src/CiA301node.cpp
    src/CanEssentials.cpp
	)

# Create a static library named ROBOTlib using the listed source files
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})

# Add the necessary include directories for the ROBOTlib library
target_include_directories(ROBOTlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

    
add_compile_options(-Wall -Wextra -pedantic)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") 

# Find the liblely-coapp package using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBLELY_COAPP REQUIRED liblely-coapp)

# Include directories from liblely-coapp package
include_directories(${LIBLELY_COAPP_INCLUDE_DIRS})

# add_executable(${PROJECT_NAME} master.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBLELY_COAPP_LIBRARIES})


set(CANopenFiles_directory ${CMAKE_BINARY_DIR}/CANopenFiles)
# Generate a C++ header file with the variable definition

# Pass the CANopenFiles_directory variable as a preprocessor definition
target_compile_definitions(${PROJECT_NAME} PRIVATE CANopenFiles_directory="${CANopenFiles_directory}")
# Pass the CANopenFiles_directory variable as a preprocessor definition
# target_compile_definitions(${PROJECT_NAME} PRIVATE EnoderStoreFiles_directory="${CMAKE_CURRENT_SOURCE_DIR}/config/")
target_compile_definitions(${PROJECT_NAME} PRIVATE EnoderStoreFiles_directory="$ENV{HOME}/Documents/handheld_CTR/encoder_memory/")
target_compile_definitions(${PROJECT_NAME} PRIVATE Log_directory="log/Robot/")


# Find and link Blaze package
set(blaze_DIR /usr/local/include/blaze)
find_package(blaze REQUIRED PATH ${blaze_DIR})
if( blaze_FOUND )
   message(STATUS ">>> Blaze library has been found!")
   target_link_libraries(${PROJECT_NAME} PRIVATE blaze::blaze)
elseif(NOT Blaze_FOUND)
    error(STATUS ">>> Blaze has not been found!")
endif()

find_package(spdlog REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)

# remove the cdf and bin files from the share directory
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E remove "${CANopenFiles_directory}/master.dcf"
                    COMMAND ${CMAKE_COMMAND} -E remove "${CANopenFiles_directory}/master.bin"
                    COMMENT "Removing previous .dcf and .bin files."
)

# make the dcf file from the yaml definition 
add_custom_command( TARGET ${PROJECT_NAME} 
                    POST_BUILD
                    COMMAND dcfgen -r master.yml
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config
                    COMMENT "Generating master.cdf and master.bin files."
)

# copy the needed files to build directory
add_custom_command( TARGET ${PROJECT_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/config/master.dcf" "${CANopenFiles_directory}/master.dcf"
                    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/config/master.bin" "${CANopenFiles_directory}/master.bin"
                    COMMENT "Copying master.dcf and master.bin files."
)

# remove the cdf and bin files from the share directory
add_custom_command( TARGET ${PROJECT_NAME}  
                    POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E remove "${CMAKE_CURRENT_SOURCE_DIR}/config/master.dcf"
                    COMMAND ${CMAKE_COMMAND} -E remove "${CMAKE_CURRENT_SOURCE_DIR}/config/master.bin"
                    COMMENT "Removing master.dcf and master.bin files from the source directory."
)
