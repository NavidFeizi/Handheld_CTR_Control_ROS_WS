project(KOOPMANlib)

# List the EMT library source files into a variable
file(GLOB KOOPMAN_LIB_SOURCES 
    "src/*.cpp")

# Create a static library named EMTlib using the listed source files
add_library(${PROJECT_NAME} STATIC ${KOOPMAN_LIB_SOURCES})

# Add the necessary include directories for the EMTlib library
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Find the Torch package
set(Torch_DIR "/usr/local/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
if (Torch_FOUND)
    message(STATUS ">>> Torch library has been found!")
    target_link_libraries(${PROJECT_NAME} PRIVATE "${TORCH_LIBRARIES}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
else (NOT Torch_FOUND)
    error(STATUS ">>> Torch library has not been found")
endif()

# Check for the LAPACK libray, if found print message, include dirs and link libraries.
find_package(LAPACK REQUIRED)
if(LAPACK_FOUND)
    message(">>> LAPACK has been found!")
    include_directories(${LAPACK_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LAPACK_LIBRARIES})
elseif(NOT LAPACK_FOUND)
    error("LAPACK API has not been found")
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE model_Directory="${CMAKE_CURRENT_SOURCE_DIR}/model")


##### complie optimization  
# Adding optimization flags
include(CheckCXXCompilerFlag)

# Check for -O3 support
check_cxx_compiler_flag("-O3" COMPILER_OPT_O3_SUPPORTED)
if(COMPILER_OPT_O3_SUPPORTED)
    message(STATUS ">>> COMPILER_OPT_O3_SUPPORTED")
    target_compile_options(${PROJECT_NAME} PRIVATE -O3)
endif()

# # Check for -march=native support
check_cxx_compiler_flag("-mtune=native" COMPILER_OPT_MTUNE_NATIVE_SUPPORTED)
if(COMPILER_OPT_MTUNE_NATIVE_SUPPORTED)
    message(STATUS ">>> COMPILER_OPT_MTUNE_NATIVE_SUPPORTED")
    target_compile_options(${PROJECT_NAME} PRIVATE -mtune=native)
endif()

