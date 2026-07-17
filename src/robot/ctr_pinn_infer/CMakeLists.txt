# src/cpp/ctr_pinn_infer/CMakeLists.txt

project(CtrPinnInference 
        VERSION 1.2 
        LANGUAGES CXX)

add_library(CtrPinnInference INTERFACE)

target_include_directories(CtrPinnInference INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Find the Torch package
# set(Torch_DIR "/usr/local/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
find_package(Blaze REQUIRED)
find_package(nlohmann_json REQUIRED)  

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

target_link_libraries(${PROJECT_NAME} 
    INTERFACE 
        ${LAPACK_LIBRARIES}
        blaze::blaze
        ${TORCH_LIBRARIES}
        nlohmann_json::nlohmann_json
)


target_compile_definitions(${PROJECT_NAME} INTERFACE models_Directory="${CMAKE_CURRENT_SOURCE_DIR}/models")


##### complie optimization  
# Adding optimization flags
include(CheckCXXCompilerFlag)

# Check for -O3 support
check_cxx_compiler_flag("-O3" COMPILER_OPT_O3_SUPPORTED)
if(COMPILER_OPT_O3_SUPPORTED)
    message(STATUS ">>> COMPILER_OPT_O3_SUPPORTED")
    target_compile_options(${PROJECT_NAME} INTERFACE -O3)
endif()


# # Check for -march=native support
check_cxx_compiler_flag("-mtune=native" COMPILER_OPT_MTUNE_NATIVE_SUPPORTED)
if(COMPILER_OPT_MTUNE_NATIVE_SUPPORTED)
    message(STATUS ">>> COMPILER_OPT_MTUNE_NATIVE_SUPPORTED")
    target_compile_options(${PROJECT_NAME} INTERFACE -mtune=native)
endif()



# Allow installation
install(TARGETS CtrPinnInference
        EXPORT CtrPinnInferenceTargets
        DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

# Generate a Config.cmake for downstream projects
install(EXPORT CtrPinnInferenceTargets
        FILE CtrPinnInferenceTargets.cmake
        NAMESPACE CtrPinnInference::
        DESTINATION lib/cmake/CtrPinnInference
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/CtrPinnInferenceConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/CtrPinnInferenceConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CtrPinnInferenceConfig.cmake
    INSTALL_DESTINATION lib/cmake/CtrPinnInference
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/CtrPinnInferenceConfig.cmake
    DESTINATION lib/cmake/CtrPinnInference
)
