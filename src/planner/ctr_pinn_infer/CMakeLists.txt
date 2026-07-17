# src/cpp/PINNs/CMakeLists.txt

project(CtrPinnInference 
        VERSION 1.1 
        LANGUAGES CXX)

add_library(CtrPinnInference INTERFACE)

target_include_directories(CtrPinnInference INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Find the Torch package.
#
# Problem: stale headers in /usr/local/include/ (ATen/, c10/, caffe2/, …) from
# an older manual LibTorch install shadow both the system 2.10.0 headers at
# /usr/include/ AND the self-contained 2.9.0 headers at /usr/local/libtorch/.
# GCC's implicit include search order is: /usr/local/include THEN /usr/include,
# so the stale files are always found first and their header guard
# C10_UTIL_OPTIONAL_H_ fires without #include <optional>, causing:
#   • 'std::optional' has not been declared            (variable.h:390)
#   • ambiguous overload for operator<<                (c10::BFloat16)
#   • could not convert std::optional to c10::optional (torch/fft.h)
#   • too many arguments to torch::autograd::grad      (PINNsence.hpp)
#
# Fix: force Torch_DIR to the self-contained libtorch 2.9.0.
#   CMake propagates TORCH_INCLUDE_DIRS (=/usr/local/libtorch/include;...) as
#   explicit -isystem flags.  GCC searches -isystem entries BEFORE its built-in
#   system directories, so the consistent 2.9.0 c10/ATen headers are found first
#   and the stale /usr/local/include ones are blocked.
#   All shared libraries (libtorch, libc10, …) also come from the same coherent
#   /usr/local/libtorch/lib prefix, avoiding the ABI mismatch that would cause a
#   segfault in c10::AutogradState::set_tls_state at runtime.
#
# RPATH note: because /usr/local/libtorch/lib is NOT in GCC's implicit link
# directories, CMake auto-adds it to the binary RPATH and emits
# "Cannot generate a safe runtime search path" warnings (duplicate .so names in
# /usr/lib).  These warnings are harmless: the explicit RPATH entry ensures the
# correct 2.9.0 libraries are loaded at runtime.  The true fix is to remove the
# stale leftovers from /usr/local/include and /usr/local/lib.
set(Torch_DIR "/usr/local/libtorch/share/cmake/Torch")

# TorchConfig.cmake calls find_library(c10_LIBRARY c10 PATHS "${TORCH_INSTALL_PREFIX}/lib").
# Without NO_DEFAULT_PATH, CMAKE_SYSTEM_PREFIX_PATH (/usr/local) is searched
# first and resolves c10 to the stale /usr/local/lib/libc10.so instead of the
# intended /usr/local/libtorch/lib/libc10.so.  Pre-populate with NO_DEFAULT_PATH
# to pin the library to the correct libtorch prefix.
if(NOT DEFINED c10_LIBRARY OR c10_LIBRARY STREQUAL "c10_LIBRARY-NOTFOUND")
    find_library(c10_LIBRARY c10
        PATHS "/usr/local/libtorch/lib"
        NO_DEFAULT_PATH)
endif()

find_package(Torch REQUIRED)

# Suppress "Cannot generate a safe runtime search path" warnings while keeping
# the correct runtime library loading behaviour.
#
# Background: /usr/local/libtorch/lib is NOT in GCC's CMAKE_*_IMPLICIT_LINK_DIRECTORIES
# (/usr/lib, /lib, …).  For each non-implicit directory that the link command
# needs, CMake's RPATH conflict-checker runs:
#   "Does any .so in this dir have the same name as a .so in an implicit dir?"
# Both libtorch.so and libc10.so exist in /usr/lib (the system 2.10.0 package),
# so the check fires and emits the four warnings.
#
# Two-step fix propagated to parent scope (add_subdirectory ordering in the root
# CMakeLists.txt: PINNs [48] → examples [53] → tests [56]):
#
#   1. Add /usr/local/libtorch/lib to CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES.
#      CMake then treats the directory as "trusted / already searched" and skips
#      the conflict check entirely → no warning.
#
#   2. Add it to CMAKE_BUILD_RPATH and CMAKE_INSTALL_RPATH.
#      Marking a dir "implicit" causes CMake to omit it from the auto-generated
#      RPATH (implicit dirs don't need an explicit RPATH entry).  We NEED the
#      entry in the binary RPATH so the runtime linker loads the 2.9.0 libraries
#      from /usr/local/libtorch/lib BEFORE the ABI-incompatible 2.10.0 copies
#      in /usr/lib (without this the segfault in
#      c10::AutogradState::set_tls_state returns).  CMAKE_BUILD_RPATH entries
#      are always emitted unconditionally, independent of the implicit-dir list,
#      so they restore the entry that step 1 suppresses.
if(DEFINED TORCH_INSTALL_PREFIX)
    set(_torch_lib_dir "${TORCH_INSTALL_PREFIX}/lib")
    list(APPEND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${_torch_lib_dir}")
    list(APPEND CMAKE_BUILD_RPATH   "${_torch_lib_dir}")
    list(APPEND CMAKE_INSTALL_RPATH "${_torch_lib_dir}")
    set(CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
        "${CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES}" PARENT_SCOPE)
    set(CMAKE_BUILD_RPATH   "${CMAKE_BUILD_RPATH}"   PARENT_SCOPE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}" PARENT_SCOPE)
    unset(_torch_lib_dir)
endif()

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

set(MODELS_DIRECTORY "${CMAKE_SOURCE_DIR}/shared/models" CACHE PATH "Path to the models directory")
target_compile_definitions(${PROJECT_NAME} INTERFACE models_Directory="${MODELS_DIRECTORY}")

##### compile optimization  
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-O3" COMPILER_OPT_O3_SUPPORTED)
if(COMPILER_OPT_O3_SUPPORTED)
    message(STATUS ">>> COMPILER_OPT_O3_SUPPORTED")
    target_compile_options(${PROJECT_NAME} INTERFACE -O3)
endif()

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