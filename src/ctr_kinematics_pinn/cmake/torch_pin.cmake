# Pin LibTorch to the self-contained install at /usr/local/libtorch (override
# with -DCTR_TORCH_DIR=<prefix>). Runs both when this package builds and, via
# CONFIG_EXTRAS, whenever a downstream package find_package()s ctr_kinematics_pinn.
#
# Problem this solves (see the original planner/ctr_pinn_infer/CMakeLists.txt):
# stale headers in /usr/local/include (ATen/, c10/, caffe2/, ...) from an older
# manual LibTorch install shadow both the system Torch headers AND the
# self-contained ones, producing std::optional/c10::optional compile errors and
# an ABI mismatch that segfaults in c10::AutogradState::set_tls_state at
# runtime. Pinning Torch_DIR makes CMake pass the coherent include dirs as
# -isystem flags (searched before the stale defaults) and link the matching
# libraries from one prefix.
if(NOT DEFINED CTR_TORCH_DIR)
    set(CTR_TORCH_DIR "/usr/local/libtorch")
endif()

if(EXISTS "${CTR_TORCH_DIR}/share/cmake/Torch")
    set(Torch_DIR "${CTR_TORCH_DIR}/share/cmake/Torch")

    # TorchConfig.cmake calls find_library(c10_LIBRARY c10 PATHS "${TORCH_INSTALL_PREFIX}/lib").
    # Without NO_DEFAULT_PATH, CMAKE_SYSTEM_PREFIX_PATH (/usr/local) is searched
    # first and resolves c10 to a stale /usr/local/lib/libc10.so instead of the
    # intended ${CTR_TORCH_DIR}/lib/libc10.so. Pre-populate with NO_DEFAULT_PATH
    # to pin the library to the correct libtorch prefix.
    if(NOT DEFINED c10_LIBRARY OR c10_LIBRARY STREQUAL "c10_LIBRARY-NOTFOUND")
        find_library(c10_LIBRARY c10
            PATHS "${CTR_TORCH_DIR}/lib"
            NO_DEFAULT_PATH)
    endif()
endif()

find_package(Torch REQUIRED)

# Suppress "Cannot generate a safe runtime search path" warnings while keeping
# correct runtime library loading: mark the libtorch lib dir implicit (skips
# CMake's same-soname conflict check against /usr/lib) but force it back into
# the RPATH so the pinned libraries are loaded before any system copies.
if(DEFINED TORCH_INSTALL_PREFIX)
    set(_torch_lib_dir "${TORCH_INSTALL_PREFIX}/lib")
    list(APPEND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${_torch_lib_dir}")
    list(APPEND CMAKE_BUILD_RPATH   "${_torch_lib_dir}")
    list(APPEND CMAKE_INSTALL_RPATH "${_torch_lib_dir}")
    unset(_torch_lib_dir)
endif()
