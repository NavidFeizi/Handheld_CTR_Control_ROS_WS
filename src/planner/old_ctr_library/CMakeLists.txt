# Source files
set(CTR_LIB_SOURCES
    src/CTR.cpp
    src/Observer.cpp
    src/ODESystem.cpp
    src/Segment.cpp
    src/Tube.cpp
)

# Static library
add_library(CTRlib STATIC ${CTR_LIB_SOURCES})

# Include directories (PUBLIC for dependents)
target_include_directories(CTRlib
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies (PRIVATE if used only in implementation)
target_link_libraries(CTRlib
    PRIVATE
        NLopt::nlopt      # Use imported target if available
        m                 # Math library
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        gfortran
)

target_compile_features(CTRlib PUBLIC cxx_std_17)