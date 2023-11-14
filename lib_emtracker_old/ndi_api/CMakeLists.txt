project(NDI)

# List the EMT library source files into a variable
file(GLOB NDI_LIB_SOURCES 
    "src/*.cpp")

# Create a dhared/dynamic library using the listed source files
add_library(${PROJECT_NAME} SHARED ${NDI_LIB_SOURCES})

# Add the necessary include directories for the EMTlib library
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/include)

# Turn off build warnings from NDI-api 
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unknown-pragmas")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-variable")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-but-set-variable")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-unused-parameter")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-sign-compare")
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-overflow")




