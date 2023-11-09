project(KEYINPUTlib)

set(INPUT_LIB_SOURCES
    src/keyboardInput.cpp
)


# Create a static library named EMTlib using the listed source files
add_library(${PROJECT_NAME} STATIC ${INPUT_LIB_SOURCES})

# Add the necessary include directories for the EMTlib library
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


# Find the ncurses library and link
find_package(X11 REQUIRED)
if (X11_FOUND)
    message(STATUS ">>> Curses has been found!")
include_directories(${X11_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${X11_LIBRARIES})
else (NOT Curses_FOUND)
    error(STATUS ">>> Curses has not been found")
endif()
