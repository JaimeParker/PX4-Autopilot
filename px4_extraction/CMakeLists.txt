cmake_minimum_required(VERSION 3.10)
project(px4_attitude_controller_extraction VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compile options
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/attitude_control
)

# Add subdirectories
add_subdirectory(lib/matrix)
add_subdirectory(attitude_control)

# Optional: Build tests
option(BUILD_TESTS "Build test executables" ON)
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

# Print configuration
message(STATUS "PX4 Attitude Controller Extraction")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Tests: ${BUILD_TESTS}")
