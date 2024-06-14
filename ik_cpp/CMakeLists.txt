# Modified from https://gitlab.com/Fraser999/Rust-From-CXX/

project(ik_cpp)
cmake_minimum_required(VERSION 3.0)

# Allow shared or static libraries
add_library(ik_object OBJECT
    cpp/robot.hpp
    cpp/robot.cpp
    )

target_include_directories(ik_object PRIVATE cpp)
# Enable ExternalProject CMake module
include(ExternalProject)

# Set default ExternalProject root directory
set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/Rust)

# Add rust_example as a CMake target
ExternalProject_Add(
    ik_cpp
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build --release
    BINARY_DIR "${CMAKE_SOURCE_DIR}/rust-wrapper"
    INSTALL_COMMAND ""
    LOG_BUILD ON)

# Create dependency of Example on rust_example, should be bundled in the code
add_dependencies(ik_object ik_cpp)


# Specify link libraries

if (WIN32)
    target_link_libraries(ik_object
        debug "${CMAKE_SOURCE_DIR}/rust-wrapper/target/debug/libik_cpp.a"
        optimized "${CMAKE_SOURCE_DIR}/rust-wrapper/target/release/libik_cpp.a"
        ws2_32 userenv advapi32)
else ()
    target_link_libraries(ik_object
        debug "${CMAKE_SOURCE_DIR}/rust-wrapper/target/debug/libik_cpp.a"
        optimized "${CMAKE_SOURCE_DIR}/rust-wrapper/target/release/libik_cpp.a")
endif()



set_target_properties(ik_object PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)

# Build to the dist folder
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/dist)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/dist)

# Create both shared and static libraries to the dist folder
# Only building static library right now because runtime speed is important
add_library(ik_geo STATIC $<TARGET_OBJECTS:ik_object>)
# add_library(ik_geo_dynamic SHARED $<TARGET_OBJECTS:ik_object>)


# Custom command to copy libik_cpp.a to the dist folder
add_custom_command(TARGET ik_geo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    "${CMAKE_SOURCE_DIR}/rust-wrapper/target/release/libik_cpp.a"
    "${CMAKE_BINARY_DIR}/dist/libik_cpp.a")

# Copy ik_geo.h to the dist folder
add_custom_command(TARGET ik_geo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    "${CMAKE_SOURCE_DIR}/ik_geo.h"
    "${CMAKE_BINARY_DIR}/dist/ik_geo.h")
