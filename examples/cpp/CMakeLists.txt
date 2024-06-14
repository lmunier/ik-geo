cmake_minimum_required(VERSION 3.12)
project(ik_geo_example VERSION 1.0.0)


find_library(ik_geo libik_geo.so PATHS lib)

# Add executable from main.cpp
add_executable(${PROJECT_NAME} main.cpp)


# Include the .so files in the include folder
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/libik_geo.a)

# Include libik_cpp.a
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/lib/libik_cpp.a)


include_directories(${CMAKE_SOURCE_DIR}/include)


# Link the executable to the library
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)