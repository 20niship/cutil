cmake_minimum_required(VERSION 3.15)
project(mylib_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# C++ shared library: mycpplib
add_library(mycpplib SHARED cpplib.cpp)

set_target_properties(mycpplib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
