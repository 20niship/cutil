cmake_minimum_required(VERSION 3.15)
project(mylib_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ext/nanobind/include)

# nanobind core sources
set(NANOBIND_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../ext/nanobind/src/common.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../ext/nanobind/src/nb_func.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../ext/nanobind/src/nb_combined.cpp
)

# C++ shared library: mycpplib
add_library(mycpplib SHARED cpplib.cpp)

set_target_properties(mycpplib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Python module: mylib
add_library(mylib MODULE mylib.cpp ${NANOBIND_SOURCES})

target_include_directories(mylib PRIVATE
    ${Python_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../ext/nanobind/include
)

target_link_libraries(mylib PRIVATE Python::Python)

# Set Python extension properties
set_target_properties(mylib PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
