cmake_minimum_required(VERSION 3.12)
project(cutil)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(ext/nanobind)
nanobind_add_module( mylib NB_STATIC STABLE_ABI LTO example/mylib.cpp)

enable_testing()
add_subdirectory(tests)
