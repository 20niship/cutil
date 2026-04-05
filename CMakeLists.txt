cmake_minimum_required(VERSION 3.12)
project(cutil)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# O3 optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(Python COMPONENTS Interpreter Development REQUIRED)
add_subdirectory(ext/nanobind)
nanobind_add_module(mylib NB_STATIC STABLE_ABI LTO example/mylib.cpp)

enable_testing()
add_subdirectory(tests)
