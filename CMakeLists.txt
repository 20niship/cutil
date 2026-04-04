cmake_minimum_required(VERSION 3.12)
project(cutil)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

enable_testing()
add_subdirectory(tests)
add_subdirectory(example)
