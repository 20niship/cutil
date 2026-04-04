cmake_minimum_required(VERSION 3.12)
project(cutil)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Enable testing
enable_testing()

# Add subdirectories
add_subdirectory(tests)
