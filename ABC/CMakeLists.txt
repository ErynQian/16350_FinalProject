cmake_minimum_required(VERSION 3.6)   # CMake version check
project(planner)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

set(SOURCE_FILES bco.cpp bco.hpp)            # Add main.cpp file of project root directory as source file
add_executable(planner ${SOURCE_FILES})