cmake_minimum_required(VERSION 3.0)
project(tools)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "")

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-O2)
add_compile_options(-g)

add_subdirectory(calc_one_frame)
add_subdirectory(trace_throw)
