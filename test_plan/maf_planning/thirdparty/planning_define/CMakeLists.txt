cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 11)
project(planning_define)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
