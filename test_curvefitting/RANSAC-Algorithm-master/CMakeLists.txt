cmake_minimum_required(VERSION 3.5)

project(test4)

# 添加Eigen库的头文件路径
include_directories(/usr/include/eigen3)

# 将test2.cc添加到源文件列表中
file(GLOB SOURCE ./*.cpp)

# 添加可执行文件
add_executable(test ${SOURCE})