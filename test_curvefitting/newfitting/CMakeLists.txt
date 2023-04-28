cmake_minimum_required(VERSION 3.5)

project(test4)

# 添加Eigen库的头文件路径
include_directories("/usr/include/eigen3")
# include_directories(include/eigen)
# 将test2.cc添加到源文件列表中
set(SOURCE_FILES test.cc)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# 添加可执行文件
add_executable(test ${SOURCE_FILES})

# 链接Eigen库
# target_link_libraries(test4 Eigen3)