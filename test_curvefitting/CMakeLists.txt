cmake_minimum_required(VERSION 3.5)

project(test4)

# 添加Eigen库的头文件路径
include_directories(/usr/include/eigen3)

# 将test2.cc添加到源文件列表中
set(SOURCE_FILES test4.cc)

# 添加可执行文件
add_executable(test4 ${SOURCE_FILES})

# 链接Eigen库
# target_link_libraries(test4 Eigen3)
