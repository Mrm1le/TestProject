# 设置项目名称
project(test_control_service)

# 设置最低的CMake版本
cmake_minimum_required(VERSION 3.10)

# 添加可执行文件，并指定要编译的源文件
add_executable(${PROJECT_NAME} core.cpp mfr_node.cpp pie_node.cpp main.cpp)
