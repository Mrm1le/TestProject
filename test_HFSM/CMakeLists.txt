cmake_minimum_required(VERSION 3.0)
project(MyStateMachine)

set(CMAKE_CXX_STANDARD 17)
# 指定HFSM库的位置

set(HFSM_ROOT /home/thj/Desktop/TestProject/test_HFSM/)
# 包含HFSM库的头文件路径

include_directories(${HFSM_ROOT})
# 添加源文件

file(GLOB_RECURSE HFSM2_SRC HFSM2/*.cpp)
add_executable(MyStateMachine test.cc
                            ${HFSM2_SRC})
# 链接HFSM库

# target_link_libraries(MyStateMachine ${HFSM_ROOT}/libhfsm.a)


# 在cmakelists.txt文件中，我们首先指定了我们的项目名称和所需的CMake版本。接下来，我们将C++标准设置为C++17。然后，我们指定了HFSM库的位置，并将其包含在我们的项目中。最后，我们添加了我们的源文件，并链接了HFSM库。

# 现在，我们可以使用CMake编译我们的项目，并运行生成的可执行文件来测试我们的状态机实现。

# ```bash
# mkdir build
# cd build
# cmake ..
# make
# ./MyStateMachine
