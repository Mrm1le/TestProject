project(bag_analysis)

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-fPIC)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CLANG_COMPILER ON)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
if(CLANG_COMPILER)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-sign-compare -Wno-reorder -Wno-parentheses")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wno-sign-compare -Wno-parentheses")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wvla")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-reorder -Wno-parentheses")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-parentheses")
endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_subdirectory(osqp)
add_custom_target(osqp_include
    COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/osqp_include && cp -r ${CMAKE_CURRENT_SOURCE_DIR}/osqp/include ${CMAKE_CURRENT_BINARY_DIR}/osqp_include/osqp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/osqp)
add_dependencies(osqp osqp_include)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${rosbag_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/maf_interface/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/planning/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/nlohmann_json
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/planning_define/include
    ${CMAKE_CURRENT_BINARY_DIR}/osqp_include
    /home/ros/catkin_ws/src/maf_interface/include
)


find_package(yaml-cpp REQUIRED)
add_library(yaml-cpp-interface INTERFACE)
target_link_libraries(yaml-cpp-interface INTERFACE ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(yaml-cpp-interface INTERFACE ${YAML_CPP_INCLUDE_DIR})

add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(tools)
