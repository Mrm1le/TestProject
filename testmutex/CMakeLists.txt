project(test_mutex)
cmake_minimum_required(VERSION 3.15)
# set(CMAKE_CXX_FLAGS -pthread)
# set(CMAKE_CXX_FLAGS -lboost_system)

find_package(Boost REQUIRED COMPONENTS thread chrono)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_executable(
    ${PROJECT_NAME}
    test_mutex.cpp
)

target_link_libraries(
    ${PROJECT_NAME} 
    Boost::thread Boost::chrono pthread)

add_executable(
    ${PROJECT_NAME}_std
    teststdmutex.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_std 
    pthread)