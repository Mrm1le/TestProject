cmake_minimum_required(VERSION 3.0)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

project(msquare_pnc_basic)
get_filename_component(DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
set (MSQUARE_PNC_DIR ${DIR}/../../../../)

ExternalProject_Add(msquare_pnc
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/
  BUILD_ALWAYS 1
  SOURCE_DIR ${MSQUARE_PNC_DIR}
  CMAKE_ARGS -DMLOG_ENABLE_ROS=ON
             -DMTIME_ENABLE_ROS=ON
             -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
             -DMLOG_ENABLE_PROFILING=ON
)

ExternalProject_Get_Property(msquare_pnc binary_dir)

add_library(msquare_pnc_basic INTERFACE)
add_dependencies(msquare_pnc_basic msquare_pnc)
target_include_directories(msquare_pnc_basic INTERFACE ${binary_dir}/install/include)
target_link_libraries(msquare_pnc_basic INTERFACE ${binary_dir}/install/lib/libmsquare_pnc.so)

install(DIRECTORY ${binary_dir}/install/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY ${MSQUARE_PNC_DIR}/resource/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
  FILES_MATCHING PATTERN "*"
)
