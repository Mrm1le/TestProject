add_library(msg_cache SHARED)

target_sources(msg_cache
  PRIVATE
  maf_message_cache.cpp
  ros_convert.cpp)

target_include_directories(
  msg_cache
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  /opt/ros/catkin_ws/install/include
  /opt/ros/catkin_ws/build/maf_common/generate_ros_cpp_struct/src/
  /opt/ros/kinetic/include)

find_package(rosbag)
include(CMakePrintHelpers)
cmake_print_variables(rosbag_INCLUDE_DIRS)
target_include_directories(msg_cache
  PRIVATE
  ${GLOG_INCLUDE_DIR}
  ${MAF_INTERFACE_INCLUDE_DIR}
  ${rosbag_INCLUDE_DIRS})

cmake_print_variables(rosbag_LIBRARIES)
target_link_libraries(msg_cache
  PRIVATE
  ${rosbag_LIBRARIES})

install(TARGETS msg_cache
  LIBRARY DESTINATION lib)

