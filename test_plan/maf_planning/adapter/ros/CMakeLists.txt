cmake_minimum_required(VERSION 2.8.3)
project(msquare_pnc_ros)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  momenta_msgs
  tf
  planning_msgs
  worldmodel_msgs
  mla_localization_msgs
  framework_status_msgs
  gps_imu_msgs
  sensor_interface_msgs
  generate_ros_cpp_struct
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES decision_planning
#  DEPENDS system_lib
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include/maf_interface/include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/nlohmann_json
  ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/mtime/mtime_core/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/mlog/mlog_core/include
  # ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/cpr/include
)


get_filename_component(DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
set(MLOG_ENABLE_ROS ON CACHE BOOL "Enable ROS functions of MLog" FORCE)
set(MLOG_NAMESPACE ${PROJECT_NAME})
add_subdirectory(${DIR}/../../thirdparty/mlog thirdparty/mlog)
set(MTIME_ENABLE_ROS ON CACHE BOOL "Enable ROS functions of MTime" FORCE)
set(MTIME_NAMESPACE ${PROJECT_NAME})
add_subdirectory(${DIR}/../../thirdparty/mtime thirdparty/mtime)
add_subdirectory(${DIR}/../../thirdparty/mjson thirdparty/mjson)

add_subdirectory(thirdparty/msquare_pnc)

set(APPS ${PROJECT_NAME} ${PROJECT_NAME}_ldp ${PROJECT_NAME}_sbp)
add_executable(${PROJECT_NAME}
  src/msd_planning_node.cpp
  src/upload_msd_test.cpp
)
# target_link_libraries(${PROJECT_NAME} PRIVATE ${DIR}/../../thirdparty/cpr/lib/libcpr.so)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME decision PREFIX "")

add_executable(${PROJECT_NAME}_ldp
  src/ldp_planning_node.cpp
)
set_target_properties(${PROJECT_NAME}_ldp PROPERTIES OUTPUT_NAME ldp_planning_node PREFIX "")

add_executable(${PROJECT_NAME}_sbp
  src/search_based_planning_node.cpp
)
set_target_properties(${PROJECT_NAME}_sbp PROPERTIES OUTPUT_NAME search_based_planning_node PREFIX "")

target_link_libraries(${PROJECT_NAME} 
                      ${MLOG_NAMESPACE}::mlog_publisher
                      ${MTIME_NAMESPACE}::mtime_customize_timeline
                      ${MTIME_NAMESPACE}::mtime_core)
target_link_libraries(${PROJECT_NAME}_ldp
                      ${MLOG_NAMESPACE}::mlog_publisher
                      ${MTIME_NAMESPACE}::mtime_customize_timeline
                      ${MTIME_NAMESPACE}::mtime_core)
target_link_libraries(${PROJECT_NAME}_sbp
                      ${MLOG_NAMESPACE}::mlog_publisher
                      ${MTIME_NAMESPACE}::mtime_customize_timeline
                      ${MTIME_NAMESPACE}::mtime_core)

foreach(APP IN LISTS APPS)
  target_link_libraries(${APP} PRIVATE ${catkin_LIBRARIES})
  target_link_libraries(${APP} PRIVATE
                        msquare_pnc_basic
                        ${MLOG_NAMESPACE}::mlog_msg_id
                        ${MLOG_NAMESPACE}::mlog_publisher
                        ${MTIME_NAMESPACE}::mtime_customize_timeline
                        ${MTIME_NAMESPACE}::mtime_core)

  add_dependencies(${APP} ${${APP}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach(APP)

install(
  DIRECTORY ${DIR}/../../thirdparty/cpr/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(
  DIRECTORY ${DIR}/../../thirdparty/mimalloc-release/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    TARGETS ${APPS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/rosconsole.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

set(GENERATED_LAUNCH_FILES_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_launch")
file(GLOB LAUNCH_FILES "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.launch")
foreach(LAUNCH_FILE IN LISTS LAUNCH_FILES)
  get_filename_component(LAUNCH_FILE_NAME "${LAUNCH_FILE}" NAME)
  string(REPLACE ".launch" "__orig.launch" LAUNCH_FILE_NAME_RENAMED "${LAUNCH_FILE_NAME}")
  install(
      FILES ${LAUNCH_FILE}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      RENAME ${LAUNCH_FILE_NAME_RENAMED}
  )
  # Generate wrapper launch file
  set(WRAPPED_LAUNCH_FILE_NAME "${LAUNCH_FILE_NAME_RENAMED}")  # For template substitution
  set(GENERATED_LAUNCH_FILE "${GENERATED_LAUNCH_FILES_DIR}/${LAUNCH_FILE_NAME}")
  configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/launch/install_mode_resource_context.launch.in"
      "${GENERATED_LAUNCH_FILE}"
      @ONLY
  )
  install(
      FILES ${GENERATED_LAUNCH_FILE}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
      RENAME ${LAUNCH_FILE_NAME}
  )
endforeach(LAUNCH_FILE)