include_directories(${CMAKE_CURRENT_LIST_DIR})
set(CALC_ONE_FRAME_NPP_SRCS
  calc_one_frame.cpp
  planning_task_on_running.cpp
)

add_library(calc_one_frame_npp SHARED ${CALC_ONE_FRAME_NPP_SRCS})

target_include_directories(
  calc_one_frame_npp INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(
  calc_one_frame_npp
  PRIVATE
  ${CONAN_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_SOURCE_DIR}/src/highway_worldmodel/include
  ${CMAKE_SOURCE_DIR}/src/parking_worldmodel/include
  ${CMAKE_SOURCE_DIR}/src/parking_worldmodel/worldmodel_pec/interface
  ${CMAKE_SOURCE_DIR}/src/planning/include
  ${CMAKE_SOURCE_DIR}/thirdparty/planning_define/include
  ${CMAKE_SOURCE_DIR}/src/common/include
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/src/planning/include/planner/motion_planner/path_planner_ceres/
  ${extra_data_proto_INCLUDE_DIRS}
)

target_link_libraries(
  calc_one_frame_npp
  PUBLIC
  apa_msquare_pnc
  absl::symbolize
  ${OpenCV_LIBS}
  ${CONAN_LIBS}
  ${CONAN_LIB_DIRS}
  ${extra_data_proto_LIBRARIES}
  msg_cache
  )

install(TARGETS calc_one_frame_npp RUNTIME DESTINATION lib)


