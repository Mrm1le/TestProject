project(msquare_pnc_planning)

get_property(CONANBUILDINFO_FILE GLOBAL PROPERTY "CONANBUILDINFO_FILE" ) 
include(${CONANBUILDINFO_FILE})

if (MPH_FRAMEWORK_ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11")
else()
endif()

if (RUN_IN_FPP)
  add_definitions(-DRUN_IN_FPP)
endif()
file(GLOB_RECURSE OPENSPACE_OPTIMIZER_SRC_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/planner/motion_planner/optimizers/openspace_optimizer/*
)
file(GLOB_RECURSE DDP_CPP_FILES src/data_driven_planner/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11 -Werror=non-virtual-dtor")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11")

#[fenix.shi] temporary usage during refactor.apa.statemachine (for pragma region)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")


include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/maf_interface/include/)
add_library(${PROJECT_NAME} STATIC
  src/planner/behavior_planner/lateral_behavior_avd_info.cpp
  src/planner/behavior_planner/lateral_behavior_avoid_cars.cpp
  src/planner/behavior_planner/lateral_behavior_intersect_state.cpp
  src/planner/behavior_planner/lateral_behavior_path_planner.cpp
  src/planner/behavior_planner/lateral_behavior_planner.cpp
  src/planner/behavior_planner/lateral_behavior_planner_output.cpp
  src/planner/behavior_planner/lateral_behavior_data_manager.cpp
  src/planner/behavior_planner/lateral_behavior_request_manager.cpp
  src/planner/behavior_planner/lateral_behavior_road_state.cpp
  src/planner/behavior_planner/lateral_behavior_state_machine.cpp
  src/planner/behavior_planner/longitudinal_behavior_planner.cpp

  ## zjt debug 
  src/planner/behavior_planner/deciders/apf_decider.cpp
  src/planner/behavior_planner/deciders/st_boundary_mapper.cpp
  src/planner/behavior_planner/deciders/st_graph_generator.cpp
  src/planner/behavior_planner/deciders/path_decider.cpp
  src/planner/behavior_planner/deciders/speed_boundary_decider.cpp
  src/planner/behavior_planner/deciders/backup_path_decider.cpp
  src/planner/tasks/obstacle_decider.cpp
  src/planner/tasks/obstacle_decider_preprocessor.cpp
  src/planner/behavior_planner/general_motion_planner.cpp
  src/planner/behavior_planner/general_motion_planner_preprocessor.cpp
  src/planner/behavior_planner/deciders/lane_change_decider.cpp
  # src/planner/behavior_planner/deciders/freespace_decider.cpp
  src/planner/behavior_planner/deciders/ground_line_decider.cpp
  src/planner/behavior_planner/deciders/openspace_utils.cpp
  src/planner/behavior_planner/deciders/leader_decider.cpp
  src/planner/behavior_planner/deciders/remain_dist_decider.cpp
  src/planner/behavior_planner/deciders/collision_checker.cpp
  src/planner/behavior_planner/deciders/collision_checker_longitudinal.cpp
  src/planner/behavior_planner/parking/speed_margin_limiter.cpp
  src/planner/behavior_planner/deciders/openspace_decider_interface.cpp
  src/planner/behavior_planner/deciders/openspace_decider.cpp
  src/planner/behavior_planner/deciders/apoa_openspace_decider.cpp
  src/planner/behavior_planner/deciders/apa_openspace_decider.cpp
  src/planner/behavior_planner/deciders/rpa_straight_openspace_decider.cpp
  src/planner/behavior_planner/parking_behavior_planner.cpp
  src/planner/behavior_planner/parking_refline_manager.cpp
  src/planner/behavior_planner/parking_longitudinal_behavior_planner.cpp
  src/planner/behavior_planner/parking/sv_speed_generator.cpp
  src/planner/tasks/task.cpp
  src/planner/tasks/task_factory.cpp
  src/planner/tasks/deciders/decider.cpp
  src/planner/scenarios/scenario_facade.cpp
  src/planner/scenarios/loop_scenario_facade.cpp
  src/planner/scenarios/scenario_factory.cpp
  src/planner/arbitrators/arbitrator.cpp
  src/planner/arbitrators/priority_based_arbitrator.cpp
  src/planner/arbitrators/merit_based_arbitrator.cpp
  src/planner/arbitrators/single_scenario_arbitrator.cpp
  src/planner/arbitrators/sequence_arbitrator.cpp
  src/planner/arbitrators/arbitrator_cost_strategy.cpp
  src/planner/motion_planner/apf_planner.cpp
  src/planner/motion_planner/parking_motion_planner.cpp
  src/planner/motion_planner/lateral_motion_planner.cpp
  src/planner/motion_planner/lateral_motion_planner_preprocessor.cpp
  src/common/pass_intersection_planner_preprocessor.cpp
  src/planner/motion_planner/speed_planner_ceres/speed_planner_preprocessor.cpp
  src/planner/motion_planner/speed_planner_ceres/obstacle_headway_preprocessor.cpp
  src/planner/motion_planner/speed_planner_ceres/speed_planner_ceres.cpp
  src/planner/motion_planner/longitudinal_motion_planner.cpp
  src/planner/motion_planner/common/window_smoother.cpp
  src/planner/motion_planner/common/speed_profile_generator.cpp
  src/planner/motion_planner/optimizers/optimal_speed_planner.cpp
  src/planner/motion_planner/path_planner_ceres/path_planner_ceres.cpp
  src/planner/motion_planner/path_planner_ceres/path_planner_decider.cpp
  src/planner/motion_planner/path_planner_ceres/lateral_active_avoid.cpp
  src/planner/motion_planner/parking_longitudinal_motion_planner.cpp
  src/planner/motion_planner/parking_speed_planner_v2.cpp
  src/planner/motion_planner/openspace_motion_planner/openspace_motion_planner.cpp
  src/planner/motion_planner/openspace_motion_planner/zigzag_path.cpp
  src/planner/motion_planner/openspace_motion_planner/zigzag_pathsampler.cpp
  src/planner/motion_planner/openspace_motion_planner/path_sampler.cpp
  src/common/search_based_planning_utils.cpp
  src/common/parking_config_deserialization.cpp
  src/planner/motion_planner/optimizers/parking_speed_optimizer/osqp_wrapper.cpp
  src/planner/motion_planner/optimizers/parking_speed_optimizer/piecewise_jerk_qp.cpp
  src/planner/motion_planner/optimizers/parking_speed_optimizer/st_pieceewise_jerk_speed_optimizer.cpp
  # src/common/search_based_planning.cpp
  src/common/refline_generator.cpp
  src/common/search_based_planning_task.cpp
  src/common/search_based_planning_engine.cpp
  src/common/search_based_planning_engine_interface.cpp
  src/common/priority_obs.cpp
  src/common/sbp_strategy.cpp
  src/common/parking_task_config.cpp
  src/common/parking_slot_manager.cpp
  src/common/ego_state_manager.cpp
  src/common/parking_ego_state_manager.cpp
  src/common/map_info_manager.cpp
  src/common/parking_map_info_manager.cpp
  src/common/obstacle.cpp
  src/common/parking_obstacle.cpp
  src/common/obstacle_manager.cpp
  src/common/parking_obstacle_manager.cpp
  src/common/obstacle_decision.cpp
  src/common/obstacle_decision_manager.cpp
  src/common/planning_engine.cpp
  src/common/ldp_planning_engine.cpp
  src/common/planning_engine_interface.cpp
  src/common/ldp_planning_engine_interface.cpp
  src/common/scenario_manager.cpp
  src/common/traffic_light_decision.cpp
  src/common/world_model.cpp
  src/common/parking_world_model.cpp
  src/common/baseline_info.cpp
  src/common/collision_check.cpp
  src/common/static_check.cpp
  src/common/lateral_obstacle.cpp
  src/common/lateral_virtual_lane.cpp
  src/common/tracklet_maintainer.cpp
  src/common/refline.cpp
  src/common/extract_path_bound.cpp
  src/common/config/vehicle_param.cpp
  src/common/config/lateral_planning_config.cpp
  src/common/config/refline_generator_config.cpp
  src/common/parking_lot.cpp
  src/common/angle_parking_slot.cpp
  src/common/parallel_parking_slot.cpp
  src/common/parking_slot_factory.cpp
  src/common/math/vec2d.cpp 
  src/common/math/aabox2d.cpp 
  src/common/math/box2d.cpp 
  src/common/math/line_segment2d.cpp 
  src/common/math/line_segmentxd.cpp 
  src/common/math/line2d.cpp 
  src/common/math/linear_interpolation.cpp 
  src/common/math/math_utils.cpp 
  src/common/math/polygon2d.cpp 
  src/common/math/circle2d.cpp 
  src/common/math/curve_join.cpp 
  src/common/path/discretized_path.cpp
  src/common/path/path_data.cpp
  src/common/path/path_boundary.cpp
  src/common/refline/reference_line_provider.cc
  src/common/refline/qp_spline_reference_line_smoother.cc
  src/common/refline/refline_smoother/affine_constraint.cc
  src/common/refline/refline_smoother/spline_2d.cc
  src/common/refline/refline_smoother/spline_2d_constraint.cc
  src/common/refline/refline_smoother/spline_2d_kernel.cc
  src/common/refline/refline_smoother/spline_2d_seg.cc
  src/common/refline/refline_smoother/spline_seg_kernel.cc
  src/common/refline/reference_point.cc
  src/common/parking_lot.cpp
  include/common/math/curve_math.cc
  include/common/math/cartesian_frenet_conversion.cc
  include/common/math/angle.cc 
  include/common/math/matrix_operations.cc
  include/common/math/sin_table.cc
  include/common/math/polynomial_xd.cc
  include/common/utils/util.cc
  src/common/trajectory/discretized_trajectory.cpp
  src/common/trajectory/bounded_constant_jerk_trajectory1d.cc
  src/common/trajectory/smooth_brake_trajectory.cc
  src/common/trajectory/trajectory_stitcher.cpp
  src/common/speed/st_point.cpp
  src/common/speed/speed_limit.cpp
  src/common/speed/st_graph_point.cpp
  src/common/speed/st_graph_data.cpp
  src/common/speed/st_boundary.cpp
  src/common/speed/speed_data.cpp
  src/common/speed/sl_polygon_seq.cpp
  src/common/utils/lateral_utils.cpp
  src/common/utils/pose2d_utils.cpp
  src/common/utils/polyfit.cpp
  src/common/utils/spline.cpp
  src/common/utils/cartesian_coordinate_system.cpp
  src/common/utils/frenet_coordinate_system.cpp
  src/common/utils/trajectory_point_utils.cpp
  src/common/vehicle_model/vehicle_model.cpp
  src/common/trajectory_loader.cpp
  # src/gjk/collision/CircleDetector.cpp
  # src/gjk/collision/EPAMinkowskiPenetrationSolver.cpp
  # src/gjk/collision/ExpandingSimplex.cpp
  # src/gjk/collision/GJKCollisionDetector.cpp
  # src/gjk/geometry/Circle.cpp
  # src/gjk/geometry/Polygon.cpp
  # src/gjk/math/Epsilon.cpp
  ${OPENSPACE_OPTIMIZER_SRC_FILE}
  ${DDP_CPP_FILES}
  src/common/ego_model_manager.cpp
  src/common/math/control.cpp
  src/common/time_state_machine/time_state_machine.cpp
  src/common/hmi/acc_takeover.cpp
  src/common/hmi/hmi_manager.cpp

  src/common/apa_workflow/parking_planning_task.cpp
  src/common/apa_workflow/apa_state_machine_common.cpp
  src/common/apa_workflow/apa_state_machine_wait.cpp
  src/common/apa_workflow/apa_state_machine_parkin.cpp
  src/common/apa_workflow/apa_state_machine_parkout.cpp
  src/common/apa_workflow/apa_state_machine_rpastraight.cpp
  src/common/apa_workflow/openspace_state_machine.cpp
  src/common/apa_workflow/apa_behavior_calculator_parkin.cpp
  src/common/apa_workflow/apa_behavior_calculator_parkout.cpp
  src/common/apa_workflow/apa_behavior_decider_parkin.cpp

#  src/common/apa_workflow/parking_scenario_manager.cpp
#  src/common/apa_workflow/scenario_manager_state_apa.cpp
#  src/common/apa_workflow/scenario_manager_state_openspace.cpp
#  src/common/apa_workflow/scenario_manager_state_parking.cpp

  include/common/grid_map/grid_map.cpp
)

if (MPH_FRAMEWORK_ON)
else()
  include_directories(${YAML_CPP_INCLUDE_DIR})
  include_directories(${EIGEN3_INCLUDE_DIR})

endif()

if ("${CONAN_SETTINGS_OS}" STREQUAL "MDC")
    target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_COMMON_PATH="config")
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_COMMON_PATH="${CMAKE_CURRENT_LIST_DIR}/include/common")
endif()
target_include_directories(${PROJECT_NAME} PUBLIC msd_planning_common)
target_include_directories(${PROJECT_NAME} PUBLIC parking_worldmodel)
target_include_directories(${PROJECT_NAME} PUBLIC highway_worldmodel)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC include/planner/motion_planner/path_planner_ceres/)
target_include_directories(${PROJECT_NAME} PUBLIC include/planner/motion_planner/speed_planner_ceres/)
target_include_directories(${PROJECT_NAME} PUBLIC include/planner/motion_planner/planner_cubic_spline/)

if (MPH_FRAMEWORK_ON)
else()
  add_library(yaml-cpp-interface INTERFACE)
  target_link_libraries(yaml-cpp-interface INTERFACE ${YAML_CPP_LIBRARIES})
  target_include_directories(yaml-cpp-interface INTERFACE ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp-interface)
endif()

# target_link_libraries(${PROJECT_NAME} PUBLIC ${CONAN_LIBS_MF_MLOG_CORE})
# target_link_libraries(${PROJECT_NAME} PUBLIC ${CONAN_LIBS_MF_MTIME_CORE})
#target_link_libraries(${PROJECT_NAME} PRIVATE rapidjson)
target_link_libraries(${PROJECT_NAME} PUBLIC msd_planning_common)
target_link_libraries(${PROJECT_NAME} PUBLIC parking_worldmodel)
target_link_libraries(${PROJECT_NAME} PUBLIC highway_worldmodel)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${CONAN_LIBS_MF_MTASKFLOW})
target_link_libraries(${PROJECT_NAME} PUBLIC planning_define)
#target_link_libraries(${PROJECT_NAME} PUBLIC osqpstatic)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${CONAN_LIBS})
#target_link_libraries(${PROJECT_NAME} PUBLIC despot)

if (MPH_FRAMEWORK_ON)
  if (MOTIONWISE)
    target_link_libraries(${PROJECT_NAME} PUBLIC
      ${BOOST_PLANNING_LIB}
      ${CONAN_LIBS_YAML})

  else()
    target_link_libraries(${PROJECT_NAME} PUBLIC
      ${YAML_CPP_PLANNING_LIB}
      ${BOOST_PLANNING_LIB})
    include_directories(${YAML_CPP_INCLUDE_DIR})

  endif()  
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
endif()

# target_link_libraries(${PROJECT_NAME} PUBLIC ${PYTHON_LIBRARIES})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}) 

if (CMAKE_SYSTEM_NAME STREQUAL "QNX")
else()
  target_link_libraries(${PROJECT_NAME}
    PRIVATE
      rt
      pthread
      dl)
endif()
