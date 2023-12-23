project(parking_plotter)

# find_package (Python COMPONENTS Interpreter Development)
set(PYBIND11_PYTHON_VERSION 3.5)
find_package(pybind11 REQUIRED PATHS 
             /home/ros/.local/lib/python3.5/site-packages/pybind11/share/cmake/pybind11)
# find_package(osqp REQUIRED)



option(DLOCAL_LOG "print log" OFF)

if(DLOCAL_LOG)
    message("Defined DLOCAL_LOG in cmake")    
    add_definitions(-DDLOCAL_LOG)
else()
    message("Not defined dlog in cmake")
endif()
add_definitions(-DLOCAL_MODE)

file(GLOB_RECURSE CPP_SRCS 
     ../../../src/planning/src/common/utils/spline.cpp
     ../../../src/planning/src/planner/motion_planner/optimizers/parking_speed_optimizer/*.cpp
     sbp_planner_wrapper.cpp 
     util.cpp
     parking_scenario.cpp
     vehicle_kinematic_model.cpp
     pose_adjuster.cpp
     planner_interface.cpp
     perfect_scene.cpp
     clothoid_bind.cpp
     speed_planner/*.cpp
     heuristic_cost.cpp

     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/*.c*
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/*.cpp

    ../../../src/planning/src/planner/behavior_planner/parking/sv_speed_generator.cpp
     
     ../../../src/planning/src/common/config/vehicle_param.cpp
     ../../../src/planning/src/common/math/aabox2d.cpp
     ../../../src/planning/src/common/math/box2d.cpp
     ../../../src/planning/src/common/math/circle2d.cpp
     ../../../src/planning/src/common/math/line_segment2d.cpp
     ../../../src/planning/src/common/math/math_utils.cpp
     ../../../src/planning/src/common/math/polygon2d.cpp
     ../../../src/planning/src/common/math/vec2d.cpp

     ../../../src/planning/src/common/utils/trajectory_point_utils.cpp
     ../../../src/planning/src/common/parking_config_deserialization.cpp

     ../../../src/planning/src/ego_model_manager.cpp
     ../../../src/planning/src/priority_obs.cpp
     ../../../src/planning/src/sbp_strategy.cpp

     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/Compare.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/config.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/parallel_path_segments.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/reeds_shepp_path.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/sbp_rspath.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/State.cc
     ../../../src/planning/src/planner/behavior_planner/parking/speed_margin_limiter.cpp

     #hybrid_a_star_2 files
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.cc
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/curve.cpp
     ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/search_node.cpp
     )
list(FILTER CPP_SRCS EXCLUDE REGEX ../../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/config_utils.cpp)
     
find_package (Threads)

if(NOT DEFINED CMAKE_OBJCOPY_TARGET)
  if(${CMAKE_SYSTEM_NAME} STREQUAL Linux AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64)
    set(CMAKE_OBJCOPY_TARGET elf64-x86-64)
  elseif(${CMAKE_SYSTEM_NAME} STREQUAL Linux AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL aarch64)
    set(CMAKE_OBJCOPY_TARGET elf64-littleaarch64)
  else()
    message(FATAL_ERROR "Can't deduce target format for objcopy, please set CMAKE_OBJCOPY_TARGET manually")
  endif()
endif()
if(NOT DEFINED CMAKE_OBJCOPY_ARCH)
  if(${CMAKE_SYSTEM_NAME} STREQUAL Linux AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64)
    set(CMAKE_OBJCOPY_ARCH i386)
  elseif(${CMAKE_SYSTEM_NAME} STREQUAL Linux AND ${CMAKE_SYSTEM_PROCESSOR} STREQUAL aarch64)
    set(CMAKE_OBJCOPY_ARCH aarch64)
  else()
    message(FATAL_ERROR "Can't deduce target architecture for objcopy, please set CMAKE_OBJCOPY_ARCH manually")
  endif()
endif()
set(MODEL_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../resource/config/scenario_configs_json/parking/)
set(MODEL_FILES
  car_configs/multi_circle_footprint_model_lacar.yaml
  car_configs/multi_circle_footprint_model_wls_c03.yaml
  car_configs/multi_circle_footprint_model_lhs_sg.yaml
  car_configs/multi_circle_footprint_model_lhs_uxe.yaml
  car_configs/multi_circle_footprint_model_lbcar.yaml
  car_configs/multi_circle_footprint_model_bys_a02.yaml
  car_configs/multi_circle_footprint_model_harz_s450l.yaml
  car_configs/multi_circle_footprint_model_rocky_lyriq.yaml
  car_configs/multi_circle_footprint_model_lhs_esea.yaml
  car_configs/multi_circle_footprint_model_lccar.yaml
  heuristic_offline/boundary_cost.bin
  heuristic_offline/gear_switch_cost_1.bin
  heuristic_offline/gear_switch_cost_3.bin
  heuristic_offline/gear_switch_cost_5.bin
  heuristic_offline/gear_switch_cost_7.bin
  heuristic_offline/gear_switch_cost_10.bin
)
foreach(FILE ${MODEL_FILES})
  get_filename_component(FILE_DIR ${FILE} DIRECTORY)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${FILE_DIR})
  set(FILE_OBJ ${CMAKE_CURRENT_BINARY_DIR}/${FILE}.o)
  add_custom_command(
    OUTPUT ${FILE_OBJ}
    COMMAND cd ${MODEL_BASE_DIR} && ${CMAKE_OBJCOPY} --input-target binary --output-target ${CMAKE_OBJCOPY_TARGET}
    --binary-architecture ${CMAKE_OBJCOPY_ARCH} ${FILE} ${FILE_OBJ}
    DEPENDS ${CMAKE_OBJCOPY} ${MODEL_BASE_DIR}/${FILE}
  )
  list(APPEND MODEL_OBJECT_FILES ${FILE_OBJ})
endforeach()

add_library(${PROJECT_NAME} ${CPP_SRCS} ${MODEL_OBJECT_FILES})
target_link_libraries(${PROJECT_NAME}
  ${rosbag_LIBRARIES}
  ${pybind11_LIBRARIES}
  yaml-cpp-interface
  pthread
  stdc++fs
  osqp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE HASTAR2_HAS_INTERNAL_MODEL)
target_compile_definitions(${PROJECT_NAME} PUBLIC BUILD_IN_TEST_BAG_RECURRENT)

message("----------------->"${CMAKE_CURRENT_SOURCE_DIR})
pybind11_add_module(py_${PROJECT_NAME} MODULE bind.cpp)
target_link_libraries(py_${PROJECT_NAME} ${PROJECT_NAME})

target_include_directories(py_${PROJECT_NAME} PUBLIC ${pybind11_INCLUDE_DIRS})
add_library(${PROJECT_NAME}_c SHARED c_bind.cpp)
target_link_libraries(${PROJECT_NAME}_c ${PROJECT_NAME})

pybind11_add_module(${PROJECT_NAME}_cpython_wrapper MODULE py_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_cpython_wrapper
  ${pybind11_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_cpython_wrapper PUBLIC ${pybind11_INCLUDE_DIRS})



add_executable(test_get_next_state_offline 
    has2_get_next_state_offline/test_get_next_state_offline_main.cpp
    has2_get_next_state_offline/next_state_generator.cpp
    has2_get_next_state_offline/next_state_visualizer.cpp
)

target_link_libraries(test_get_next_state_offline ${PROJECT_NAME})


add_executable(test_voroni_opencv
  has2_obstacle_grid_voroni/test_voroni_opencv_main.cpp 

)

target_link_libraries(test_voroni_opencv ${PROJECT_NAME})


add_executable(test_obstacle_grid
  has2_obstacle_grid_voroni/test_obstacle_grid_main.cpp 

)

target_link_libraries(test_obstacle_grid ${PROJECT_NAME})

