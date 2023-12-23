cmake_minimum_required(VERSION 2.8.3)
project(openspace_computation_analysis)
add_compile_options(-std=c++14)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -pg")

add_executable(openspace_computation_analysis
    openspace_computation_analysis.cpp
    ../../src/planning/src/common/sbp_strategy.cpp
    ../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.cc
    ../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.cc
    ../../src/planning/src/common/priority_obs.cpp

)
target_include_directories(openspace_computation_analysis PUBLIC ../src/planning/include/planner/motion_planner/optimizers/openspace_optimizer/)
target_include_directories(openspace_computation_analysis  PUBLIC ../src/planning/include/)
target_link_libraries(openspace_computation_analysis
  openspace_optimizer
  msquare_pnc_planning
)

include_directories(
  include
  /usr/include/python2.7
)

add_executable(a_star_test 
    a_star_test.cpp
    ../../src/planning/src/common/sbp_strategy.cpp
    ../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.cc
    ../../src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.cc
    ../../src/planning/src/common/priority_obs.cpp
)
target_include_directories(a_star_test PUBLIC ../src/planning/include/planner/motion_planner/optimizers/openspace_optimizer/)
target_include_directories(a_star_test  PUBLIC ../src/planning/include/)
target_link_libraries(a_star_test
  openspace_optimizer
  msquare_pnc_planning
  /usr/lib/x86_64-linux-gnu/libpython2.7.so
)
