cmake_minimum_required(VERSION 2.8.3)
project (pnc_test)
add_compile_options(-std=c++14)
add_compile_options(-w)

add_subdirectory(unittest)
add_subdirectory(openspace_computation_analysis)
# add_subdirectory(bag_recurrent)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# add_subdirectory(openspace_computation_analysis)
# set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# add_executable(mpc_traj_planner
#   mpc_traj_planner_test.cpp
# )

# target_link_libraries(mpc_traj_planner
#   ${PROJECT_NAME}_GJK
#   ${PROJECT_NAME}_PLANNER
#   libtraj_plannermpc.so
#   ${catkin_LIBRARIES}
#   -ldl
# )

# add_executable(lane_keeping_lateral_test
#   lane_keeping_lateral_mpc_planner_test.cpp
# )

# target_link_libraries(lane_keeping_lateral_test
#   liblane_keeping_lateralmpc.so 
#   ${catkin_LIBRARIES}
#   -ldl
# )

#set(CMAKE_BUILD_TYPE RelWithDebInfo)

#find_package(GTest)
#add_executable(general_test src/main.cc src/path_planner_test.cc)
#add_dependencies(general_test msquare_pnc_planning)
#target_include_directories(general_test PRIVATE ${GTEST_INCLUDE_DIRS})
#target_link_libraries(general_test PRIVATE ${GTEST_LIBRARIES} pthread msquare_pnc_planning osqpstatic -ldl)

set(PLOT_SEARCH_ONLY ON)
if(PLOT_SEARCH_ONLY)
    add_library(openspace_optimizer SHARED openspace_optimizer/search_only_wrapper.cc)
else()
    add_library(openspace_optimizer SHARED openspace_optimizer/planner_wrapper.cc)
endif()

target_include_directories(
    openspace_optimizer 
    PUBLIC     ../src/planning/include/planner/motion_planner/optimizers/openspace_optimizer/
    )
target_include_directories(openspace_optimizer PUBLIC /home/ros/catkin_ws/src/maf_planning/src/planning/include/)
target_link_libraries(openspace_optimizer
   msquare_pnc_planning
)

target_include_directories(
    openspace_optimizer 
    PUBLIC     ../src/planning/include/planner/motion_planner/optimizers/openspace_optimizer/
    )
target_include_directories(openspace_optimizer PUBLIC ../src/planning/include/)
target_link_libraries(openspace_optimizer
   msquare_pnc_planning
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/maf_interface/include/)

add_library(traj_evaluation SHARED
 traj_evaluation/diff_between_sides.cpp
 traj_evaluation/diff_between_sides.h
 traj_evaluation/topsis.h
 traj_evaluation/trend.cpp
 traj_evaluation/trend.h
 traj_evaluation/evalution.h
 traj_evaluation/evalution_so.cpp
 traj_evaluation/params.cpp

 ../src/planning/include/common/math/aabox2d.h
 ../src/planning/include/common/math/line_segment2d.h
 ../src/planning/include/common/math/vec2d.h
 ../src/planning/include/common/math/box2d.h

 ../src/planning/src/common/math/aabox2d.cpp
 ../src/planning/src/common/math/line_segment2d.cpp
 ../src/planning/src/common/math/vec2d.cpp
 ../src/planning/src/common/math/box2d.cpp
 ../src/planning/src/common/math/polygon2d.cpp
 ../src/planning/src/common/math/math_utils.cpp
)

message("yaml-cpp include dir: " ${YAML_CPP_INCLUDE_DIR})
message("yaml-cpp library dir: " ${YAML_CPP_LIBRARIES})
target_include_directories(traj_evaluation PUBLIC ${YAML_CPP_INCLUDE_DIR})
target_include_directories(traj_evaluation PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(traj_evaluation PUBLIC ../src/planning/include/)
target_include_directories(traj_evaluation PUBLIC ../include/)
target_link_libraries(traj_evaluation
    ${YAML_CPP_LIBRARIES}
)

# target_include_directories(eva_exe PUBLIC ../src/planning/include/)
# target_link_libraries(eva_exe
#     msquare_pnc_planning
#     traj_evaluation
# )



# Add gtest based cpp test target and link libraries
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
add_executable(
 ${PROJECT_NAME}
 src/box2d_test.cc
 src/parking_slot_test.cc
)

if(TARGET ${PROJECT_NAME})
 target_link_libraries(${PROJECT_NAME}
   ${GTEST_LIBRARIES}
   msquare_pnc_planning
   dl
   gtest_main  # entrance program of all tests
 )
endif()

#add_executable(
#  line2d_test
#  src/line2d_test.cc
#)

# target_link_libraries(line2d_test
#  msquare_pnc_planning
#  ${GTEST_LIBRARIES}
#  gtest_main
# )
# add_executable(refline_smooth_test src/main.cc src/refline_smooth_test.cc)
# target_include_directories(refline_smooth_test PRIVATE ${GTEST_INCLUDE_DIRS})
# target_link_libraries(refline_smooth_test PRIVATE ${GTEST_LIBRARIES} pthread msquare_pnc_planning osqpstatic -ldl)
# add_executable(general_test src/main.cc src/path_planner_test.cc)
# target_include_directories(general_test PRIVATE ${GTEST_INCLUDE_DIRS})
# target_link_libraries(general_test PRIVATE ${GTEST_LIBRARIES} pthread msquare_pnc_planning osqpstatic -ldl)
# add_executable(smoother_test src/main.cc src/smoother_test.cc)
# target_include_directories(smoother_test PRIVATE ${GTEST_INCLUDE_DIRS})
# target_link_libraries(smoother_test PRIVATE ${GTEST_LIBRARIES} pthread msquare_pnc_planning osqpstatic -ldl)

# add_executable(pomdp_test src/pomdp_test.cpp)
# target_include_directories(pomdp_test PRIVATE ${GTEST_INCLUDE_DIRS})
# target_link_libraries(pomdp_test PRIVATE ${GTEST_LIBRARIES} pthread msquare_pnc_planning osqpstatic -ldl)

