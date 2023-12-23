project (parking_unittest)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/cmake/modules)
set( EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin) 

find_package(rosbag)
message(${rosbag_INCLUDE_DIRS})

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/src/planning/include/planner/motion_planner/optimizers/openspace_optimizer/
    ${CMAKE_SOURCE_DIR}/thirdparty/googletest/googletest/include

    # rosbag topic convert
    ${rosbag_INCLUDE_DIRS}
    /home/ros/catkin_ws/devel/include/
    ${CMAKE_SOURCE_DIR}/../../devel/include/
    ${CMAKE_SOURCE_DIR}/../../build/include/
    ${CMAKE_SOURCE_DIR}/adapter/ros/include
    
)


file(GLOB_RECURSE ALL_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.c*)
message(${ALL_SRCS})
add_executable(${PROJECT_NAME} ${ALL_SRCS} src/main.cpp)
target_link_libraries(
    ${PROJECT_NAME} 
    ${rosbag_LIBRARIES}
    msquare_pnc_planning 
    gtest 
    osqpstatic 
    -ldl

    # rosbag topic convert
    /home/ros/catkin_ws/devel/lib/libgenerate_ros_cpp_struct.so
)


if (CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    append_coverage_compiler_flags()

    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")

    set(COVERAGE_EXCLUDES 
        "${CMAKE_SOURCE_DIR}/src/common/*"
        "${CMAKE_SOURCE_DIR}/src/common/config/*"
        "${CMAKE_SOURCE_DIR}/src/common/math/*"
        "${CMAKE_SOURCE_DIR}/src/common/path/*"
        "${CMAKE_SOURCE_DIR}/src/common/refline/*"
        "${CMAKE_SOURCE_DIR}/src/common/speed/*"
        "${CMAKE_SOURCE_DIR}/src/common/trajectory/*"
        "${CMAKE_SOURCE_DIR}/src/common/utils/*"
        "${CMAKE_SOURCE_DIR}/src/common/vehicle_model/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/arbitrators/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/scenarios/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/tasks/*"
        "${CMAKE_SOURCE_DIR}/src/planning/include/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/refline/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/speed/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/path/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/trajectory/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/vehicle_model/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/config/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/math/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/utils/*"
    )
    set(COVERAGE_EXTRACTS 
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/parking*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/scenario*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/common/sbp_strategy.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/parking*.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/optimizers/openspace_optimizer/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/openspace_motion_planner/*"
        "${CMAKE_SOURCE_DIR}/src/planning/include/common/*.h"
        "${CMAKE_SOURCE_DIR}/src/planning/include/planner/behavior_planner/deciders/apf_decider.h"

        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/parking_longitudinal_behavior_planner.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/parking_behavior_planner.cpp.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/apa_openspace_decider.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/apf_decider.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/collision_checker.cppp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/leader_decider.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/apoa_openspace_decider.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/rpa_straight_openspace_decider.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/openspace_utils.cpp"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/behavior_planner/deciders/openspace_decider_interface.cpp"

    )

    setup_target_for_coverage_lcov(
            NAME ${PROJECT_NAME}_coverage
            EXECUTABLE ${PROJECT_NAME}
    )
endif ()

