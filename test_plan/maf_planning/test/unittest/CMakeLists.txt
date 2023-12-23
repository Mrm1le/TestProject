project (pnc_unittest)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/cmake/modules)
file(GLOB_RECURSE CPP_FILES src/*test.c* ${CMAKE_CURRENT_LIST_DIR}/src/*.cc)

#find_package(Ceres REQUIRED)

include_directories(${CONAN_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${CPP_FILES} 
    ${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/path_planner_ceres/path_planner_ceres.cpp
    ${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/path_planner_ceres/path_planner_decider.cpp
    ${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/path_planner_ceres/lateral_active_avoid.cpp)
target_link_libraries(${PROJECT_NAME} 
    gtest
    msquare_pnc_planning
    ${CONAN_LIBS}
)
if (CMAKE_SYSTEM_NAME STREQUAL "QNX")
    target_link_libraries(${PROJECT_NAME} socket atomic c m)
else()
    target_link_libraries(${PROJECT_NAME} pthread rt dl)
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE TEST_DATA_DIR="data/")

return()

if (CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    append_coverage_compiler_flags()

    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O0 -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")

    #set(COVERAGE_EXCLUDES "src/*")
    # set(COVERAGE_EXTRACTS "${CMAKE_SOURCE_DIR}/src/*")
    set(COVERAGE_EXTRACTS
        "${CMAKE_SOURCE_DIR}/src/planning/include/common/obstacle_process_utils.h"

        "${CMAKE_SOURCE_DIR}/src/planning/include/planner/motion_planner/path_planner_ceres/*"
        "${CMAKE_SOURCE_DIR}/src/planning/include/planner/motion_planner/planner_cubic_spline/*"
        "${CMAKE_SOURCE_DIR}/src/planning/src/planner/motion_planner/path_planner_ceres/*.cpp"
    )
    
    setup_target_for_coverage_lcov(
            NAME coverage
            EXECUTABLE ${PROJECT_NAME}
    )
endif ()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data
  DESTINATION ../test/unittest
  FILES_MATCHING PATTERN "*"
)
