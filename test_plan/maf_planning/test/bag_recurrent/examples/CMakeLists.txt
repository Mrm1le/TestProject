project(parking_plotter_examples)

add_executable(sbp_planner_test sbp_planner_test.cpp)
target_link_libraries(sbp_planner_test
  parking_plotter
)

add_executable(test_config test_config.cpp)
target_link_libraries(test_config
  parking_plotter
)

add_executable(test_projection test_projection.cpp)
target_link_libraries(test_projection
  parking_plotter
)
add_executable(test_sv_speed test_sv_speed.cpp)
target_link_libraries(test_sv_speed
  parking_plotter
)
