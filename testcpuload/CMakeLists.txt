
cmake_minimum_required(VERSION 3.15)

project(exam)
add_executable(example)

target_sources(example
    PUBLIC test.cpp
  )
  
target_include_directories(example
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include/
  )
target_link_directories(example PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/
)


set(absl_libs absl_strings absl_str_format_internal absl_throw_delegate absl_raw_logging_internal)

target_link_libraries(example ${absl_libs})
# target_link_libraries(example
#     absl_strings
#     absl_str_format_internal
#     absl_throw_delegate
#     absl_raw_logging_internal
# )
