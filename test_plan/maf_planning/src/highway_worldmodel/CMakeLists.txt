project(highway_worldmodel)

get_property(CONANBUILDINFO_FILE GLOBAL PROPERTY "CONANBUILDINFO_FILE" ) 
include(${CONANBUILDINFO_FILE})

# find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/maf_interface/include/)
add_library(${PROJECT_NAME} STATIC
  src/ddmap_generator_task.cc
  src/ddmap_generator.cpp
  src/fusion_filter.cpp
  src/fusion_task_impl.cc
  src/common/logging.cpp
  src/common/timing.cpp
  src/utility/coordinate.cc
  src/utility/geometry.cc
)

# include_directories(${EIGEN3_INCLUDE_DIR})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CONAN_INCLUDE_DIRS}
  ${CONAN_LIB_DIRS})

include_directories(${CONAN_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  # maf_interface
  # mtaskflow
  # ${MLOG_NAMESPACE}::mlog_core 
  # ${MTIME_NAMESPACE}::mtime_core
  # eigen
  ${CONAN_LIBS}
)

