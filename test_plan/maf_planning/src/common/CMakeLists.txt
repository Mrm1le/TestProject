project(msd_planning_common)

get_property(CONANBUILDINFO_FILE GLOBAL PROPERTY "CONANBUILDINFO_FILE" ) 
include(${CONANBUILDINFO_FILE})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/maf_interface/include/)
add_library(${PROJECT_NAME} STATIC 
  src/logging.cpp
  src/timing.cpp
  src/sysconf.cpp
  src/inner_snapshot.cpp
  src/statistic.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${CONAN_LIBS_MF_MLOG_CORE}
  ${CONAN_LIBS_MF_MTIME_CORE}
)
