project(msquare_pnc_sample)
set(CMAKE_CXX_STANDARD 11)

file(GLOB CPP_FILES src/*.cpp)

foreach(SAMPLE_SOURCE ${CPP_FILES})
  string(REGEX REPLACE ".+/(.+)\\..*" "\\1" SAMPLE_NAME ${SAMPLE_SOURCE})
  add_executable(${SAMPLE_NAME} ${SAMPLE_SOURCE})
  if (CMAKE_SYSTEM_NAME STREQUAL "QNX")
    target_link_libraries(${SAMPLE_NAME} msquare_pnc pthread mjson mtime)
  else()
    target_link_libraries(${SAMPLE_NAME} msquare_pnc mjson mtime)
  endif()
endforeach(SAMPLE_SOURCE ${CPP_FILES})

