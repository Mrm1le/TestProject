# include(ExternalProject)
# ExternalProject_Add(
#   glog_external
#   URL https://github.com/google/glog/archive/refs/tags/v0.5.0.tar.gz
#   PREFIX ${CMAKE_CURRENT_BINARY_DIR}/glog

#   CMAKE_ARGS "-DWITH_GTEST=OFF" -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/glog

#   # Disable install step
#   # INSTALL_COMMAND ""

#   # Wrap download, configure and build steps in a script to log output
#   LOG_DOWNLOAD ON
#   LOG_CONFIGURE ON
#   LOG_BUILD ON
# )

# ExternalProject_Get_Property(glog_external source_dir binary_dir)
# Set(OUTPUT_DIR "${binary_dir}")
# add_library(glog IMPORTED STATIC GLOBAL)
# add_dependencies(glog glog_external)

# set(GLOG_INCLUDE_DIRECTORY "${binary_dir}")
# file(MAKE_DIRECTORY ${GLOG_INCLUDE_DIRECTORY})
# target_include_directories(glog INTERFACE ${GLOG_INCLUDE_DIRECTORY})

# set(GLOG_LIB_FILE_NAME "${CMAKE_SHARED_LIBRARY_PREFIX}glog${CMAKE_SHARED_LIBRARY_SUFFIX}")
# set_target_properties(glog PROPERTIES
#   "IMPORTED_LOCATION" "${OUTPUT_DIR}/${GLOG_LIB_FILE_NAME}")
