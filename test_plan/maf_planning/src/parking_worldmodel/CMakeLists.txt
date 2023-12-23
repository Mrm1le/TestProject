cmake_minimum_required(VERSION 3.2)
project(parking_worldmodel)

get_property(CONANBUILDINFO_FILE GLOBAL PROPERTY "CONANBUILDINFO_FILE" ) 
include(${CONANBUILDINFO_FILE})

# Use C++ 14 standard
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-fPIC)

# Define Platform
set (LIBRARY_PLATFORM ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "!!!!! Current PLATFORM is ${LIBRARY_PLATFORM}")

# Find MDK package
if(MPH_FRAMEWORK_ON)
    include(mmv REQUIRED)   
    mmv_init()
endif()

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

file(GLOB_RECURSE CPP_FILES src/*.cpp)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/maf_interface/include/)
add_library (${PROJECT_NAME} STATIC ${CPP_FILES})

# All src files
target_include_directories( ${PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/../common/include
${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty
${CMAKE_CURRENT_SOURCE_DIR}/worldmodel_pec/interface
# ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/mtime/mtime_core/include
# ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/mlog/mlog_core/include
# ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/mjson/include
)
file(GLOB src_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# maf interface
# add_subdirectory(../../thirdparty/mtaskflow thirdparty/mtaskflow)
# add_subdirectory(../../thirdparty/mlog thirdparty/mlog)
# add_subdirectory(../../include/maf_interface maf_interface)
# add_subdirectory(../../thirdparty/mtime thirdparty/mtime)

# WorldModel executable
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/worldmodel_pec)

target_include_directories(${PROJECT_NAME} PRIVATE
    # maf_interface
    # mtaskflow
    worldmodel_pec
    # ${MLOG_NAMESPACE}::mlog_core 
    # ${MTIME_NAMESPACE}::mtime_core
    ${CONAN_INCLUDE_DIRS}
    ${CONAN_LIB_DIRS}
  )

include_directories(${CONAN_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    # maf_interface
    # mtaskflow
    worldmodel_pec
    # ${MLOG_NAMESPACE}::mlog_core 
    # ${MTIME_NAMESPACE}::mtime_core 
    ${CONAN_LIBS}
  )