cmake_minimum_required(VERSION 2.8.3)
project(planning_mfr)

if (WITH_MFF) 
    add_compile_options(-DMFF_ENABLE)
else()
    list(REMOVE_ITEM generate_mfr_LIBRARIES $ENV{MPH_PROJECT_ROOT}/modules/dependants/mfr_common/build/mfrproto/code/bin/libmff_pnc_lib.so)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-O2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=gnu++14)

if(COMMAND catkin_package)
  set(BUILD_MODE "catkin_build")
  message(STATUS "BUILD_MODE: catkin_build")
else()
  set(BUILD_MODE "normal_build")
  message(STATUS "BUILD_MODE: normal_build")
endif()

if (ON_QNX)
else()
  add_compile_options(-fPIC)
  set(MACHINE_ALREADY_EXIST ON)
  set(MLOG_ENABLE_MFR ON)
  set(MTIME_ENABLE_MFR ON)
endif()

set(MSD_CORE_ARGS
  ${MSD_PLATFORM_ARGS}
  "-DMACHINE_ALREADY_EXIST=ON"
  "-DMFRUNTIME_BUILD_PATH=${MFRUNTIME_BUILD_PATH}"
  "-DMFRUNTIME_PATH=${MFRUNTIME_PATH}"
  "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
)

if(${BUILD_MODE} STREQUAL "catkin_build")
  find_package(catkin REQUIRED COMPONENTS
    roscpp

    momenta_mfrmsgs
    generate_mfr_cpp_struct
    generate_ros_mfr
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS momenta_utils
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()
  
add_subdirectory(thirdparty/planning)
ExternalProject_Get_Property(planning_mfr binary_dir)

get_filename_component(DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)

include_directories(
  include
  ${binary_dir}/install/include
  ${binary_dir}/install/include/maf_interface/include
  ${generate_mfr_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/nlohmann_json
)

# message(FATAL_ERROR "======${binary_dir}/install/include")

file(GLOB CPP_FILES src/*.cpp)
foreach(TARGET_SOURCE ${CPP_FILES})
  string(REGEX REPLACE ".+/(.+)\\..*" "\\1" TARGET ${TARGET_SOURCE})

  if(${TARGET} STREQUAL "planning_mfr_node" OR ${TARGET} STREQUAL "ldp_mfr_node" OR ${TARGET} STREQUAL "sbp_mfr_node")
    add_library(${TARGET} SHARED ${TARGET_SOURCE})
  else()
    if(${BUILD_MODE} STREQUAL "catkin_build")
      add_executable(${TARGET} ${TARGET_SOURCE})
    else()
      continue()
    endif()
  endif()

  target_include_directories(${TARGET} PRIVATE
    include
    ${generate_mfr_INCLUDE_DIRS}
    ${MFR_CPP_STRUCT_CONVERT_INCLUDE_DIR}
  )
  target_link_libraries(${TARGET} PRIVATE
    planning_mfr_basic
    ${CONAN_LIBS_MF_MFRUNTIME}
    ${generate_mfr_LIBRARIES}
    ${CONAN_LIBS_MF_MLOG_PUBLISHER}
    ${CONAN_LIBS_MF_MTIME_CUSTOMIZE_TIMELINE}
    ${CONAN_LIBS_MF_MTIME_CORE}
  )

  if(${BUILD_MODE} STREQUAL "catkin_build")
    target_link_libraries(${TARGET} PRIVATE
      ${catkin_LIBRARIES}
    )
    install(
      TARGETS ${TARGET}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  else()
    install(
      TARGETS ${TARGET}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  endif()
endforeach()

if(${BUILD_MODE} STREQUAL "catkin_build")
  install(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
else()
  install(
    DIRECTORY launch/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/launch
  )
endif()

