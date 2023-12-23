cmake_minimum_required(VERSION 3.0)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

project(planning_mfr_basic)
get_filename_component(DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)

set (PLANNING_DIR ${DIR}/../../../../)

ExternalProject_Add(planning_mfr
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/
  BUILD_ALWAYS 1
  SOURCE_DIR ${PLANNING_DIR}
  CMAKE_ARGS ${MSD_CORE_ARGS}
    -DMPH_FRAMEWORK_ON=${MPH_FRAMEWORK_ON}
    -DBUILD_SHARED_LIBS=OFF
    -DCMAKE_AR=${CMAKE_AR}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_COMPILER_TARGET=${CMAKE_CXX_COMPILER_TARGET}
    -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_C_COMPILER_TARGET=${CMAKE_C_COMPILER_TARGET}
    -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}
    -DCMAKE_LINKER=${CMAKE_LINKER}
    -DCMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}
    -DCMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}
    -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_REAL_PATH}
    -DMLOG_ENABLE_PROFILING=OFF
    -DMLOG_ENABLE_MFR=${MLOG_ENABLE_MFR}    
    -DCMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}
    -DMTIME_ENABLE_MFR=${MTIME_ENABLE_MFR}
    -DMAF_COMMON_PATH=${MAF_COMMON_PATH}
    -DMAF_INTERFACE_PATH=${MAF_INTERFACE_PATH}
    -DMFR_RELEASE_PATH=${MFR_RELEASE_PATH}
    -DMFR_COMMON_PATH=${MFR_COMMON_PATH}
    -DMFR_COMMON_BUILD_PATH=${MFR_COMMON_BUILD_PATH}
)

ExternalProject_Get_Property(planning_mfr binary_dir)

add_library(planning_mfr_basic INTERFACE)
add_dependencies(planning_mfr_basic planning_mfr)
target_include_directories(planning_mfr_basic INTERFACE ${binary_dir}/install/include)
target_link_libraries(planning_mfr_basic INTERFACE ${binary_dir}/install/lib/libmsquare_pnc.so)
#target_link_libraries(planning_basic INTERFACE ${binary_dir}/install/lib/libcpp-hocon.so)

if (MPH_FRAMEWORK_ON)
  install(DIRECTORY ${binary_dir}/install/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "*"
  )
else()

if(${BUILD_MODE} STREQUAL "catkin_build")
  install(DIRECTORY ${binary_dir}/install/lib/
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    FILES_MATCHING PATTERN "*"
  )
  install(DIRECTORY ${PLANNING_DIR}/resource/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
    FILES_MATCHING PATTERN "*"
  )
else()
  install(DIRECTORY ${binary_dir}/install/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "*"
  )
  install(DIRECTORY ${PLANNING_DIR}/resource/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/resource
    FILES_MATCHING PATTERN "*"
  )
endif()

install(FILES ${PLANNING_DIR}/start.sh
  DESTINATION script
  RENAME start_planning.sh
)
install(FILES ${PLANNING_DIR}/start_ldp.sh
  DESTINATION script
  RENAME start_ldp.sh
)
install(FILES ${PLANNING_DIR}/start_sbp.sh
  DESTINATION script
  RENAME start_sbp.sh
)
endif()
