cmake_minimum_required(VERSION 3.0)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
project(apa_msquare_pnc)

include($ENV{MF_SYSTEM_ROOT_DIR}/package/utils.cmake)
activate_common_configuration()
auto_build_type_and_compile_flags()
configure_output_directories()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CLANG_COMPILER ON)
    message(STATUS "CLANG_FOUNDED: ON")
endif()

if(DEFINED ENV{PLANNING_COMMIT})
  execute_process(
    COMMAND bash "-c" "echo $PLANNING_COMMIT"
    OUTPUT_VARIABLE PLANNING_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
  message(STATUS "PLANNING_COMMIT(env)=" ${PLANNING_COMMIT})
else()
  execute_process(
    COMMAND git rev-parse --short=8 --vertify HEAD
    OUTPUT_VARIABLE PLANNING_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
  message(STATUS "PLANNING_COMMIT(git)=" ${PLANNING_COMMIT})
endif()

if(DEFINED ENV{PLANNING_BRANCH})
  execute_process(
    COMMAND bash "-c" "echo $PLANNING_BRANCH"
    OUTPUT_VARIABLE PLANNING_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
  message(STATUS "PLANNING_BRANCH(env)=" ${PLANNING_BRANCH})
else()
  execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    OUTPUT_VARIABLE PLANNING_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  )
  message(STATUS "PLANNING_BRANCH(git)=" ${PLANNING_BRANCH})
endif()

configure_file (
  "${CMAKE_SOURCE_DIR}/src/planning/include/common/version.h.in"
  "${CMAKE_SOURCE_DIR}/src/planning/include/common/version.h"
)
# deps from conan
if(NOT CONAN_INSTALL_ARGS)
    if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
        set(CONAN_INSTALL_ARGS "--generator cmake --profile qnx")
    else()
        set(CONAN_INSTALL_ARGS "--generator cmake")
    endif()
endif()

option(RUN_ON_CPU "run x86 cpu model" OFF)
option(BUILD_WITH_OPENCV "enable opencv for debug" OFF)
option(BUILD_WITH_HFSM_ENABLE_LOG_INTERFACE "hfsm enable log interface" ON)

set(CONAN_INSTALL_PREFIX_ARGS "MAF_VERSION=${MAF_VERSION} RUN_ON_CPU=${RUN_ON_CPU}")
conan_install()

include(${CONAN_BUILD_INFO_CMAKE})
conan_basic_setup(NO_OUTPUT_DIRS)
# include_directories(SYSTEM ${CONAN_INCLUDE_DIRS})

if(RUN_ON_CPU)
  add_definitions(-DUSE_TORCH)
else()
  add_definitions(-DUSE_INFERJAM)
  add_definitions(-DENABLE_X86_GPU_PLATFORM)
endif ()

if(BUILD_WITH_OPENCV)
  message("build with opencv")
  add_definitions(-DENABLE_OPENCV)
endif(BUILD_WITH_OPENCV)

if(BUILD_WITH_HFSM_ENABLE_LOG_INTERFACE)
  message("build with hfsm enable log interface")
  add_definitions(-DHFSM_ENABLE_LOG_INTERFACE)
endif(BUILD_WITH_HFSM_ENABLE_LOG_INTERFACE)

if(CONAN_SETTINGS_OS STREQUAL "MDC")
  message(STATUS "CONAN_SETTINGS_OS is MDC")
  if(CONAN_SETTINGS_OS_PLATFORM STREQUAL "810")
    add_definitions(-DENABLE_MDC_810_PLATFORM)
    message(STATUS "ENABLE_MDC_810_PLATFORM: ON")
  elseif(CONAN_SETTINGS_OS_PLATFORM STREQUAL "610")
    message(STATUS "ENABLE_MDC_610_PLATFORM: ON")
    if ("$ENV{BENV_ID}" MATCHES mdc610wulingshan)
      message(STATUS "ENABLE_MDC_610_WLS_PLATFORM: ON")
      add_definitions(-DENABLE_MDC_610_WLS_PLATFORM)
    elseif("$ENV{BENV_ID}" STREQUAL mdc610)
      message(STATUS "ENABLE_MDC_610_LHS_PLATFORM: ON")
      add_definitions(-DENABLE_MDC_610_LHS_PLATFORM)
    elseif ("$ENV{BENV_ID}" STREQUAL mdc610wulingshan_refcar)
      message(STATUS "ENABLE_MDC_610_WLS_PLATFORM: ON")
      add_definitions(-DENABLE_MDC_610_WLS_PLATFORM)
    elseif ("$ENV{BENV_ID}" STREQUAL mdc210wulingshan_refcar)
      message(STATUS "ENABLE_MDC_610_WLS_PLATFORM: ON")
      add_definitions(-DENABLE_MDC_610_WLS_PLATFORM)
    elseif ("$ENV{BENV_ID}" STREQUAL mdc210mfr_refcar)
      message(STATUS "ENABLE_MDC_610_WLS_PLATFORM: ON")
      add_definitions(-DENABLE_MDC_610_WLS_PLATFORM)
    else()
      message(FATAL_ERROR "ENABLE_MDC_610_PLATFORM: Error")
    endif()
  endif()
endif()

if(CONAN_SETTINGS_OS_PLATFORM STREQUAL "Orin")
  add_definitions(-DENABLE_ORIN_PLATFORM)
  message(STATUS "ENABLE_ORIN_PLATFORM: ON")
endif()

#option(BUILD_WITH_OPENCV "enable opencv for debug" OFF)
#if(BUILD_WITH_OPENCV)
#  add_definitions(-DENABLE_OPENCV)
#  find_package(OpenCV REQUIRED)
#  include_directories(${OpenCV_INCLUDE_DIRS})
#else()
#  message("build without opencv")
#endif(BUILD_WITH_OPENCV)

if(CLANG_COMPILER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-sign-compare -Wno-unused-function -Wno-reorder -Wno-parentheses -Wno-unused-value -Wno-vla-extension")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-maybe-uninitialized -Wno-unused-variable -Wno-sign-compare -Wno-unused-function -Wno-reorder -Wno-parentheses -Wno-unused-value -Wno-vla-extension")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Werror=non-virtual-dtor")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Werror=non-virtual-dtor")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-unused-function -Wno-reorder -Wno-parentheses -Wno-unused-value -Wno-vla")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-maybe-uninitialized -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-unused-function -Wno-parentheses -Wno-unused-value -Wno-vla")
endif()


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_EXTRA_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_EXTRA_FLAGS}")

function(print_all_variables)
    get_cmake_property(vars VARIABLES)
    list(SORT vars)
    message(STATUS "all variables:")
    foreach (var ${vars})
        message(STATUS "-   ${var}=${${var}}")
    endforeach()
endfunction()
# print_all_variables()

set_property(GLOBAL PROPERTY CONANBUILDINFO_FILE ${CONAN_BUILD_INFO_CMAKE})

if (${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
  add_definitions(-DFEAT_LOG_UTIL)
  add_definitions(-DFEAT_SWITCH_UTIL)
  add_definitions(-DFEAT_SHMCOM_MESSAGE)
endif()

if (WITHOUT_CPP_LOG)
  add_definitions("-DWITHOUT_CPP_LOG")
endif()

# add_definitions("-DNEW_CP") 

# MDC且是610环境中启用AP
if ("$ENV{BENV_ID}" MATCHES mdc610wulingshan OR "$ENV{BENV_ID}" MATCHES mdc210wulingshan)
  set(NODE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/adapter/ap)
  # find_package(maf_ap_definitions REQUIRED) # 这个名字一定一定一定一定一定要有
  include(${CONAN_LIB_DIRS_MAF_AP_DEFINITIONS}/maf_ap_definitions/maf_ap_definitionsConfig.cmake)
  add_ap_executable(APAPlanning ${NODE_SOURCE_DIR}/src/planning_ap_node.cpp)
  target_include_directories(
      APAPlanning
      PRIVATE
      ${PROJECT_NAME}
      ${NODE_SOURCE_DIR}/include
      ${CONAN_INCLUDE_DIRS}
      ${CONAN_LIB_DIRS}
  )
  target_link_libraries(
      APAPlanning
      PRIVATE
      ${PROJECT_NAME}
      ${CONAN_LIBS}
  )
else()
  set(NODE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/adapter/mfr)
  add_library(apa_planning_mfr_node SHARED ${NODE_SOURCE_DIR}/src/planning_mfr_node.cpp)
  target_include_directories(
      apa_planning_mfr_node
      PRIVATE
      ${PROJECT_NAME}
      ${NODE_SOURCE_DIR}/include
      ${CONAN_INCLUDE_DIRS}
      ${CONAN_LIB_DIRS}
  )
  target_link_libraries(
      apa_planning_mfr_node
      PRIVATE
      ${PROJECT_NAME}
      ${CONAN_LIBS}
  )

  add_library(apa_sbp_mfr_node SHARED ${NODE_SOURCE_DIR}/src/sbp_mfr_node.cpp)
  target_include_directories(
      apa_sbp_mfr_node
      PRIVATE
      ${PROJECT_NAME}
      ${NODE_SOURCE_DIR}/include
      ${CONAN_INCLUDE_DIRS}
      ${CONAN_LIB_DIRS}
  )
  target_link_libraries(
      apa_sbp_mfr_node
      PRIVATE
      ${PROJECT_NAME}
      ${CONAN_LIBS}
  )
endif()

file(WRITE ${PROJECT_BINARY_DIR}/empty.cpp "")
add_library (${PROJECT_NAME} SHARED ${PROJECT_BINARY_DIR}/empty.cpp )

include(CMakePrintHelpers)
if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
  if (DEFINED ENV{BUILD_SAFETY_FLAG})
    set(QNX_BUILD_SAFETY_FLAG $ENV{BUILD_SAFETY_FLAG})
  else()
    set(QNX_BUILD_SAFETY_FLAG "False")
  endif()
  cmake_print_variables(QNX_BUILD_SAFETY_FLAG)

  if (QNX_BUILD_SAFETY_FLAG STREQUAL "True")
    find_package(CUDA REQUIRED)
    include_directories(${CUDA_INCLUDE_DIRS})
    add_definitions(-DENABLE_XAVIER_SAFETY_MODEL)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${CUDA_LIBRARIES})
    message(STATUS "ENABLE_XAVIER_SAFETY_MODEL: ON")
  else()
    add_definitions(-DENABLE_XAVIER_MODEL)
    message(STATUS "ENABLE_XAVIER_MODEL: ON")
  endif()
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN")
include_directories(SYSTEM ${CONAN_INCLUDE_DIRS})

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)
add_compile_options(-DMSD_ENABLE_EXCEPTION=ON)
add_compile_options(-DMSD_ENABLE_LOGGING=ON)
add_compile_options(-DMSD_SHARED_LIBRARY=1)
add_compile_options(-DMSD_EXPORT=1)
add_compile_options(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

if(NOT CLANG_COMPILER)
    add_compile_options(-static-libstdc++)
endif()

add_compile_options(-fPIC)
add_subdirectory(thirdparty/planning_define EXCLUDE_FROM_ALL)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/maf_interface/include/)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include_v1)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include_v1/maf_interface/include/)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nlohmann_json)
add_subdirectory(src/common)
add_subdirectory(src/planning)
add_subdirectory(src/parking_worldmodel)
add_subdirectory(src/highway_worldmodel)

target_include_directories(${PROJECT_NAME} INTERFACE include)
target_include_directories(${PROJECT_NAME} INTERFACE include_v1)
target_link_libraries(${PROJECT_NAME} PRIVATE
  -Wl,-whole-archive
  msd_planning_common
  parking_worldmodel
  highway_worldmodel
  msquare_pnc_planning
  -Wl,-no-whole-archive
  ${CONAN_LIBS}
  )

if(BUILD_WITH_OPENCV)
  target_link_libraries(apa_planning_mfr_node PUBLIC ${OpenCV_LIBS})
endif(BUILD_WITH_OPENCV)

if ("$ENV{BENV_ID}" MATCHES mdc610wulingshan OR "$ENV{BENV_ID}"  MATCHES mdc210wulingshan)
  # AP和MFR不同，输出目录为install/SwcRoot中，因此要将算法库、配置文件安装到里面
  set(NODE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/adapter/ap)
  install(TARGETS ${PROJECT_NAME} DESTINATION APAPlanningRoot/lib)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/resource/
    DESTINATION APAPlanningRoot/resource
    FILES_MATCHING PATTERN "*")
  install(DIRECTORY
      DESTINATION APAPlanningRoot/lib
      FILES_MATCHING PATTERN "*"
  )
else()
  install(TARGETS apa_planning_mfr_node DESTINATION lib)
  install(TARGETS apa_sbp_mfr_node DESTINATION lib)

  install(TARGETS ${PROJECT_NAME} DESTINATION lib)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/resource/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/resource
    FILES_MATCHING PATTERN "*")
  install(DIRECTORY
    ${NODE_SOURCE_DIR}/launch/
    DESTINATION launch
  )
  install(FILES
    ${NODE_SOURCE_DIR}/script/startup.sh
    ${NODE_SOURCE_DIR}/script/runtime_env.sh
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                GROUP_READ GROUP_WRITE GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
    DESTINATION scripts
  )
endif()

set(PNC_ENABLE_TEST OFF)
message(STATUS "PNC_ENABLE_TEST=" ${PNC_ENABLE_TEST})
if (PNC_ENABLE_TEST)
  # add_subdirectory(thirdparty/googletest EXCLUDE_FROM_ALL)
  add_subdirectory(test)
endif()

# add_subdirectory(test)

option (RELWITHDEBINFO_FORCE_OPTIMIZATION_O0 "RelWithDebInfo force optimization O0" OFF)
if (RELWITHDEBINFO_FORCE_OPTIMIZATION_O0)
  message (STATUS "RelWithDebInfo force optimization O0")
  string(REPLACE "-O2" "-O0" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif ()

# set(CMAKE_BUILD_TYPE RelWithDebInfo)

#set(CI_BUILD ON) # TODO remove me!
option (CI_BUILD "build ci tools" OFF)
if (CI_BUILD)
  message (STATUS "include tools")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error")
  add_subdirectory (tools)
endif ()

