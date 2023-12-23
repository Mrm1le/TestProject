cmake_minimum_required(VERSION 2.8)
project(worldmodel_pec)

get_property(CONANBUILDINFO_FILE GLOBAL PROPERTY "CONANBUILDINFO_FILE" ) 
include(${CONANBUILDINFO_FILE})

# =================== set build flags =============================
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=c++11 -g -O3 -Wall -fPIC -pthread -Werror=non-virtual-dtor")

# =================== include cmake macros and scripts =============================
include(cmake/sys_arch.cmake)

# =================== include directories and link directories =============================


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  # ../../../thirdparty
  # ../../../thirdparty/eigen
  utils
  ../include
  ../../../include/maf_interface/include/
  ../../common/include/)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "***include dir='${dir}'")
endforeach()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY LINK_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "***lib dir='${dir}'")
endforeach()


# =================== set output directory =============================

# set(OUTPUT_MAIN_DIR ${PROJECT_BINARY_DIR}/${PKM_SYSTEM_ARCH})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/lib)

# =================== build libraries and executables =============================
# 是否打开可视化以调试
option(ENABLE_DEBUG_VIS "Open Debug Visualization" OFF)

set(PEC_IMPL_FILES
	 src/interface_impl/worldmodel_pec_impl.cpp 
)

set(PEC_IMPL_FILES
	 src/interface_impl/worldmodel_pec_impl.cpp 
)

set(PEC_PARKING_SLOT_FILES
	 src/slot_release/parking_slot_element/parking_slot_element.cpp
   src/slot_release/parameters/slot_release_params.cpp
   src/slot_release/parameters/slot_release_vehicle_param.cpp
   src/slot_release/schmitt_trigger/schmittTrigger.cpp
	 src/slot_release/slot_release_operator/slot_release_manager.cpp
   src/slot_release/slot_release_operator/slot_release_algorithm_operator.cpp
)


if(ENABLE_DEBUG_VIS)
  add_definitions(-DDEBUG_VIZ)
endif()


add_library(${PROJECT_NAME} STATIC  
	${PEC_IMPL_FILES}
	${PEC_PARKING_SLOT_FILES}
)
	

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/interface/  ${CONAN_INCLUDE_DIRS} ${CONAN_LIB_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE
            # maf_interface
					  #opencv
            # ${MLOG_NAMESPACE}::mlog_core 
            # ${MTIME_NAMESPACE}::mtime_core 
            # mjson
            ${dependencies_LIBS}
            ${CONAN_LIBS})