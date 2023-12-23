# set dependencies path
macro(set_dependencies_dir depds_dir)
    if(NOT EXISTS ${depds_dir})
        message(FATAL_ERROR "${depds_dir} does not exist" )
    endif()

    set(parking_localization_dependencies_DIR ${depds_dir} )
    message(STATUS "parking_localization_dependencies_DIR=${depds_dir}")
endmacro()

# find package
macro(find_dependency_package var_name)
    if(NOT EXISTS ${parking_localization_dependencies_DIR}/${var_name}/)
        message(FATAL_ERROR "${parking_localization_dependencies_DIR}/${var_name} does not exist" )
    endif()

    set(CMAKE_PREFIX_PATH 
    	${parking_localization_dependencies_DIR}/${var_name}/
    	${CMAKE_PREFIX_PATH}
    )
    find_package(${var_name} REQUIRED)
    include_directories(${${var_name}_INCLUDE_DIRS})
    link_directories(${${var_name}_LIB_DIRS})
    set(dependencies_LIBS ${${var_name}_LIBS} ${dependencies_LIBS})
endmacro()


# find thirdparty
macro(find_dependency_thirdparty var_name)
    if(NOT EXISTS ${parking_localization_dependencies_DIR}/thirdparty/${var_name}/)
        message(FATAL_ERROR "${parking_localization_dependencies_DIR}/${var_name} does not exist" )
    endif()

    set(CMAKE_PREFIX_PATH 
    	${parking_localization_dependencies_DIR}/thirdparty/${var_name}/
    	${CMAKE_PREFIX_PATH}
        
    )
    find_package(${var_name} REQUIRED)
    include_directories(${${var_name}_INCLUDE_DIRS})
    link_directories(${${var_name}_LIB_DIRS})
    set(dependencies_LIBS ${${var_name}_LIBS} ${dependencies_LIBS})
endmacro()
