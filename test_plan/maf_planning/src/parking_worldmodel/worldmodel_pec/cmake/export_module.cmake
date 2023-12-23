macro(export_header_files EXPORT_HEADER_FILES)
    execute_process(COMMAND mkdir -p ${OUTPUT_MAIN_DIR}/include)

    # the first file
    execute_process(COMMAND cp ${CMAKE_SOURCE_DIR}/${ARGV0} ${OUTPUT_MAIN_DIR}/include)
    
    # the other files
    set(list_var "${ARGN}")
    foreach(loop_var IN LISTS list_var)
        execute_process(COMMAND cp ${CMAKE_SOURCE_DIR}/${loop_var} ${OUTPUT_MAIN_DIR}/include)
    endforeach()

    # generate Config.cmake
    configure_file(cmake/export_moduleConfig.cmake.in
    "${OUTPUT_MAIN_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

endmacro()