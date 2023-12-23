
macro(generate_version MODULE_NAME WORLD_VERSION MAJOR_VERSION MINOR_VERSION OUT_VERSION_FILE)
    message(STATUS "Generating ${OUT_VERSION_FILE}")

    # 版本号
    set(version "${WORLD_VERSION}.${MAJOR_VERSION}.${MINOR_VERSION}")

    # 创建version.h
    execute_process(COMMAND touch ${OUT_VERSION_FILE})
    message(STATUS "touch ${OUT_VERSION_FILE}")

    # if dirty
    execute_process(COMMAND bash -c "git diff --quiet || echo dirty" OUTPUT_VARIABLE DIRTY OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(DIRTY MATCHES "dirty")
        set(IF_DIRTY 1)
    else()
        set(IF_DIRTY 0)
    endif()

    message(STATUS "version: ${WORLD_VERSION}.${MAJOR_VERSION}.${MINOR_VERSION}")
    message(STATUS "dirty: ${IF_DIRTY}")

    # pragma once
    file(WRITE ${OUT_VERSION_FILE} "#pragma once\n\n")

    # WORLD_VERSION
    string(TOUPPER ${MODULE_NAME} MODULE_NAME_UPPERCASE)
    set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_WORLD_VERSION (${WORLD_VERSION})\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${VERSION_STR})

    # MAJOR VERSION
    set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_MAJOR_VERSION (${MAJOR_VERSION})\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${VERSION_STR})

    # MINOR VERSION
    set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_MINOR_VERSION (${MINOR_VERSION})\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${VERSION_STR})

    # VERSION_IS_DIRTY
    set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_VERSION_IS_DIRTY (${IF_DIRTY})\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${VERSION_STR})

    # VERSION_STRING
    if(${IF_DIRTY})
        set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_VERSION_STRING \"${version}-dirty\"\n\n")
    else()
        set(VERSION_STR "#define ${MODULE_NAME_UPPERCASE}_VERSION_STRING \"${version}\"\n\n")
    endif()
    file(APPEND ${OUT_VERSION_FILE} ${VERSION_STR})

    # GIT_COMMIT_HASH
    execute_process(COMMAND git log -1 --pretty=format:%H OUTPUT_VARIABLE GIT_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(GIT_COMMIT ${GIT_COMMIT}${GIT_DIFF})
    message(STATUS "git commit: ${GIT_COMMIT}")
    set(GIT_COMMIT_STR "#define ${MODULE_NAME_UPPERCASE}_GIT_COMMIT_HASH \"${GIT_COMMIT}\"\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${GIT_COMMIT_STR})

    # GIT_COMMIT_DATE
    execute_process(COMMAND bash -c "git show -s --format=%ci" OUTPUT_VARIABLE GIT_COMMIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "git commit date: ${GIT_COMMIT_DATE}")
    set(GIT_COMMIT_DATE_STR "#define ${MODULE_NAME_UPPERCASE}_GIT_COMMIT_DATE \"${GIT_COMMIT_DATE}\"\n\n")
    file(APPEND ${OUT_VERSION_FILE} ${GIT_COMMIT_DATE_STR})

    message(STATUS "Generated ${OUT_VERSION_FILE}")

endmacro(generate_version)
