#!/bin/bash
source ${CONTAINER_AUTO_WS}/devel/setup.bash

current_path=$(cd $(dirname "$0") && pwd)
export PACKAGE_PATH=$current_path/..
export CMAKE_BINARY_PATH=${CONTAINER_AUTO_WS}/devel
export APA_PLANNING_RESOURCE_PATH=${PACKAGE_PATH}/../../resource
export APA_WORLDMODEL_RESOURCE_PATH=${PACKAGE_PATH}/../../resource
export MFR_RELEASE_PATH=$(roscd common_thirdparty && pwd)/mfr-release

export MSD_LIB_MIMALLOC_PATH=${PACKAGE_PATH}/../../thirdparty/mimalloc-release/libmimalloc.so
source $current_path/../../../start.sh $@
