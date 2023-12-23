#!/bin/bash

set -euo pipefail

#export MFR_IP="127.0.0.1"

export PWD="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd -P )"
export DEPLOY_ROOT=$(realpath ${PWD}/../../..)
# export DEPLOY_ROOT=$(realpath ${PWD}/../../../build/deploy)
echo "DEPLOY_ROOT: ${DEPLOY_ROOT}"
ls -alh ${DEPLOY_ROOT}/common/bin/mfrlaunch

export MODULE_ROOT_DIR=$(realpath ${PWD}/..)
echo "MODULE_ROOT_DIR: ${MODULE_ROOT_DIR}"

export COMMON_LIB_DIR=${DEPLOY_ROOT}/common/lib
echo "COMMON_LIB_DIR: ${COMMON_LIB_DIR}"

export MODULE_LIB_DIR=${MODULE_ROOT_DIR}/lib
# export MODULE_LIB_DIR=${DEPLOY_ROOT}/..
echo "MODULE_LIB_DIR: ${MODULE_LIB_DIR}"

if [ -d /root/.iso_compiler/v2/cuda-x86_11.1 ]; then
    echo "HAVE CUDA 11"
fi

if [ -d /root/.iso_compiler/v2/cuda-x86_64-11.1 ]; then
    export CUDA_X86_64_LIB_DIR=/root/.iso_compiler/v2/cuda-x86_64-11.1/usr/local/cuda-11.1/targets/x86_64-linux/lib
else
    export CUDA_X86_64_LIB_DIR=/root/.iso_compiler/v2/cuda-x86_64-10.2/usr/local/cuda-10.2/targets/x86_64-linux/lib
fi

export LD_LIBRARY_PATH=${MODULE_LIB_DIR}:${COMMON_LIB_DIR}:${LD_LIBRARY_PATH:-}
export LD_LIBRARY_PATH=${CUDA_X86_64_LIB_DIR}:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/utopia/libtorch/lib/

if [ -d /root/.iso_compiler/v2/cuda-x86_64-11.1 ]; then
    echo "RUN ON SIMULATION USING CUDA 11"
    export LD_LIBRARY_PATH=$(echo ${LD_LIBRARY_PATH} | sed 's/cuda-10.2/cuda-11.1/g' | sed 's/cuda-x86_64-10.2/cuda-x86_64-11.1/g')
fi

echo "LD_LIBRARY_PATH: ${LD_LIBRARY_PATH}"

export COMMON_BIN_DIR=${DEPLOY_ROOT}/common/bin
export MODULE_BIN_DIR=${MODULE_ROOT_DIR}/bin
export PATH=${MODULE_BIN_DIR}:${COMMON_BIN_DIR}:${PATH:-}
echo "PATH: ${PATH}"
