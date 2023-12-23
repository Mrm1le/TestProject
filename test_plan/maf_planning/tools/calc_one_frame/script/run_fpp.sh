#!/bin/bash


bag=$1
export PWD="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd -P )"
PLANNING_PATH=$(cd $PWD/../../..; pwd)
export deploy_dir="build/deploy/modules/planning"
export RESOURCE_PATH=${PLANNING_PATH}/${deploy_dir}/resource/config/
export NPP_RESOURCE_PATH=${PLANNING_PATH}/${deploy_dir}/resource/config/
export CPP_MACHINE_PATH=${PLANNING_PATH}/${deploy_dir}/resource/mfr_nodes/
export APA_PLANNING_RESOURCE_PATH=${PLANNING_PATH}/${deploy_dir}/resource/

${PLANNING_PATH}/build/bin/fpp --calc-one-frame ${bag} -is-closed-loop=1 -run-retimer=1 -start-time=$2 --is_pnp=1
