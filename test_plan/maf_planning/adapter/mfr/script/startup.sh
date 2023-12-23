#!/usr/bin/env bash

#############################################################################
# These environment variables need to be defined in system startup scripts
#############################################################################
# PROJECT_ROOT_DIR
# LD_LIBRARY_PATH
# WHICH_CAR
# MFR_IP
# CONTAINER_HOME
# CAM_CALIB_DIR
# LOGS_DIR
# COREDUMP_DIR
# MFR_TOOLS_BIN_PATH
#############################################################################

#############################################################################
# The environment variables define at planning startup
#############################################################################
export APA_PLANNING_PACKAGE_PATH=${PROJECT_ROOT_DIR}/modules/apa_planning
export LD_LIBRARY_PATH=${APA_PLANNING_PACKAGE_PATH}/lib:${LD_LIBRARY_PATH}
export APA_PLANNING_RESOURCE_PATH=${APA_PLANNING_PACKAGE_PATH}/resource
export APA_WORLDMODEL_RESOURCE_PATH=${APA_PLANNING_PACKAGE_PATH}/resource
export APA_PLANNING_MFR_LIBRARY=${APA_PLANNING_PACKAGE_PATH}/lib/libapa_planning_mfr_node.so
export EHR_WORLDMODEL_LOG_PATH=${LOGS_DIR}/ehr_logs
export APA_MSD_WORLDMODEL_MDK_CONFIG=${APA_WORLDMODEL_RESOURCE_PATH}/config.toml
export MFR_FLAG_MONITOR_LEVEL=1
export MSD_CAR_NAME=`hostname`

if [[ "$(echo ${MSD_CAR_NAME}  | grep "MKZ")" != "" ]]
then
    echo "devcar MKZ enable snapshot"
    export MFC_THREAD_SETTING=${APA_PLANNING_RESOURCE_PATH}/config/mtaskflow_config.json
    export MFR_SNAPSHOT_CONFIG_KEY=mfr_snapshot_planning
    export MFR_SNAPSHOT_MODE=realcar
elif [[ "$(echo ${MSD_CAR_NAME}  | grep "RX5")" != "" ]]
then
    echo "devcar RX5 enable snapshot"
    export MFC_THREAD_SETTING=${APA_PLANNING_RESOURCE_PATH}/config/mtaskflow_config.json
    export MFR_SNAPSHOT_CONFIG_KEY=mfr_snapshot_planning
    export MFR_SNAPSHOT_MODE=realcar
else
    echo "${MSD_CAR_NAME} disable snapshot"
fi
#############################################################################

set -euo pipefail

export PWD="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd -P )"
. ${PWD}/runtime_env.sh

mkdir -p ${EHR_WORLDMODEL_LOG_PATH}
mfrlaunch_yaml_file=${APA_PLANNING_PACKAGE_PATH}/launch/planning_mfr_node.yaml
${MFR_TOOLS_BIN_PATH}/mfrlaunch --wait file -c ${mfrlaunch_yaml_file} -m mfrrpc://127.0.0.1:11300

