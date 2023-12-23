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

export APA_PLANNING_PACKAGE_PATH=${PROJECT_ROOT_DIR}/modules/planning/CPPlanningRoot
export LD_LIBRARY_PATH=${APA_PLANNING_PACKAGE_PATH}/lib:${LD_LIBRARY_PATH}
export APA_PLANNING_RESOURCE_PATH=${APA_PLANNING_PACKAGE_PATH}/resource
export APA_WORLDMODEL_RESOURCE_PATH=${APA_PLANNING_PACKAGE_PATH}/resource
export PLANNING_BIN_PATH=${APA_PLANNING_PACKAGE_PATH}/bin
export EHR_WORLDMODEL_LOG_PATH=${LOG_CONFIG_DIR}/ehr_logs
export APA_MSD_WORLDMODEL_MDK_CONFIG=${APA_WORLDMODEL_RESOURCE_PATH}/config.toml
export MFR_FLAG_MONITOR_LEVEL=1
#############################################################################

mkdir -p ${EHR_WORLDMODEL_LOG_PATH}
./run_CPPlanning.sh

