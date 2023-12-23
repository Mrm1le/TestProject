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
export PLANNING_PACKAGE_PATH=${PROJECT_ROOT_DIR}/modules/planning
export LD_LIBRARY_PATH=${PLANNING_PACKAGE_PATH}/lib:${LD_LIBRARY_PATH}
export LDP_PLANNING_RESOURCE_PATH=${PLANNING_PACKAGE_PATH}/resource
export LDP_PLANNING_MFR_LIBRARY=${PLANNING_PACKAGE_PATH}/lib/libldp_mfr_node.so
export MFR_FLAG_MONITOR_LEVEL=1
#############################################################################

./run_LDPPlanning.sh

