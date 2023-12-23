#!/bin/bash
source /script/prepare.sh

CURRENT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "CURRENT_DIR: " ${CURRENT_DIR}
echo "PLANNING_MODE: " ${PLANNING_MODE}
${CURRENT_DIR}/start_ldp.sh &
${CURRENT_DIR}/start_sbp.sh &
${CURRENT_DIR}/start.sh -f planning_scene_type:=${PLANNING_MODE}