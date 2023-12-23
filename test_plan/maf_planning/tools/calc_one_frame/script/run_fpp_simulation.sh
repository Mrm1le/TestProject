#!/bin/bash

export PWD="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd -P )"
export DATA_DIR=/home/ros/Downloads
export CALIB_DIR=$DATA_DIR/calib
INPUT_BAG=$DATA_DIR/`echo $CASE_FILE | awk -F '/' '{print $NF}'`

if [ ! -L /model ]; then
  ln -s /opt/maf_planning/resource/model /model
fi

LOG_DIR=$DATA_DIR/logs/${POD_NAME}/np_planning
mkdir -p ${LOG_DIR}
rm -rf ${LOG_DIR}/planning.$MSIM_ID.log

start_time=0
bash $PWD/run_fpp.sh $INPUT_BAG $start_time > ${LOG_DIR}/planning.$MSIM_ID.log 2>&1

errno=$?
rm -rf $DATA_DIR/errno.$MSIM_ID.txt
echo $errno > $DATA_DIR/errno.$MSIM_ID.txt
