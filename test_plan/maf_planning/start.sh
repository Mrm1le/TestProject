#!/bin/bash
echo "WHICH_CAR: " ${WHICH_CAR}
echo "CAM_CALIB_DIR: " ${CAM_CALIB_DIR}
echo "CMAKE_BINARY_PATH: " ${CMAKE_BINARY_PATH}
echo "PACKAGE_PATH: " ${PACKAGE_PATH}
echo "APA_PLANNING_RESOURCE_PATH: " ${APA_PLANNING_RESOURCE_PATH}
echo "MFR_RELEASE_PATH: " ${MFR_RELEASE_PATH}
echo "MSD_LIB_MIMALLOC_PATH" ${MSD_LIB_MIMALLOC_PATH}

export APA_PLANNING_MFR_LIBRARY=${CMAKE_BINARY_PATH}/lib/libplanning_mfr_node.so
mfrlaunch_yaml_file=$PACKAGE_PATH/launch/planning_mfr_node.yaml

export LD_LIBRARY_PATH=${MFR_RELEASE_PATH}/lib/Linux-x86_64-gcc5.4:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${CMAKE_BINARY_PATH}/mfrproto/code/bin:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${MFR_RELEASE_PATH}/thirdparty/protobuf_libs/Linux-x86_64-gcc5.4/lib:${LD_LIBRARY_PATH}

export LD_PRELOAD=${MSD_LIB_MIMALLOC_PATH}

if [ -d "/script" ]; then
    source /script/prepare.sh
fi

export EHR_WORLDMODEL_LOG_PATH=/home/ros/Downloads/ehr_logs
mkdir -p ${EHR_WORLDMODEL_LOG_PATH}
export APA_MSD_WORLDMODEL_MDK_CONFIG=${APA_WORLDMODEL_RESOURCE_PATH}/config.toml
export MFR_FLAG_MONITOR_LEVEL=1

echo "EHR_WORLDMODEL_LOG_PATH: " ${EHR_WORLDMODEL_LOG_PATH}
echo "APA_MSD_WORLDMODEL_MDK_CONFIG: " ${APA_MSD_WORLDMODEL_MDK_CONFIG}

cd ${MFR_RELEASE_PATH}/tools/mfrtools/bin/Linux-x86_64-gcc5.4

wait_interval=600
while true;do
  echo "check if mfrmaster start, seconds remain: " $wait_interval
  let wait_interval=wait_interval-1
  if [ ${wait_interval} -lt 0 ];then
    echo "mfrmaster did not start during 10 min check!"
    break
  fi
  res=`netstat -npl | grep 11300 | wc -l`
  if [ $res -ne 0 ];then
    sleep 10
    echo "mfrmaster has been started, begin to launch mfr node"
    ./mfrlaunch file -c ${mfrlaunch_yaml_file} -m mfrrpc://127.0.0.1:11300 $@
    break
  fi
  sleep 1
done
