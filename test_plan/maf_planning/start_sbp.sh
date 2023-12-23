#!/bin/bash
echo "CMAKE_BINARY_PATH: " ${CMAKE_BINARY_PATH}
echo "PACKAGE_PATH: " ${PACKAGE_PATH}
echo "SBP_PLANNING_RESOURCE_PATH: " ${SBP_PLANNING_RESOURCE_PATH}
echo "MFR_RELEASE_PATH: " ${MFR_RELEASE_PATH}
echo "MSD_LIB_MIMALLOC_PATH" ${MSD_LIB_MIMALLOC_PATH}

export SBP_PLANNING_MFR_LIBRARY=${CMAKE_BINARY_PATH}/lib/libsbp_mfr_node.so
mfrlaunch_yaml_file=$PACKAGE_PATH/launch/sbp_mfr_node.yaml

export LD_LIBRARY_PATH=${MFR_RELEASE_PATH}/lib/Linux-x86_64-gcc5.4:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${CMAKE_BINARY_PATH}/mfrproto/code/bin:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=${MFR_RELEASE_PATH}/thirdparty/protobuf_libs/Linux-x86_64-gcc5.4/lib:${LD_LIBRARY_PATH}

export LD_PRELOAD=${MSD_LIB_MIMALLOC_PATH}

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
