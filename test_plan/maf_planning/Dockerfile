# syntax=docker/dockerfile:experimental
ARG MSQUARE_PNC_BASE=artifactory.momenta.works/docker-mpilot-highway-dev/for-new-planner-build:v1.0.2
ARG IMAGE_MOMENTA_COMMON=artifactory.momenta.works/docker-mpilot-parking/maf_common:dev-20210615.0
ARG RUNTIME_BASE=artifactory.momenta.works/docker-mpilot-highway-dev/maf_planning:for-new-lat-planner-runtime-v1.1

# ========================================
    
FROM ${IMAGE_MOMENTA_COMMON} as dep_proper
# ====================================== build stage
FROM ${MSQUARE_PNC_BASE} AS build

COPY . ${CONTAINER_AUTO_WS}/src/maf_planning

COPY --from=dep_proper ${CONTAINER_AUTO_WS}/ ${CONTAINER_AUTO_WS}/

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && source ${CONTAINER_AUTO_WS}/install/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -C ${CONTAINER_AUTO_WS} -DENABLE_MSIM_PROVIDER=OFF -j4 install"

RUN rm -rf ${CONTAINER_AUTO_WS}/install/include \
    ${CONTAINER_AUTO_WS}/install/mfrproto/code/include \
    ${CONTAINER_AUTO_WS}/install/mfrproto/code/py \
    ${CONTAINER_AUTO_WS}/install/mfrproto/code/src \
    ${CONTAINER_AUTO_WS}/install/mfrproto/proto \
    ${CONTAINER_AUTO_WS}/install/mfrproto/temporary \
    ${CONTAINER_AUTO_WS}/install/lib/python2.7 \
    ${CONTAINER_AUTO_WS}/install/lib/pkgconfig  

RUN mv ${CONTAINER_AUTO_WS}/install/share ${CONTAINER_AUTO_WS}/install/shareold && \
    mkdir ${CONTAINER_AUTO_WS}/install/share && \
    # mv ${CONTAINER_AUTO_WS}/install/shareold/msquare_pnc_ros ${CONTAINER_AUTO_WS}/install/share && \
    # rm -rf ${CONTAINER_AUTO_WS}/install/share/msquare_pnc_ros/cmake && \
    # rm -rf ${CONTAINER_AUTO_WS}/install/shareold && \
    # mkdir -p ${CONTAINER_AUTO_WS}/tmp/lib && \
    # mv ${CONTAINER_AUTO_WS}/install/lib/libmsquare_pnc.so ${CONTAINER_AUTO_WS}/tmp/lib
    mv ${CONTAINER_AUTO_WS}/install/shareold/planning_mfr ${CONTAINER_AUTO_WS}/install/share/planning_mfr && \
    mv ${CONTAINER_AUTO_WS}/install/shareold/common_thirdparty ${CONTAINER_AUTO_WS}/install/share/common_thirdparty && \
    rm -rf ${CONTAINER_AUTO_WS}/install/share/planning_mfr/cmake && \
    rm -rf ${CONTAINER_AUTO_WS}/install/share/common_thirdparty/cmake && \
    rm -rf ${CONTAINER_AUTO_WS}/install/shareold


# =================================== runtime stage
FROM ${RUNTIME_BASE} AS runtime-stage
# When ROS not used, just comment out  ROS related steps
# ROS will be replaced by MFR
# -------------------------------------------------
ENV CONTAINER_AUTO_WS /home/ros/catkin_ws

# # Disable ROS msg generation for languages other than C++/Python
ENV ROS_LANG_DISABLE geneus:genlisp:gennodejs
# ROS basic settings
ENV ROS_MASTER_IP 127.0.0.1
ENV ROS_MASTER_PORT 11311
ENV ROS_MASTER_URI http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}
ENV ROS_IP 127.0.0.1
ENV ROS_PORT 11311
ENV CAM_CALIB_DIR /calib
ENV WHICH_CAR MKZ_SIM
ENV HDMAP latest
ENV LOCATION sz
ENV GLOBAL_TRAJECTORY "demo-v2.40"

# ------------------------------------------------

ENV MSQUARE_PNC_PROVIDER_ADDRESS 0.0.0.0:20001
ENV MSQUARE_PNC_LOG_LEVEL INFO

# HIGHWAY/URBAN: 0, SVP: 1, LVP: 4, APA, 5
ENV PLANNING_MODE 0

STOPSIGNAL SIGINT

# Executable files change frequently , placed in the back
# Copy the executable file and built dependency to the current image
COPY --from=build ${CONTAINER_AUTO_WS}/install ${CONTAINER_AUTO_WS}/install
#COPY --from=build ${CONTAINER_AUTO_WS}/tmp/lib/libmsquare_pnc.so ${CONTAINER_AUTO_WS}/install/lib/
COPY entrypoint.sh /entrypoint.sh
COPY start.sh /start.sh
COPY start_ldp.sh /start_ldp.sh
COPY start_sbp.sh /start_sbp.sh
COPY start_all.sh /start_all.sh

ENTRYPOINT ["/entrypoint.sh"]

#===================================
FROM runtime-stage AS provider-stage
CMD ["roslaunch", "msquare_pnc_ros", "debug.launch", "msim_provider:=true"]

#==============================
FROM runtime-stage AS ros-stage
RUN apt-get update \
 && apt-get install -y libyaml-cpp-dev \
 && apt-get install -y net-tools
RUN apt install -y libcurl4-openssl-dev libssl-dev
ENV PACKAGE_PATH ${CONTAINER_AUTO_WS}/install/share/planning_mfr
ENV CMAKE_BINARY_PATH ${CONTAINER_AUTO_WS}/install
ENV APA_PLANNING_RESOURCE_PATH ${PACKAGE_PATH}/resource
ENV LDP_PLANNING_RESOURCE_PATH ${PACKAGE_PATH}/resource
ENV SBP_PLANNING_RESOURCE_PATH ${PACKAGE_PATH}/resource
ENV APA_WORLDMODEL_RESOURCE_PATH ${PACKAGE_PATH}/resource
ENV MFR_RELEASE_PATH ${CONTAINER_AUTO_WS}/install/share/common_thirdparty/mfr-release
CMD ["/start_all.sh"]
