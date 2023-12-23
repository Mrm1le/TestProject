# syntax=docker/dockerfile:experimental
ARG IMAGE_FDI=artifactory.momenta.works/docker-momenta/fdi/dfdi_ros_dev_core_with_cuda11.1:v1.0.13
ARG IMAGE_CALIB=artifactory.momenta.works/docker-msd-sim/calib-data:20220817-0e87dd6

FROM ${IMAGE_FDI} as maf_common_build

RUN git clone -b dev_maf3.2.0 --depth 1 https://devops.momenta.works/Momenta/maf/_git/maf_common \
        /opt/maf_common && \
    cd /opt/maf_common && \
    git submodule update --init --recursive && \
    git clone -b dev_maf3.2_v3.2.1_candidate --depth 1 \
        https://devops.momenta.works/Momenta/maf/_git/mf_system && \
    make catkin_make

RUN find /root/catkin_ws -name *mfrmsg* | xargs -n1 -I{} rm -rf {}

FROM ${IMAGE_CALIB} AS calib
FROM ${IMAGE_FDI}

# for compile opencv in dfdi image
RUN rm -f /usr/lib/x86_64-linux-gnu/libjpeg.* && \
    rm /usr/lib/x86_64-linux-gnu/libtiff.* && \
    rm /usr/lib/x86_64-linux-gnu/libwebp.* && \
    rm /usr/lib/x86_64-linux-gnu/libjasper.*

COPY --from=maf_common_build /root/catkin_ws /opt/ros/catkin_ws

COPY --from=calib /calib /opt/calib
RUN cp -r /opt/calib/MKZ-SIM /opt/calib/MKZ_SIM

RUN mkdir -p /utopia && cd /utopia && curl -fSL --insecure -o libtorch.zip \
    https://download.pytorch.org/libtorch/lts/1.8/cpu/libtorch-cxx11-abi-shared-with-deps-1.8.2%2Bcpu.zip && \
    cd /utopia && unzip libtorch.zip

RUN apt install -y python-dev

RUN chmod +xr /root && \
    rm /usr/bin/cmake && \
    ln -s /root/.iso_compiler/v2/cmake-3.21.2-linux-x86_64/bin/cmake /usr/bin/cmake && \
    sed -i '1iDISABLE_AUTO_UPDATE=true' ~/.zshrc ~/.bashrc

RUN rm -rf /usr/lib/ccache

ENV PATH=/root/.iso_compiler/v2/gcc/x86_64-5.4/bin/:$PATH
ENV LD_LIBRARY_PATH=/root/.iso_compiler/v2/cuda-x86_64-11.1/usr/local/cuda-11.1/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
