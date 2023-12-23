ARG MOMENTA_COMMON_BASE=artifactory.momenta.works/docker-msd-dev/momenta-common-base:v2.0.1

# =================================
FROM ${MOMENTA_COMMON_BASE}

# Acado (https://acado.github.io/install_linux.html)
RUN apt-get update \
 && apt-get install -y gnuplot doxygen graphviz \
 && apt-get install -y libglm-dev g++ \
 && apt-get install -y google-mock libgtest-dev \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/acado/acado.git -b stable --depth 1 /ACADOtoolkit \
 && cd /ACADOtoolkit && mkdir build && cd build \
 && cmake -D CMAKE_BUILD_TYPE=Release .. && make -j4 && make install \
 && rm -rf /ACADOtoolkit
RUN ldconfig

# build & install gtest (cmake -DBUILD_SHARED_LIBS=ON . for shlibs) 
RUN cd /usr/src/gtest ; cmake . ; make 
RUN mv /usr/src/gtest/*.a /usr/lib/

# build & install gmock (cmake -DBUILD_SHARED_LIBS=ON . for shlibs) 
RUN cd /usr/src/gmock ; cmake . ; make 
RUN mv /usr/src/gmock/*.a /usr/lib/
