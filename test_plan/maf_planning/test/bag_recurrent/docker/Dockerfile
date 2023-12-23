from nvidia/cuda:11.1.1-base-ubuntu16.04 as base
shell ["/bin/bash", "-c"]
workdir /
run rm -r /usr/local/cuda /usr/local/cuda-11.1

run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        DEBIAN_FRONTEND=noninteractive apt-get install -y \
            tzdata
run ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
run dpkg-reconfigure -f noninteractive tzdata

# basic
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            curl \
            lsb-core \
            ttf-wqy-microhei \
            libxml2-dev \
            gcc \
            g++
env LANG=C.UTF-8

# cuda
from base as cuda
run --mount=type=bind,source=cuda_11.1.0_455.23.05_linux.run,target=cuda.run \
        ./cuda.run --silent --toolkit
run tar -cf /cuda.tar /usr/local/cuda-11.1/targets/x86_64-linux/lib/lib*.so* /usr/local/cuda /usr/local/cuda-11.1/lib64
from base
run --mount=type=bind,from=cuda,source=/cuda.tar,target=cuda.tar \
        tar -xf cuda.tar
env LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# pyenv and python3
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            make \
            build-essential \
            libssl-dev \
            zlib1g-dev \
            libbz2-dev \
            libreadline-dev \
            libsqlite3-dev \
            wget \
            llvm \
            libncurses5-dev \
            libncursesw5-dev \
            xz-utils \
            tk-dev \
            libffi-dev \
            liblzma-dev \
            python-openssl \
            git
env PYENV_ROOT=/usr/local/.pyenv
run curl https://pyenv.run | bash
env PATH=$PYENV_ROOT/bin:$PATH
env PYTHON_MAJOR_MINOR=3.7
env PYTHON_PATCH=16
run pyenv install -v $PYTHON_MAJOR_MINOR.$PYTHON_PATCH
env PATH=$PYENV_ROOT/versions/$PYTHON_MAJOR_MINOR.$PYTHON_PATCH/bin:$PATH

# slurm
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            openssh-server
run sed -i -r 's/.?UseDNS\syes/UseDNS no/' /etc/ssh/sshd_config; \
        sed -i -r 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config; \
        sed -i -r 's/#UsePAM no/UsePAM no/g' /etc/ssh/sshd_config; \
        sed -i -r 's/.?PermitRootLogin.+/PermitRootLogin yes/' /etc/ssh/sshd_config; \
        mkdir -p /var/run/sshd; chmod 700 /var/run/sshd/; chmod +s /usr/sbin/sshd
run wget http://172.20.196.100:/train-docker/docker-cmd.sh -O /docker-cmd.sh; \
        wget http://172.20.196.100:/train-docker/fixuid -O /usr/local/bin/fixuid; \
        chown root:root /usr/local/bin/fixuid; chmod 4755 /usr/local/bin/fixuid; \
        chown root:root /docker-cmd.sh; chmod +x /docker-cmd.sh; groupadd -g 1000 docker; \
        useradd -u 1000 -g docker -d /home/docker -s /bin/sh docker; mkdir -p /etc/fixuid; \
        USER=docker GROUP=docker printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml
run mkdir -p /syncdata; mkdir -p /share/public/

# ros
run echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros-latest.list
run curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | apt-key add -
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            ros-kinetic-desktop-full
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            liblz4-dev
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        python3 -m pip install \
            rospkg \
            py3rosmsgs \
            pycryptodomex
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        python3 -m pip install --extra-index-url https://rospypi.github.io/simple/ \
            roslz4

# develop dependencies
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        apt-get update; \
        apt-get install -y \
            libopencv-dev \
            libeigen3-dev \
            libyaml-cpp-dev \
            gdb \
            vim \
            tmux \
            htop
run --mount=type=cache,sharing=locked,id=custom-aptlib,target=/var/lib/apt \
        --mount=type=cache,sharing=locked,id=custom-aptcache,target=/var/cache/apt \
        --mount=type=tmpfs,target=/tmp \
        python3 -m pip install \
            pybind11 \
            'bokeh<3.0.0' \
            ipywidgets \
            numpy \
            notebook \
            jupytext \
            scipy \
            easydict \
            requests \
            pymongo
run jupyter serverextension enable jupytext
env pybind11_DIR=$PYENV_ROOT/versions/$PYTHON_MAJOR_MINOR.$PYTHON_PATCH/lib/python$PYTHON_MAJOR_MINOR/site-packages/pybind11/share/cmake/pybind11

arg ARCH
arg CMAKE_NAME=cmake-3.24.2-linux-$ARCH
run wget https://cmake.org/files/v3.24/$CMAKE_NAME.tar.gz; \
        tar -zxf $CMAKE_NAME.tar.gz; mv $CMAKE_NAME /usr/local; \
        rm $CMAKE_NAME.tar.gz
env PATH=/usr/local/$CMAKE_NAME/bin:$PATH
