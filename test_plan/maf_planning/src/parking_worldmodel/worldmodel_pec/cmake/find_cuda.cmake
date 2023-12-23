# For QNX Cross Compile
if(PKM_SYSTEM_ARCH MATCHES "QNX-aarch64le")
    message("------------set cuda path for qnx cross compile-------------")
    # NOTE: CUDA_TOOLKIT_ROOT may differ on your machine
    set(CUDA_TOOLKIT_ROOT /home/zhangfan/tools/cross_compiler/cuda-10.1)
    # NOTE: CUDA_TOOLKIT_ROOT may differ on your machine

    set(CUDA_TOOLKIT_ROOT_DIR ${CUDA_TOOLKIT_ROOT})
    set(CUDA_TOOLKIT_TARGET_NAME "aarch64-qnx")
    set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
    set(CMAKE_AR $ENV{QNX_HOST}/usr/bin/ntoaarch64-ar)
    set(CMAKE_C_COMPILER $ENV{QNX_HOST}/usr/bin/ntoaarch64-gcc )
    set(CMAKE_CXX_COMPILER $ENV{QNX_HOST}/usr/bin/ntoaarch64-g++ )
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_SYSTEM_NAME QNX)
    set(CMAKE_CUDA_HOST_COMPILER $ENV{QNX_HOST}/usr/bin/ntoaarch64-gcc)
    set(CUDA_CUDART_LIBRARY ${CUDA_TOOLKIT_ROOT}/targets/aarch64-qnx/lib/libcudart.so )
endif()


