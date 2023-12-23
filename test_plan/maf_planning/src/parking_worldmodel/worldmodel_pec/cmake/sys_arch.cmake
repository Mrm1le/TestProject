
set(PKM_SYSTEM_ARCH ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR})
	execute_process( COMMAND uname -r COMMAND tr -d '\n' OUTPUT_VARIABLE KERNEL_NAME )
	if(KERNEL_NAME MATCHES "4.9.0-yocto-standard")
	set(PKM_SYSTEM_ARCH "Linux-aarch64-RcarH3")
endif()

if($ENV{CROSS_COMPILE} MATCHES "aarch64-poky-linux-")
	set(CROSS_COMPILE "poky_RcarH3")
elseif($ENV{CROSS_COMPILE} MATCHES "QNX-aarch64le")
    set(CROSS_COMPILE "QNX-aarch64le")	
endif()

if(CROSS_COMPILE MATCHES "poky_RcarH3")
	set(PKM_SYSTEM_ARCH "Linux-aarch64-RcarH3")
	message(STATUS "------poky cross compile detected " "--------")
elseif(CROSS_COMPILE MATCHES "QNX-aarch64le")

    set(PKM_SYSTEM_ARCH "QNX-aarch64le")
    set(CMAKE_SYSTEM_NAME QNX)
    set(arch gcc_ntoaarch64le)
    set(CMAKE_C_COMPILER qcc)
    set(CMAKE_C_COMPILER_TARGET ${arch})
    set(CMAKE_CXX_COMPILER qcc)
    set(CMAKE_CXX_COMPILER_TARGET ${arch})
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11 -O3 -Wall -g " )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} $ENV{CXXFLAGS} -std=gnu++11 -O3 -Wall -g " )
	message(STATUS "------cross compile for QNX-aarch64le detected ${arch}" "--------")    
	
endif()

message(STATUS "------SYSTEM ARCH is " ${PKM_SYSTEM_ARCH} " --------")





if(PKM_SYSTEM_ARCH MATCHES "Linux-x86_64")
    message(STATUS "--- platform is Linux-x86_64 ---")
    add_definitions(-DPLATFORM_IS_LINUX_X86_64)
elseif(PKM_SYSTEM_ARCH MATCHES "Linux-aarch64-RcarH3")
    message(STATUS "--- platform is Linux-aarch64-RcarH3 ---")
    add_definitions(-DPLATFORM_IS_LINUX_AARCH64_RCARH3)
elseif(PKM_SYSTEM_ARCH MATCHES "Linux-aarch64")
    message(STATUS "--- platform is Linux-aarch64 ---")
    add_definitions(-DPLATFORM_IS_LINUX_AARCH64)
elseif(PKM_SYSTEM_ARCH MATCHES "QNX-aarch64le")
    message(STATUS "--- platform is QNX-aarch64le ---")
    add_definitions(-DPLATFORM_IS_QNX_AARCH64LE)
endif()