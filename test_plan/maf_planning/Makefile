PROJECT_SOURCE_DIR ?= $(abspath ./)
BUILD_DIR ?= $(PROJECT_SOURCE_DIR)/build
INSTALL_DIR ?= $(BUILD_DIR)/install
CONAN_PKG_DIR ?= $(BUILD_DIR)/conan_package
DEPLOY_DIR ?= $(BUILD_DIR)/deploy
NUM_JOB ?= 8
CONAN_REMOTE ?= conan-momenta

ISIM := ./sim
ISIM_URL := https://artifactory.momenta.works/artifactory/msim-tools/release/latest/sim
ISIM_MAFVERSION := maf4.0
ISIM_PRODUCT := cp
ISIM_PORT := 8060
ISIM_PLANNING_RESOURCE_PATH := /opt/maf_planning/build/resource/
FPP_LOCAL_IMAGE := artifactory.momenta.works/docker-momenta/np_planning/fpp_local:maf4.0-4rc3

# 模块尽量支持所有平台
RUN_IN_BUILD_ENV_BATCH_ARGS ?= --force-includes win manylinux --

# https://devops.momenta.works/Momenta/public/_git/conan-packages-bundle
PACKAGE_COMPATIBILITY_LIST ?= sample_bundle/0.0.2@momenta/stable
BUILD_TYPE ?= RelWithDebInfo
RUN_ON_CPU ?= ON

ifneq ($(MF_SYSTEM_ROOT_DIR), )
    MF_SYSTEM_ROOT := ${MF_SYSTEM_ROOT_DIR}
endif
MF_SYSTEM_ROOT ?= ./mf_system
FPP_SIM_BUILD ?= OFF
ifeq ($(RUN_IN_FPP_CONTAINER),TRUE)
BUILD_TYPE = RelWithDebInfo
MF_SYSTEM_ROOT = ${HOME}/modo_mf_system
$(shell git config --global credential.helper store)
$(shell git config --global --list | grep "safe.directory=\*" >/dev/null 2>&1 || git config --global --add safe.directory "*")
$(shell sudo git config --system --unset credential.helper)
ifneq ($(wildcard ${MF_SYSTEM_ROOT}/.),)
else
$(shell git clone -b dev_maf4.0_v4.0.0_candidate --depth 1 https://devops.momenta.works/Momenta/maf/_git/mf_system ${MF_SYSTEM_ROOT})
endif
endif

ifeq ($(RUN_IN_BUILD_ENV),TRUE)
include ${MF_SYSTEM_ROOT}/package/utils.mk
BUILD_ENVS_JSON := ${MF_SYSTEM_ROOT}/package/build_envs.json
RUN_IN_BUILD_ENV_PY := ${MF_SYSTEM_ROOT}/package/run_in_build_env.py
endif

all:
	@echo nothing special
clean:
	rm -rf $(BUILD_DIR) test_package/build/*

# Check build env and set PLATFORM
#
# product           os          os.platform  os.version   arch       env
# -----------------------------------------------------------------------------------------
# x86_cpu/onnx      Linux       Devcar       -            x86_64     RUN_ON_CPU=ON
# x86_gpu/onnx      Linux       Devcar       -            x86_64     RUN_ON_CPU=OFF
# orin              Linux       Orin         -            aarch64
# mdc610-wls        MDC         610          1.1201.023   aarch64
# mdc610-wls        MDC         610          1.1201.026   aarch64
# mdc610-lhs        MDC         610          1.1000.002   aarch64
# mdc610-lhs        MDC         610          1.1000.008   aarch64
# mdc810            MDC         810          1.9000.003   aarch64
# xavier            Neutrino    -            7.0          armv8
# xavier_safety     Neutrino    -            7.0          armv8      BUILD_SAFETY_FLAG=True

CONAN_OS := $(shell conan profile show default 2>/dev/null | sed -n "s/^os=//p")
CONAN_OS_PLATFORM := $(shell conan profile show default 2>/dev/null | sed -n "s/^os\\.platform=//p")
CONAN_OS_VERSION := $(shell conan profile show default 2>/dev/null | sed -n "s/^os\\.version=//p")
PLANNING_PLATFORM := unknown
ifeq ($(CONAN_OS), Linux)
	ifeq ($(CONAN_OS_PLATFORM), Devcar)
		ifeq ($(RUN_ON_CPU), ON)
PLANNING_PLATFORM := x86_cpu
		else
PLANNING_PLATFORM := x86_gpu
		endif
	else ifeq ($(CONAN_OS_PLATFORM), Orin)
PLANNING_PLATFORM := orin
	endif
else ifeq ($(CONAN_OS), MDC)
	ifeq ($(CONAN_OS_PLATFORM), 610)
		ifeq ($(CONAN_OS_VERSION), 1.1201.028)
PLANNING_PLATFORM := mdc_610_wls
		else ifeq ($(CONAN_OS_VERSION), 1.1201.026)
PLANNING_PLATFORM := mdc_610_wls
		else ifeq ($(CONAN_OS_VERSION), 1.1000.002)
PLANNING_PLATFORM := mdc_610_lhs
		else ifeq ($(CONAN_OS_VERSION), 1.1000.008)
PLANNING_PLATFORM := mdc_610_lhs
		endif
	else ifeq ($(CONAN_OS_PLATFORM), 810)
PLANNING_PLATFORM := mdc_810
	endif
else ifeq ($(CONAN_OS), Neutrino)
	ifeq (${BUILD_SAFETY_FLAG}, True)
PLANNING_PLATFORM := xavier_safety
	else
PLANNING_PLATFORM := xavier
	endif
endif

PACKAGE_NAME := apa_planning
PACKAGE_VERSION_SUFFIX := 

CMAKE_EXTRA_ARGS ?= # 留作接口，勿用
ifeq ($(PLANNING_PLATFORM), xavier)
CMAKE_EXTRA_ARGS += -D WITHOUT_CPP_LOG=ON
endif

CMAKE_ARGS := \
	-DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) \
	-DBUILD_SHARED_LIBS=OFF \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DRUN_ON_CPU=$(RUN_ON_CPU) \
	-DPLANNING_PLATFORM=$(PLANNING_PLATFORM) \
	$(CMAKE_EXTRA_ARGS) 
        #-DBUILD_WITH_OPENCV=ON \

RUN_IN_BUILD_ENV_BATCH_ARGS := --excludes win --

prepare:
	make default_prepare

build/default_prepare_sentinel:
ifeq ($(RUN_IN_FPP_CONTAINER),TRUE)
	make default_prepare
	mkdir -p build
	touch build/default_prepare_sentinel
endif

local_build_prepare: build/default_prepare_sentinel
ifeq (,$(wildcard build/real_conanfile/conanfile.py))
	make conanfile_prepare
endif

ifeq ($(FPP_SIM_BUILD), ON)
CAMKE_ARGS_FPP_SIM := \
	${CMAKE_ARGS} \
	-DCI_BUILD=ON \
	-DFPP_SIM_BUILD=ON
build: local_build_prepare
	mkdir -p build && cd build && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash && source /opt/ros/catkin_ws/install/setup.bash && \
	cmake $(CAMKE_ARGS_FPP_SIM) .. && \
	make -j $(NUM_JOB) && \
	make install"
else
build:
	make default_build
endif

package:
	make default_package
upload: package
	make default_upload

CMAKE_ARGS_CI := \
	-DRUN_IN_FPP=ON \
	-DCI_BUILD=ON \
	-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	-DPNC_ENABLE_TEST=ON \
	-DCODE_COVERAGE=ON \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DMAF_VERSION=$(MAF_VERSION) \
	-DRUN_ON_CPU=$(RUN_ON_CPU) \
	-DPLATFORM=$(PLATFORM) \
	-DPLANNING_PLATFORM=$(PLANNING_PLATFORM) \
	-DPACKAGE_VERSION=$(PACKAGE_VERSION) \
	-DBUILD_FPP_MFR=$(BUILD_FPP_MFR)
_ci_build: local_build_prepare # pass RUN_ON_CPU=ON manually
	if [ -f "./build/tools/calc_one_frame/fpp_sim" ]; then \
		make -C build -j $(NUM_JOB) fpp_sim trace_throw; \
	else \
		mkdir -p build && cd build && \
		cmake $(CMAKE_ARGS_CI) .. && \
		make -j $(NUM_JOB) && \
		make install; \
	fi

ci_build:
	if command -v nvidia-smi ; then \
		$(MAKE) _ci_build RUN_ON_CPU=OFF ; \
	else \
		$(MAKE) _ci_build RUN_ON_CPU=ON ; \
	fi

_local_fpp_build: local_build_prepare
	if [ -f "./build/tools/calc_one_frame/fpp_sim" ]; then \
		make -C build -j $(NUM_JOB) fpp_sim trace_throw; \
	else \
		mkdir -p build && cd build && \
		cmake $(CMAKE_ARGS_CI) -DRELWITHDEBINFO_FORCE_OPTIMIZATION_O0=ON .. && \
		make -j $(NUM_JOB) fpp_sim trace_throw && \
		make install; \
	fi

local_fpp_build:
	if command -v nvidia-smi ; then \
		$(MAKE) _local_fpp_build RUN_ON_CPU=OFF ; \
	else \
		$(MAKE) _local_fpp_build RUN_ON_CPU=ON ; \
	fi

local_fpp_deploy:
	rm -rf ${ISIM_PLANNING_RESOURCE_PATH} && \
	mkdir -p ${ISIM_PLANNING_RESOURCE_PATH} && \
	cp -r ./resource/config/* ${ISIM_PLANNING_RESOURCE_PATH}

deploy: package
	make default_deploy

download_isim:
ifeq (,$(wildcard $(ISIM)))
	@echo "Download isim"
	@wget -O $(ISIM) $(ISIM_URL)
	@chmod +x ${ISIM};
endif

start_fpp: download_isim
	${ISIM} fpp container start -m ${ISIM_MAFVERSION} -p ${ISIM_PRODUCT} --port ${ISIM_PORT} --image ${FPP_LOCAL_IMAGE}

stop_fpp:
	${ISIM} fpp container stop

.PHONY: build package upload deploy ci_build local_fpp_build local_fpp_deploy start_fpp stop_fpp download_isim

RED := '\033[31;7m'
NC := '\033[0m'
test_all:
	@cd build && for t in $(wildcard $(BUILD_DIR)/bin/test_*); do echo $$t && eval $$t >/dev/null 2>&1 && echo 'ok' || echo $(RED)Not Ok$(NC); done


# 临时只在这三个环境下 build
my_build_all:
	python3 $(RUN_IN_BUILD_ENV_PY) --build-env $(BUILD_ENVS_JSON) --includes u16 mdc qnx devcar orin -- make build

BUILD_ENV_ID ?= $(shell hostname)
URL_PREFIX ?= https://artifactory.momenta.works/artifactory/mdk-sample-data/ci-releases/$(PACKAGE_NAME)/$(PACKAGE_VERSION)/$(BUILD_ENV_ID)
FILE_TO_UPLOAD ?=
upload_artifact:
	@test -f $(FILE_TO_UPLOAD) && \
	curl -T $(FILE_TO_UPLOAD) $(URL_PREFIX)/$(FILE_TO_UPLOAD) || \
	(echo '"$(FILE_TO_UPLOAD)" is not a file' && exit 1)

upload_conan_package_targz:
	mkdir -p $(CONAN_PKG_DIR) && cd $(CONAN_PKG_DIR) && \
	conan install $(PACKAGE_ID) --profile $(CONAN_PROFILE) -g=deploy && \
	tar cvzf package.tar.gz -C $(PACKAGE_NAME) `ls $(PACKAGE_NAME)` && \
	curl -T package.tar.gz $(URL_PREFIX)/package.tar.gz

QNX_SERVER ?= qnx
DEPLOY_DIR ?= /root/bin/$(USER)/$(PACKAGE_NAME)
qnx_deploy:
	ssh $(QNX_SERVER) mkdir -p $(DEPLOY_DIR)
	scp build/repo/3/build/bin/test_rapidjson $(QNX_SERVER):$(DEPLOY_DIR)
qnx_test:
	ssh $(QNX_SERVER) "sh -c 'export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH && cd $(DEPLOY_DIR) && ./test_rapidjson'"

install_from_remote:
	conan remove $(PACKAGE_NAME) -f
	conan install $(PACKAGE_ID)
smart_upload:
	make install_from_remote || make package upload

clean_test:
	rm -rf test_package/build
conan_test:
	conan test test_package $(PACKAGE_ID)

install_torch:
	mkdir -p /utopia && \
	cd /utopia && curl -fSL --insecure -o libtorch.zip https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.4.0%2Bcpu.zip && \
	cd /utopia && unzip libtorch.zip
