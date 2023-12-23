# 最新版本 https://confluence.momenta.works/pages/viewpage.action?pageId=135747115

PROJECT_SOURCE_DIR ?= $(abspath ./)
PROJECT_BUILD_DIR ?= $(PROJECT_SOURCE_DIR)/build
JQ ?= jq

# mph-dev 文档：https://confluence.momenta.works/pages/viewpage.action?pageId=135735242
MPH_MODULE_LIST_JSON ?= $(PROJECT_SOURCE_DIR)/mph_module_list.json
# default MPH_PROJECT_ROOT in mph-dev 镜像
MPH_PROJECT_ROOT ?= /root/workspace/mpilot_product_uniform_platform
# use <mph_name> to extract info from mph_module_list.json
define __work_space__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)").work_space' $(MPH_MODULE_LIST_JSON))
endef
define __module_dir_name__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)") | if has("module_dir_name") then .module_dir_name else .mph_name end' $(MPH_MODULE_LIST_JSON))
endef
define __package_name__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)").package_name' $(MPH_MODULE_LIST_JSON))
endef
define __module_name__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)").module_name' $(MPH_MODULE_LIST_JSON))
endef
define __version__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)").version' $(MPH_MODULE_LIST_JSON))
endef
define __release_note__
	$(shell $(JQ) -r '.[]|select(.mph_name=="$(strip $1)").release_note' $(MPH_MODULE_LIST_JSON))
endef
define __module_suffix_dir__
	$(addprefix $(call __work_space__,$1)/,$(call __module_dir_name__,$1))
endef
# source/build/install dir in local/docker
define __local_source_dir__
	$(PROJECT_SOURCE_DIR)
endef
define __docker_source_dir__
	$(addprefix $(MPH_PROJECT_ROOT)/modules/,$(call __module_suffix_dir__,$1))
endef
define __local_build_dir__
	$(PROJECT_BUILD_DIR)/$(strip $1)/build
endef
define __docker_build_dir__
	$(call __docker_source_dir__,$1)/build
endef
define __local_install_dir__
	$(PROJECT_BUILD_DIR)/$(strip $1)/install
endef
define __docker_install_dir__
	$(addprefix $(MPH_PROJECT_ROOT)/output/,$(call __module_suffix_dir__,$1))
endef
# helper functions
define __local_mkdir_dirs__
	$(strip $(call __local_build_dir__,$1) $(call __local_install_dir__,$1))
endef
define __volume_source__
	$(addprefix -v , $(addprefix $(call __local_source_dir__,$1):,$(call __docker_source_dir__ ,$1)))
endef
define __volume_build__
	$(addprefix -v , $(addprefix $(call __local_build_dir__,$1):,$(call __docker_build_dir__ ,$1)))
endef
define __volume_install__
	$(addprefix -v , $(addprefix $(call __local_install_dir__,$1):,$(call __docker_install_dir__ ,$1)))
endef
define __volumes__
	$(call __volume_source__, $1) $(call __volume_build__, $1) $(call __volume_install__, $1)
endef
