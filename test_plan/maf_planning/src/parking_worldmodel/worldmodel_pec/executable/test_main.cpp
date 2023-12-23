#include "../interface/worldmodel_pec_interface.h"

int main() {
  worldmodel_pec::WorldModelPEC *worldmodel_pec_handle =
      worldmodel_pec::WorldModelPEC::getInstance();

  std::string test_resource_path =
      "/home/dozen/Workspace/fusion/worldmodel_pec/resources";

  std::string calib_folder = test_resource_path + "/test_calib";
  std::string map_folder = test_resource_path + "/test_map";

  worldmodel_pec_handle->init();

  if (!worldmodel_pec_handle->isInited()) {
    printf("init failed \n");
    return -1;
  }

  printf("---------------------test change random search list\n");
  worldmodel_pec_handle->setSystemManagerRequestChangeRandomSearchList(
      map_folder);

  printf("---------------------test set blacklist\n");

  worldmodel_pec::SystemManagerRequestBlackList
      system_manager_request_black_list;
  system_manager_request_black_list.blacklist_id_.push_back(1);

  worldmodel_pec_handle->setSystemManagerRequestBlackList(
      system_manager_request_black_list);

  return 0;
}
