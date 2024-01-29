#pragma once

#include "../slot_release/slot_release_operator/slot_release_manager.h"
#include "interface/worldmodel_pec_interface.h"
#include <mutex>

namespace worldmodel_pec {

// WorldModelPEC模块的算法接口的实际实现和内部的顶层调度
// WorldModelPECImpl内，目前主要包含2部分算法
//	1. deleted: 静态车相关算法，通过调度PECStaticVehicleOperator
// static_vehicle_operator_ 实现
//	2. 车位信息整合&车位推荐相关算法，通过调度PECParkingSlotOperator
// parking_slot_operator_ 实现
class WorldModelPECImpl : public WorldModelPEC {

private:
  // 是否初始化的标记
  bool inited_;

  // 车位信息整合（感知+地图）和车位推荐逻辑相关算法的入口
  SlotReleaseManager parking_slot_operator_;

  // 互斥锁
  std::mutex process_mutex_;

  maf_perception_interface::FusionGroundLineResult groundline_fusion_latest_;
  maf_perception_interface::PerceptionFusionObjectResult object_fusion_latest_;

public:
  virtual bool isInited() override;

  virtual bool init(std::string params_json_str) override;

  virtual bool reset() override;

  virtual bool
  feedLocalization(const maf_mla_localization::MLALocalization &loc) override;
  virtual bool feedWirelessChargerReport(
      const maf_endpoint::WirelessChargerReport &wireless_charger_report)
      override;

  virtual bool feedParkingSlotPerceptionFusion(
      const maf_perception_interface::FusionParkingSlotResult &psd_fusion)
      override;

  virtual bool feedFusionGroundline(
      const maf_perception_interface::FusionGroundLineResult &groundline_fusion)
      override;

  virtual bool feedInitAPAMode(
      const maf_system_manager::SysPlanningRequest &planning_request) override;

  virtual bool
  feedFusionObject(const maf_perception_interface::PerceptionFusionObjectResult
                       &object_fusion) override;

  virtual maf_worldmodel::FusionAPA getParkingSlotOutput() override;

  virtual bool setSystemManagerRequestBlackList(
      const SystemManagerRequestBlackList &request_data) override;

  virtual bool setSystemManagerRequestChangeRandomSearchList(
      const std::string &map_folder) override;

  virtual bool setAPAStatus(const std::string &status,
                            const int target_id) override;

private:
  // 反初始化：回到未初始化状态
  bool deInit();

public:
  WorldModelPECImpl();
  ~WorldModelPECImpl();
};

} // namespace worldmodel_pec
