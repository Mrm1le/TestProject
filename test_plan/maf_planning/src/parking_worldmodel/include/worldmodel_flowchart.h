#pragma once

#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include "mtaskflow/mtaskflow.hpp"
#include "parking_worldmodel.h"
#include "unistd.h"
#include "worldmodel_pec_interface.h"
#include <deque>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

using namespace mtaskflow;
using std::function;
using std::string;

namespace parking {

class SetPSDFusionTask : public FlowTask {
public:
  SetPSDFusionTask(
      FlowReceiver<FusionParkingSlotResultPtr> perception_slot_recv,
      FlowReceiver<MLALocalizationPtr> loc_recv,
      FlowReceiver<FusionGroundLineResultPtr> gndl_recv,
      FlowReceiver<PerceptionFusionObjectResultPtr> od_recv,
      FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
          planning_control_cmd_request_receiver,
      FlowReceiver<std::shared_ptr<maf_endpoint::WirelessChargerReport>>
          wireless_charger_report_recv,
      mtaskflow::FlowReceiver<maf_system_manager::SysPlanningRequest>
          planning_request_receiver,
      std::string params_json_str)
      : perception_slot_recv_(perception_slot_recv), loc_recv_(loc_recv),
        gndl_recv_(gndl_recv), od_recv_(od_recv),
        planning_control_cmd_request_receiver_(
            planning_control_cmd_request_receiver),
        wireless_charger_report_recv_(wireless_charger_report_recv),
        planning_request_receiver_(planning_request_receiver) {
    callback_ = nullptr;
    parkingslot_output_ = std::make_shared<maf_worldmodel::FusionAPA>();
    module_control_cmd_request_.module_control_cmd.value =
        maf_system_manager::ModuleControlCmdEnum::PAUSE;
    (void)worldmodel_pec::WorldModelPEC::getInstance()->init(params_json_str);
  }

  void on_running();
  void setCallback(FusionAPACallback callback) { callback_ = callback; }
  void reset();
  bool setAPAStatus(const std::string &status, const int target_id);

private:
  FlowReceiver<FusionParkingSlotResultPtr> perception_slot_recv_;
  FlowReceiver<MLALocalizationPtr> loc_recv_;
  FlowReceiver<FusionGroundLineResultPtr> gndl_recv_;
  FlowReceiver<PerceptionFusionObjectResultPtr> od_recv_;
  FusionAPACallback callback_;
  mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
      planning_control_cmd_request_receiver_;
  FlowReceiver<std::shared_ptr<maf_endpoint::WirelessChargerReport>>
      wireless_charger_report_recv_;
  mtaskflow::FlowReceiver<maf_system_manager::SysPlanningRequest>
      planning_request_receiver_;

  std::shared_ptr<maf_worldmodel::FusionAPA> parkingslot_output_;

  maf_system_manager::ModuleControlCmdRequest module_control_cmd_request_{};

  uint64_t parking_slot_seq_{0};
};

} // namespace parking
