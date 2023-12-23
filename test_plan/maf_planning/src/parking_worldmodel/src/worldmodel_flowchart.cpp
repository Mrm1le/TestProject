#include "worldmodel_flowchart.h"
#include "planning/common/logging.h"

namespace parking {

void SetPSDFusionTask::on_running() {
  if (!planning_control_cmd_request_receiver_->empty()) {
    auto ret = planning_control_cmd_request_receiver_->fetch_newest_and_clear(
        module_control_cmd_request_);
    if (ret) {
      if (module_control_cmd_request_.module_control_cmd.value ==
          maf_system_manager::ModuleControlCmdEnum::PAUSE) {
        reset();
        return;
      }
    }
  }
  while (!planning_request_receiver_->empty()) {
    maf_system_manager::SysPlanningRequest frame{};
    auto ret = planning_request_receiver_->pop_oldest(frame);
    if (ret) {
      (void)worldmodel_pec::WorldModelPEC::getInstance()->feedInitAPAMode(
          frame);
      if (frame.cmd.value ==
          maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_FUNCTION_MODE) {
        const auto& mode = frame.highway_info.function_mode.value;
        if (mode == maf_system_manager::FunctionModeEnum::APA
           || mode == maf_system_manager::FunctionModeEnum::PNP) {
          reschedule("mode_running");
        } else {
          reschedule("mode_idle");
        }
      }
    }
  }
  if (!loc_recv_->empty()) {
    MLALocalizationPtr frame{};
    auto ret = loc_recv_->fetch_newest_and_clear(frame); // 获取最新帧
    if (ret) {
      (void)worldmodel_pec::WorldModelPEC::getInstance()->feedLocalization(
          *frame);
    }
  }
  while (!perception_slot_recv_->empty()) {
    FusionParkingSlotResultPtr frame{};
    auto ret =
        perception_slot_recv_->fetch_newest_and_clear(frame); // 获取最新帧
    if (ret) {
      (void)worldmodel_pec::WorldModelPEC::getInstance()
          ->feedParkingSlotPerceptionFusion(*frame);
      *parkingslot_output_ =
          worldmodel_pec::WorldModelPEC::getInstance()->getParkingSlotOutput();

      auto tmp_parkingslot_output = std::make_shared<maf_worldmodel::FusionAPA>();
      *tmp_parkingslot_output = *parkingslot_output_;
      tmp_parkingslot_output->header.seq = ++parking_slot_seq_;

      if (callback_) {
        callback_(tmp_parkingslot_output);
      } else {
        MLOG_ERROR(" *************** fusion apa callback not set");
      }
    }
  }
  if (!gndl_recv_->empty()) {
    FusionGroundLineResultPtr frame{};
    auto ret = gndl_recv_->fetch_newest_and_clear(frame); // 获取最新帧
    if (ret) {
      (void)worldmodel_pec::WorldModelPEC::getInstance()->feedFusionGroundline(
          *frame);
    }
  }
  if (!od_recv_->empty()) {
    PerceptionFusionObjectResultPtr frame{};
    auto ret = od_recv_->fetch_newest_and_clear(frame); // 获取最新帧
    if (ret) {
      (void)worldmodel_pec::WorldModelPEC::getInstance()->feedFusionObject(
          *frame);
    }
  }
  if (!wireless_charger_report_recv_->empty()) {
    MSD_LOG(ERROR, "wlc %s: %d", __FUNCTION__, __LINE__);
    std::shared_ptr<maf_endpoint::WirelessChargerReport> frame{};
    auto ret =
        wireless_charger_report_recv_->fetch_newest_and_clear(frame); // 获取最新帧
    if (ret) {
      worldmodel_pec::WorldModelPEC::getInstance()->feedWirelessChargerReport(
          *frame);
    }
  }
}

void SetPSDFusionTask::reset() {
  MSD_LOG(ERROR, "wlc %s: %d", __FUNCTION__, __LINE__);
  (void)worldmodel_pec::WorldModelPEC::getInstance()->reset();
  perception_slot_recv_->clear();
  loc_recv_->clear();
  gndl_recv_->clear();
  od_recv_->clear();
  planning_request_receiver_->clear();
  // planning_control_cmd_request_receiver_->clear();
  wireless_charger_report_recv_->clear();
  parking_slot_seq_ = 0;
  (void)setAPAStatus("idle", 0);
}

bool SetPSDFusionTask::setAPAStatus(const std::string &status,
                                    const int target_id) {
  return worldmodel_pec::WorldModelPEC::getInstance()->setAPAStatus(status,
                                                                    target_id);
}

} // namespace parking
