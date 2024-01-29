#include "worldmodel_pec_impl.h"
#include "planning/common/logging.h"
#include "utils/math/polygon.hpp"
#include <mlog_core/mlog.h>

namespace worldmodel_pec {

WorldModelPEC *WorldModelPEC::getInstance() {
  static WorldModelPECImpl instance;
  return &instance;
}

WorldModelPECImpl::WorldModelPECImpl() { inited_ = false; }

WorldModelPECImpl::~WorldModelPECImpl() {}

bool WorldModelPECImpl::isInited() { return inited_; }

bool WorldModelPECImpl::init(std::string params_json_str) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);

  // 初始化parking_slot_operator_
  nlohmann::json json = nlohmann::json::parse(params_json_str);
  SlotReleaseParams params = json;
  if (!parking_slot_operator_.init(params)) {
    MLOG_ERROR(
        "[WorldModelPECImpl::init] failed to init parking_slot_operator ");
    return false;
  }
  inited_ = true;
  return true;
}

bool WorldModelPECImpl::reset() {
  MSD_LOG(ERROR, "%s: %d", __FUNCTION__, __LINE__);
  std::lock_guard<std::mutex> lg(process_mutex_);
  MSD_LOG(ERROR, "%s: %d", __FUNCTION__, __LINE__);
  return parking_slot_operator_.reset();
}

bool WorldModelPECImpl::feedLocalization(
    const maf_mla_localization::MLALocalization &loc) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return false;
  }
  // 调用parking_slot_operator_接口，输入定位数据
  if (!parking_slot_operator_.feedLocalization(loc)) {
    MLOG_ERROR("[WorldModelPECImpl::feedLocalization] unexpected process error "
               "in parking_slot_operator_.feedLocalization");
    return false;
  }

  return true;
}

bool WorldModelPECImpl::feedWirelessChargerReport(
    const maf_endpoint::WirelessChargerReport &wireless_charger_report) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return false;
  }
  MSD_LOG(ERROR, "wlc %s: %d", __FUNCTION__, __LINE__);
  if (!parking_slot_operator_.feedWirelessChargerReport(
          wireless_charger_report)) {
    MLOG_ERROR("[WorldModelPECImpl::feedWirelessChargerReport] unexpected "
               "process error "
               "in parking_slot_operator_.feedWirelessChargerReport");
    return false;
  }
  return true;
}

bool WorldModelPECImpl::feedParkingSlotPerceptionFusion(
    const maf_perception_interface::FusionParkingSlotResult &psd_fusion) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return false;
  }
  // 调用parking_slot_operator_接口，输入PSD融合数据
  if (!parking_slot_operator_.feedPerceptionPSDFusion(psd_fusion)) {
    MLOG_ERROR("[WorldModelPECImpl::feedLocalization] unexpected process error "
               "in parking_slot_operator_.feedPSDFusion");
    return false;
  }
  return true;
}

bool WorldModelPECImpl::feedFusionGroundline(
    const maf_perception_interface::FusionGroundLineResult &groundline_fusion) {
  // 检查初始化状态
  //  std::lock_guard<std::mutex> lg(process_mutex_);
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING\n");
    return false;
  }

  // if(parking_slot_operator_.feedObjectFusion(groundline_fusion)) {
  //   MLOG_ERROR("[WorldModelPECImpl::feedLocalization] unexpected process
  //   error "
  //              "in parking_slot_operator_.feedPSDFusion");
  //   return false;
  // }
  parking_slot_operator_.feedObjectFusion(groundline_fusion);
  // groundline_fusion_latest_ = groundline_fusion;

  return true;
}

bool WorldModelPECImpl::feedFusionObject(
    const maf_perception_interface::PerceptionFusionObjectResult
        &object_fusion) {
  // std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING\n");
    return false;
  }

  // if(parking_slot_operator_.feedObjectDetection(object_fusion)) {
  //   MLOG_ERROR("[WorldModelPECImpl::feedLocalization] unexpected process
  //   error "
  //              "in parking_slot_operator_.feedPSDFusion");
  //   return false;
  // }
  parking_slot_operator_.feedObjectDetection(object_fusion);
  // object_fusion_latest_ = object_fusion;

  return true;
}

bool WorldModelPECImpl::feedInitAPAMode(
    const maf_system_manager::SysPlanningRequest &planning_request) {
  // std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING\n");
    return false;
  }

  // if(parking_slot_operator_.feedObjectDetection(object_fusion)) {
  //   MLOG_ERROR("[WorldModelPECImpl::feedLocalization] unexpected process
  //   error "
  //              "in parking_slot_operator_.feedPSDFusion");
  //   return false;
  // }
  parking_slot_operator_.feedAPAMode(planning_request);
  // object_fusion_latest_ = object_fusion;

  return true;
}

maf_worldmodel::FusionAPA WorldModelPECImpl::getParkingSlotOutput() {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return maf_worldmodel::FusionAPA{};
  }
  // parking_slot_operator_.ProcessSlotReleaseAndGetResult(
  //   object_fusion_latest_, groundline_fusion_latest_);
  // (void)parking_slot_operator_.calculateSuggestedParkingSlot();
  // 调用parking_slot_operator_接口，返回输出结果
  return parking_slot_operator_.ProcessSlotReleaseAndGetResult();
}

bool WorldModelPECImpl::setSystemManagerRequestBlackList(
    const SystemManagerRequestBlackList &request_data) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return false;
  }
  // 调用parking_slot_operator_接口，输入车位黑名单信息
  if (!parking_slot_operator_.setBlackList(request_data.blacklist_id_)) {
    MLOG_ERROR("[WorldModelPECImpl::setSystemManagerRequestBlackList] "
               "unexpected error in parking_slot_operator_.setBlackList ");
    return false;
  }

  MLOG_INFO("[WorldModelPECImpl::setSystemManagerRequestBlackList] set "
            "blacklist successful (count = %d)",
            request_data.blacklist_id_.size());
  return true;
}

bool WorldModelPECImpl::setSystemManagerRequestChangeRandomSearchList(
    const std::string &map_folder) {
  // 互斥锁，防止此函数和其他函数被同时执行
  std::lock_guard<std::mutex> lg(process_mutex_);
  // 检查初始化状态
  if (!inited_) {
    MLOG_ERROR("UNINITED, DO NOT CALL ANYTHING");
    return false;
  }

  // 调用parking_slot_operator_接口，重新通过读文件的形式加载地图白名单信息
  //  parking_slot_operator_需要重新通过读文件的形式加载地图车位白名单
  // 车位黑名单和地图车位不在此处处理
  if (!parking_slot_operator_.loadWhiteListFromMap(map_folder)) {
    MLOG_ERROR("[WorldModelPECImpl::setSystemManagerRequestChangeMap] failed "
               "in parking_slot_operator_.loadWhiteListFromMap ");

    // 致命错误：清除初始化状态
    MLOG_ERROR("[WorldModelPEC] A FATAL ERROR OCCURED");
    (void)deInit();
    return false;
  }

  MLOG_INFO("[WorldModelPECImpl::setSystemManagerRequestChangeMap] change map "
            "successful : %s",
            map_folder.c_str());

  return true;
}

bool WorldModelPECImpl::setAPAStatus(const std::string &status,
                                     const int target_id) {
  return parking_slot_operator_.setAPAStatus(status, target_id);
}

bool WorldModelPECImpl::deInit() {
  inited_ = false;

  // static_vehicle_operator_和parking_slot_operator_不需要特殊处理
  // 他们在再次初始化的时候，会清除所有的变量

  return true;
}

} // namespace worldmodel_pec
