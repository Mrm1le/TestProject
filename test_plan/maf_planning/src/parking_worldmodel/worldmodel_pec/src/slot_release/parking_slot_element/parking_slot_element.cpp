#include "parking_slot_element.h"
#include "mjson/mjson.hpp"
#include "utils/math/polygon.hpp"
#include <mlog_core/mlog.h>

namespace worldmodel_pec {

ParkingSlotElement::ParkingSlotElement() {
  charge_property_ = 0;
  track_id_ = -1;
  empty_votes_ = 0;
  last_updated_timestamp_sec_ = 0;
  last_updated_timestamp_us_ = 0;
  source_from_vision_ = false;
  source_from_uss_ = false;
  corners_.clear();
}

ParkingSlotElement::~ParkingSlotElement() {}

bool ParkingSlotElement::isMatchWith(const ParkingSlotElement &other) const {
  //边界条件检查：角点非空（防呆）
  if (corners_.empty() || other.corners_.empty()) {
    MLOG_ERROR("[ParkingSlotElement::isMatchWith] trying to call match in an "
               "invalid parking slot(? ? ? ?), this should not happen ");
    return false;
  }

  // 1. 基于距离做过滤（粗筛）
  if ((center_ - other.center_).norm() > 5.0 ||
      std::abs(corners_[0].position.z() - other.corners_[0].position.z()) >
          1.0) {
    return false;
  }

  // 3. 基于面积匹配 （精筛2） 如果重叠面积占比大于一定阈值，则判断匹配
  std::vector<Eigen::Vector2d> his_corners, obs_corners;
  for (auto &corner : other.corners_) {
    his_corners.push_back(corner.position.topRows(2));
  }
  for (auto &corner : corners_) {
    obs_corners.push_back(corner.position.topRows(2));
  }
  double intersect_area =
      putils::Polygon::getIntersectPolygonArea(his_corners, obs_corners);
  double history_area = putils::Polygon::getPolygonArea(his_corners);
  double observ_area = putils::Polygon::getPolygonArea(obs_corners);

  double iohis = intersect_area / (history_area + 0.0001);
  double ioobs = intersect_area / (observ_area + 0.0001);
  if (iohis > 0.5 || ioobs > 0.5) {
    return true;
  }
  return false;
}

bool ParkingSlotElement::pointInParkingSlot(
    const Eigen::Vector3d &point) const {
  //边界条件检查：角点非空（防呆）
  if (corners_.empty()) {
    MLOG_ERROR("[ParkingSlotElement::match] trying to call match in an invalid "
               "parking slot(? ? ? ?), this should not happen ");
    return false;
  }

  //高程差超过1.0m的，忽略
  if (std::abs(point.z() - corners_[0].position.z()) > 1.0) {
    return false;
  }

  //距离超过10.0m的，忽略
  if ((point - center_).norm() > 6.0) {
    return false;
  }

  //基于点是否在多边形框内，判断点是否在车位框内（调用putils::Polygon）
  std::vector<Eigen::Vector2d> corners_2d;
  for (auto &corner : corners_) {
    corners_2d.push_back(corner.position.topRows(2));
  }
  return putils::Polygon::pointInPolygon(point.topRows(2), corners_2d);
}

VirtualParkingSlotPoint ParkingSlotElement::extractVritualCorner() const {
  // fill virtual corner
  VirtualParkingSlotPoint virtual_corner;
  std::string info_str = reserved_info_;
  std::string error = "";
  auto slot_type_mjson = mjson::Json::parse(info_str, error);
  if (error != "") {
    // MSD_LOG( INFO,"[fill_parking_lot_info] parse reserved_info error %s\n",
    // error.c_str()); ret[i] = parking_lot_info;
    return virtual_corner;
  }
  auto slot_type_reader = mjson::Reader(info_str);

  if (false == slot_type_mjson.has_key("slot_type")) {
    // MSD_LOG(INFO, "[fill_parking_lot_info] reserved info(%s) has no slot_type
    // key\n", reserved_info.c_str()); ret[i] = parking_lot_info;
    return virtual_corner;
  }
  int slot_type = slot_type_reader.get<int>("slot_type");

  if (false == slot_type_mjson.has_key("virtual_point_index")) {
    // MSD_LOG(INFO, "[fill_parking_lot_info] reserved info(%s) has no
    // virtual_point_index key\n", reserved_info.c_str()); ret[i] =
    // parking_lot_info;
    return virtual_corner;
  }
  int virtual_point_index = slot_type_reader.get<int>("virtual_point_index");

  double vx, vy, vz;
  if (false == slot_type_mjson.has_key("virual_point_x")) {
    // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
    // virual_point_x key\n", reserved_info.c_str()); ret[i] = parking_lot_info;
    return virtual_corner;
  }
  vx = slot_type_reader.get<double>("virual_point_x");

  if (false == slot_type_mjson.has_key("virual_point_y")) {
    // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
    // virual_point_y key\n", reserved_info.c_str()); ret[i] = parking_lot_info;
    return virtual_corner;
  }
  vy = slot_type_reader.get<double>("virual_point_y");

  if (false == slot_type_mjson.has_key("virual_point_z")) {
    // MSD_LOG(INFO,"[fill_parking_lot_info] reserved info(%s) has no
    // virual_point_z key\n", reserved_info.c_str()); ret[i] = parking_lot_info;
    return virtual_corner;
  }
  vz = slot_type_reader.get<double>("virual_point_z");
  virtual_corner.slot_type = slot_type;
  virtual_corner.index = virtual_point_index;
  virtual_corner.position.x() = vx;
  virtual_corner.position.y() = vy;
  virtual_corner.position.z() = vz;

  return virtual_corner;
  // ret[i] = parking_lot_info;
}

void ParkingSlotElement::applyVritualCorner() {
  const VirtualParkingSlotPoint &virtual_corner = extractVritualCorner();
  if (virtual_corner.slot_type == 4) {
    has_virtual_point_ = true;
    virtual_point_index_ = virtual_corner.index;
    init_point_ = corners_.at(virtual_corner.index).position;
    center_ -= corners_.at(virtual_corner.index).position;
    center_ += virtual_corner.position;
    corners_.at(virtual_corner.index).position = virtual_corner.position;
  }
}

} /* namespace worldmodel_pec */
