#ifndef MSQUARE_DECISION_PLANNING_COMMON_TRACKED_OBJECT_H_
#define MSQUARE_DECISION_PLANNING_COMMON_TRACKED_OBJECT_H_

#include <float.h>
#include <vector>

namespace msquare {

struct Prediction {
  double prob;
  double interval;
  int num_of_points;
  // double const_vel_prob;
  // double const_acc_prob;
  // double still_prob;
  // double coord_turn_prob;
};

struct PredictionTrajectoryEx {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> speed;

  std::vector<double> std_dev_x;
  std::vector<double> std_dev_y;
  // std::vector<double> std_dev_yaw;
  // std::vector<double> std_dev_speed;

  // std::vector<double> relative_ego_x;
  // std::vector<double> relative_ego_y;
  std::vector<double> relative_ego_yaw;
  // std::vector<double> relative_ego_speed;

  // std::vector<double> relative_ego_std_dev_x;
  // std::vector<double> relative_ego_std_dev_y;
  // std::vector<double> relative_ego_std_dev_yaw;
  // std::vector<double> relative_ego_std_dev_speed;

  int intersection = 0;
};

struct TrackedObject {
  double timestamp = 0; // 时间戳

  int track_id = 0; // 目标id
  int type = 0;     // 目标类型, 定义为FusionObjectType

  double length = 0; // 长
  double width = 0;  // 宽
  double height = 0; // 高

  double center_x = 0; // 中心点的相对x距离(相对坐标原点在自车头, x为车头方向)
  double center_y =
      0; // 中心点的相对y距离(相对坐标原点在自车头, y为垂直车头方向向左)
  double s = 0; // 中心点的s(TrackletMaintainer中维护的相对坐标系下的frenet)
  double l = 0; // 中心点的l(TrackletMaintainer中维护的相对坐标系下的frenet)
  double theta = 0;     // 他车heading的相对朝向
  double speed_yaw = 0; // 他车速度的相对朝向

  double a = 0;         // 纵向加速度
  double v = 0;         // 绝对速度
  double v_lead = 0;    // 纵向绝对速度
  double v_rel = 0;     // 纵向相对速度
  double vy_rel = 0;    // 横向相对速度(l方向)
  double a_lead = 0;    // 和a一致
  double y_rel_ori = 0; // 横向相对位置

  double l0 =
      0; // 自车车头原点的l(TrackletMaintainer中维护的相对坐标系下的frenet)
  double c0 =
      0; // 自车车头原点的s(TrackletMaintainer中维护的相对坐标系下的frenet)
  double d_rel = 0; // 纵向相对距离(他车在车头前方,
  // d_rel是他车车尾到自车车头的距离; 他车在车头后方,
  // d_rel是他车车头到自车车头的相对距离; 他车和车头有重叠,
  // d_rel是零)
  double y_rel = 0;  // d_path基础上加了y_rel_ori - l
  double d_path = 0; // 到带避让的参考线的最近距离, 重叠则为零
  double d_path_pos =
      0; // 在自车前方的点,到带避让的参考线的最近距离, 重叠则为零
  double d_path_self = 0; // 到偏移到自车的参考线的最近距离,重叠则为零
  double d_path_self_pos =
      0; // 在自车前方的点, 到偏移到自车的参考线的最近距离,重叠则为零
  double d_center_cpath = DBL_MAX; // 等同于l
  double d_max_cpath = DBL_MAX;    // 离参考线的最远距离
  double d_min_cpath = DBL_MAX;    // 离参考线的最近距离
  double v_lat = 0;                // 相对参考线的横向速度
  double v_lat_self = 0;           // 等同于v_lat
  double s_center = 0;             // 中心点的s
  double s_max = 0;                // d_max_cpath's s
  double s_min = 0;                // d_min_cpath's s
  double vs_rel = 0;               // s方向的相对速度

  // 避让相关
  double close_time = 0;
  double last_ttc = 0;
  bool is_avd_car = false;
  bool is_ncar = false;
  bool ncar_count_in = false;
  double ncar_count = 0;
  double lat_coeff = 1.0;
  bool oncoming = false;   // 是否为对向车
  bool stationary = false; // 是否为静止

  bool is_lead = false;               // 是否为跟车目标
  bool is_temp_lead = false;          // 是否为变道原车道跟车目标
  double leadone_confidence_cnt = 0;  // leadone计数
  double leadtwo_confidence_cnt = 0;  // leadtwo计数
  double tleadone_confidence_cnt = 0; // tleadone计数
  double tleadtwo_confidence_cnt = 0; // tleadtwo计数

  double last_recv_time = 0; // 上次更新的时间

  double v_ego = 0;        // 自车速度
  double y_center_rel = 0; // 横向中心距离

  // 预测信息
  Prediction prediction;
  PredictionTrajectoryEx trajectory;
};

struct LeadCars {
  LeadCars() {
    lead_one = nullptr;
    lead_two = nullptr;
    temp_lead_one = nullptr;
    temp_lead_two = nullptr;
  }

  LeadCars(const LeadCars &source) {
    if (source.lead_one != nullptr) {
      lead_one = new TrackedObject(*source.lead_one);
    }

    if (source.lead_two != nullptr) {
      lead_two = new TrackedObject(*source.lead_two);
    }

    if (source.temp_lead_one != nullptr) {
      temp_lead_one = new TrackedObject(*source.temp_lead_one);
    }

    if (source.temp_lead_two != nullptr) {
      temp_lead_two = new TrackedObject(*source.temp_lead_two);
    }
  }

  LeadCars &operator=(const LeadCars &source) {
    if (this == &source) {
      return *this;
    }
    clear();

    if (source.lead_one != nullptr) {
      lead_one = new TrackedObject(*source.lead_one);
    }

    if (source.lead_two != nullptr) {
      lead_two = new TrackedObject(*source.lead_two);
    }

    if (source.temp_lead_one != nullptr) {
      temp_lead_one = new TrackedObject(*source.temp_lead_one);
    }

    if (source.temp_lead_two != nullptr) {
      temp_lead_two = new TrackedObject(*source.temp_lead_two);
    }

    return *this;
  }

  void clear() {
    if (lead_one != nullptr) {
      delete lead_one;
      lead_one = nullptr;
    }

    if (lead_two != nullptr) {
      delete lead_two;
      lead_two = nullptr;
    }

    if (temp_lead_one != nullptr) {
      delete temp_lead_one;
      temp_lead_one = nullptr;
    }

    if (temp_lead_two != nullptr) {
      delete temp_lead_two;
      temp_lead_two = nullptr;
    }
  }

  TrackedObject *lead_one;
  TrackedObject *lead_two;
  TrackedObject *temp_lead_one;
  TrackedObject *temp_lead_two;
};

} // namespace msquare

#endif
