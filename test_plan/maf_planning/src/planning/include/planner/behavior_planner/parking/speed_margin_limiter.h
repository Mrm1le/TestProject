#ifndef SPEED_MARGIN_LIMITER_H
#define SPEED_MARGIN_LIMITER_H

#include "common/config/vehicle_param.h"
#include "common/math/polygon2d.h"
#include "common/utils/geometry.h"
#include "nlohmann/json.hpp"
#include "planner/behavior_planner/parking/speed_planner_util.h"
namespace msquare {
namespace parking {

using SV_PAIR = std::pair<double, double>;
using VecSV = std::vector<std::pair<double, double>>;

enum SpeedType { ACC = 0, CONST_SPEED, DEC };

struct LastPose {
  bool is_valid = false;
  Pose2D p;
  void updatePose(const Pose2D &p1) {
    p.x = p1.x;
    p.y = p1.y;
    p.theta = p1.theta;
    is_valid = true;
  }
  double distance(const Pose2D &p1) {
    if (!is_valid) {
      return 0.0;
    }
    return std::hypot(p.x - p1.x, p.y - p1.y);
  }
};

struct SpeedRes {
  SpeedType type = SpeedType::CONST_SPEED;
  double travel_s = 0.0;
};
struct SpeedMarginDebug {
  bool is_reverse;
  bool is_last_segment = false;
  bool update_wheel_stop = false;
  bool is_dynamic_planning = false;
  bool is_apa_not_parallel = true;
  unsigned int speed_type;
  double last_v;
  double travel_s;
  double last_delta_s;
  VecSV vec_sv;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedMarginDebug, is_reverse,
                                   is_last_segment, update_wheel_stop,
                                   is_dynamic_planning, is_apa_not_parallel,
                                   speed_type, last_v, travel_s, last_delta_s,
                                   vec_sv)

struct SpeedMarginPara {
  bool is_reverse;
  bool is_apa_not_parallel = true;
  bool is_last_segment = false;
  bool update_wheel_stop = false;
  bool is_dynamic_planning = false;
  double dt = 0.1;
  double last_v = 0.0;
  double last_delta_s = 0;
  double max_dis_for_adjust_target = 1.1;
  double max_v;
  double forbid_acc_delta_v;
  double forbid_acc_max_s = 3.0;

  double obstacle_consider_extend_length;
  double width_wo_rearview_mirror;
  double width;
  double length;
  double side_add;

  double back_p1;
  double back_p2;
  double front_p1;
  double front_p2;
  double mirror_p1;
  double mirror_p2;
  double mirror_extrude_length;

  double obstacle_consider_min_v;
  double obstacle_consider_max_v;
  double obstacle_consider_max;
  double window_s_size;
  double v_slop;

  double acc = 1.0;
  double dec = 0.3;
  double limit_dec = 0.4;
  double min_duration_filter_time = 1.0;

  double curvature_change_threshold = 0.3;
  double curvature_limit_v = 0.3;

  // for remain s
  double traj_end_speed_limit_s;
  double control_take_over_remain_s;
  double control_take_over_speed;
  double control_take_over_acc;
  double remain_s_min_velocity;
  double remain_s_planning_deceleration;
  double const_speed_min_s;
  double lower_const_speed_min_s;

  SpeedRes last_res;

  std::vector<planning_math::Box2d> obs_boxes;
  std::vector<planning_math::Vec2d> obs_pts;
  std::vector<planning_math::LineSegment2d> obs_lines;
  SpeedMarginPara() {}
  void init(bool _is_reverse, bool _is_last_segment,
            const std::vector<planning_math::Box2d> obs_boxes,
            const std::vector<planning_math::Vec2d> obs_pts,
            const std::vector<planning_math::LineSegment2d> obs_lines);
  void UpdateByDebug(const SpeedMarginDebug &para_debug) {
    is_reverse = para_debug.is_reverse;
    is_last_segment = para_debug.is_last_segment;
    update_wheel_stop = para_debug.update_wheel_stop;
    is_dynamic_planning = para_debug.is_dynamic_planning;
    is_apa_not_parallel = para_debug.is_apa_not_parallel;
    last_res.type = msquare::parking::SpeedType(para_debug.speed_type);
    last_res.travel_s = para_debug.travel_s;
    last_v = para_debug.last_v;
    last_delta_s = para_debug.last_delta_s;
    // vec_sv_ = para_debug.vec_sv;
  }
  void setWheelStopper(double _last_v) {
    update_wheel_stop = true;
    last_v = _last_v;
  }

  void setDynamicPlanning(double _last_v, double _last_delta_s,
                          bool _is_dynamic_planning) {
    is_dynamic_planning = _is_dynamic_planning;
    last_delta_s = _last_delta_s;
    last_v = _last_v;
  }
};

struct VelocityStep {
  unsigned int i0;
  unsigned int i1;
  double s0;
  double s1;
  double v;
  double delta_s;
  VelocityStep() {}
  VelocityStep(unsigned int _i0, unsigned int _i1, double _s0, double _s1,
               double _v, double _delta_s)
      : i0(_i0), i1(_i1), s0(_s0), s1(_s1), v(_v), delta_s(_delta_s) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose2D, x, y, theta)
class SpeedMarginLimiter {

public:
  SpeedMarginLimiter() {}
  SpeedMarginLimiter(const SpeedMarginPara &para,
                     const std::vector<Pose2D> &traj,
                     const std::vector<double> &curvatures,
                     bool is_debug = false)
      : para_(para), traj_(traj), curvatures_(curvatures), is_debug_(is_debug) {
  }

  bool filterSV();
  bool limitVByObstacle(VecSV &vec_sv);
  bool limitVByCurvature(VecSV &vec_sv);
  /**
   * @brief find the min v in the following 'window_size' range:
   *        v_s = min([s, s+window_size])
   *
   * @param window_size
   * @param vec_sv
   */
  std::vector<unsigned int> filterByMinV(double window_size, VecSV &vec_sv);
  bool filterByMinS(VecSV &vec_sv,
                    const std::vector<unsigned int> &change_index);
  bool getV(double s, double &v, SpeedRes &res);

  bool getVConsiderLast(double s, double last_v, double &v, SpeedRes &res);
  bool filterByRemainS(VecSV &vec_sv);
  bool limitVAtStart(VecSV &vec_sv);
  double getVByRemains(double s);
  std::vector<VecSV> getVecSV() { return sv_list_debug_array_; }
  bool getSegmentV(std::vector<double> &vs);
  bool getVecSVSegment(VecSV &vec_sv, double s);
  bool getSVByT(VecST &vec_st, double dt);
  bool getSTByS(VecST &vec_st, double dt, double s);

  bool adaptDynamicPlanning(VecSV &vec_sv);

  //  fordebug
  void getSerializeString(std::string &debug_string);
  static SpeedMarginDebug generateFromString(const std::string &info);
  double getTotals();
  std::vector<std::vector<msquare::planning_math::Vec2d>> getTrajDebug() {
    return traj_debug_;
  }

private:
  SpeedMarginPara para_;
  std::vector<Pose2D> traj_;
  std::vector<double> curvatures_;
  bool is_debug_ = false;
  VecSV vec_sv_;
  std::vector<unsigned int> change_index_;
  std::vector<VecSV> sv_list_debug_array_;

  std::vector<std::vector<msquare::planning_math::Vec2d>> traj_debug_;
};

} // namespace parking
} // namespace msquare

#endif