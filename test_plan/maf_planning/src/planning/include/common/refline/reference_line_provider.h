#pragma once
#include "common/math/transform.h"
#include "common/refline.h"
#include "common/refline/qp_spline_reference_line_smoother.h"
#include "common/refline/refline_smoother/spline_2d.h"
#include "common/utils/cartesian_coordinate_system.h"
#include "planner/message_type.h"
#include "planning/common/common.h"
#include "pnc/define/geometry.h"
#include <iostream>

namespace msquare {

class ReferenceLineProvider {
public:
  // ReferenceLineProvider()
  ReferenceLineProvider();
  ~ReferenceLineProvider(){};
  bool Init(const std::vector<ReferenceLinePointDerived> &trajtory);
  // Input: reference now
  bool GetReferenceLine(bool borrow_flag);
  void SetLatBound(double smooth_lat_bound) {
    smooth_lat_bound_ = smooth_lat_bound;
  };
  void SetTrajectoryPoints(std::vector<ReferenceLinePointDerived> trajtory) {
    ref_trajectory = trajtory;
  };
  void SetOrigin(Pose2D navi) {
    navi_x_ = navi.x;
    navi_y_ = navi.y;
    navi_theta_ = navi.theta;
  };

  void SetFrenetCoorSystemPre(
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord_pre_) {
    frenet_coord_pre = frenet_coord_pre_;
  }

  void clear() {
    solve_status = -2;
    smooth_lat_bound_ = 0.1;
    ref_trajectory.clear();
    anchor_points.clear();
    smooth_points_out.clear();
    smooth_points_out_pre.clear();
    smoother_vx.clear();
    smoother_vy.clear();
    smoother_vx_pre.clear();
    smoother_vy_pre.clear();
    navi_x_ = 0.0;
    navi_y_ = 0.0;
    navi_theta_ = 0.0;
  }

  // void SetWorldModel(const std::shared_ptr<WorldModel> &world_model) { //
  // @zyl
  //     world_model_= world_model;
  // }
  void Setcar2enu_ref(const Transform &car2enu_ref,
                      const Transform &enu2car_ref) { // @zyl
    car2enu_ = car2enu_ref;
    enu2car_ = enu2car_ref;
  }

  void Printtraj() {
    for (int i = 0; i < smoother_vx.size(); i++) {
      MSD_LOG(INFO, "%d %lf %lf", i, smoother_vx[i], smoother_vy[i]);
    }
  }

  // void GetSmoothTrajectory(std::vector<RefLinePoint> &ref_trajectory_smooth)
  // {
  //     std::cout << "GetSmoothTrajectory : " << smoother_vx.size() <<
  //     std::endl; std::cout << "ref_trajectory_smooth.size() :" <<
  //     ref_trajectory_smooth.size() << std::endl; std::vector<double>
  //     smoother_vx, smoother_vy
  //     ref_trajectory_smooth.resize(smoother_vx.size());
  //     // // int i = 0;
  //     // for (int i = 0; i < ref_trajectory_smooth.size() ;i++) {
  //     //     (ref_trajectory_smooth[i]).x = smoother_vx[i];
  //     //     (ref_trajectory_smooth[i]).y = smoother_vy[i];

  //     // }
  // }

  void GetSmoothTrajectory(std::vector<double> &smoother_vx_m,
                           std::vector<double> &smoother_vy_m) {
    for (int i = 0; i < smoother_vx.size(); i++) {
      smoother_vx_m.push_back(smoother_vx[i]);
    }
    for (int i = 0; i < smoother_vy.size(); i++) {
      smoother_vy_m.push_back(smoother_vy[i]);
    }
  }
  void GetSmoothTrajectoryMoreInfo(std::vector<double> &smoother_x,
                                   std::vector<double> &smoother_y,
                                   std::vector<double> &smoother_theta,
                                   std::vector<double> &smoother_curv) {
    for (int i = 0; i < smooth_points_out.size(); i++) {
      smoother_x.push_back(smooth_points_out[i].x);
      smoother_y.push_back(smooth_points_out[i].y);
      smoother_theta.push_back(smooth_points_out[i].theta);
      smoother_curv.push_back(smooth_points_out[i].curv);
    }
  }

  void GetSolveStatus(int &solver_status_) { solver_status_ = solve_status; }

  // private:
  void linear_interpolation_interval(
      std::vector<ReferenceLinePointDerived> &trajectory,
      std::vector<AnchorPoint> &anchor_points);
  void interpolation_refpoint(const ReferenceLinePointDerived &p1,
                              const ReferenceLinePointDerived &p2,
                              double s_offset,
                              ReferenceLinePointDerived &ref_point,
                              double heading_p1, double heading_p2,
                              double &heading);

  void get_reference_point_traj(std::vector<double> &ref_s, double s_ref,
                                int &index_ref_s, double &ref_s_offset);
  void set_range_of_Anchor(std::vector<AnchorPoint> &anchor_points);
  // void frame_trans_from_car2enu(std::vector<double> &vx, std::vector<double>
  // &vy); void frame_trans_from_enu2car(std::vector<double> &vx,
  // std::vector<double> &vy); void frame_trans_from_enu2car(Point2D &p_enu,
  // Point2D &p_car); void frame_trans_from_car2enu(Point2D &p_enu, Point2D
  // &p_car);
  void frame_trans_from_car2enu(std::vector<double> &vx,
                                std::vector<double> &vy);              //@zyl
  void frame_trans_from_car2enu(Point2D &p_enu, const Point2D &p_car); // @zyl
  void frame_trans_from_enu2car(std::vector<double> &vx,
                                std::vector<double> &vy);              //@zyl
  void frame_trans_from_enu2car(const Point2D &p_enu, Point2D &p_car); //@zyl

  // for extend
  bool GetStartPoint(AnchorPoint &anchor_start); // start point on last refline
  bool GetAnchorPoints(const AnchorPoint &anchor_start,
                       std::vector<AnchorPoint> &
                           anchor_points); // get anchor points with start point
  bool
  GetSmootherPoint(const std::vector<AnchorPoint> &
                       anchor_points); // set refline smoother to smoother point
  // bool ExtendPoints();        // connect line with old refline
  void set_range_of_smoother(std::vector<double> &vx, std::vector<double> &vy);

  // void Car2Enu();  // zyl
  // void Enu2Car(Point2D &point); // zyl

  double findproject(const Point2D &p1, const Point2D &p2, const Point2D &p3);
  double GetDistance(double x1, double x2, double y1, double y2);
  Point2D find_projection(const Point2D &p1, const Point2D &p2,
                          const Point2D &p, double &t);

  std::vector<ReferenceLinePointDerived> ref_trajectory;
  std::vector<AnchorPoint> anchor_points;

  std::shared_ptr<QpSplineReferenceLineSmoother> smoother_;
  std::vector<double> smoother_vx, smoother_vy;         // global
  std::vector<double> smoother_vx_pre, smoother_vy_pre; // global

  std::vector<SmoothPoint> smooth_points_out, smooth_points_out_pre;
  ReferenceLineSmootherConfig smoother_config;

  double navi_x_, navi_y_, navi_theta_;

  // frenet next
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_pre;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_cur;

  // std::shared_ptr<WorldModel> world_model_;//@zyl
  Transform car2enu_; // @zyl
  Transform enu2car_; // @zyl

  const double LOOKAHEAD_DISTANCE = 25.0;
  const double INTERVAL = 0.3;        // m
  const double DIST_FROM_BEHIND = 30; // Horizon lower bound
  const double DIST_TO_FRONT = 120;   // Horizon upper bound
  const double ANCHOR_INTERVAL = 5.0;

  int solve_status = -2; // solver status
  double smooth_lat_bound_ = 0.1;
};

} // namespace msquare
