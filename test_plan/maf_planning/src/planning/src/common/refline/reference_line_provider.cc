#include "common/refline/reference_line_provider.h"
#include "common/utils/util.h"
#include "planning/common/common.h"

namespace msquare {

ReferenceLineProvider::ReferenceLineProvider() {
  smoother_config.max_constraint_interval_ = 3;
  smoother_config.longitudinal_boundary_bound_ = 1.0;
  smoother_config.max_lateral_boundary_bound_ = 0.5;
  smoother_config.min_lateral_boundary_bound_ = 0.2;
  smoother_config.num_of_total_points_ = 400;
  smoother_config.curb_shift_ = 0.2;
  smoother_config.lateral_buffer_ = 0.2;
  smoother_config.resolution_ = 0.02;
  smoother_config.qp_spline_.spline_order_ = 5;            // [default = 5];
  smoother_config.qp_spline_.max_spline_length_ = 15;      // [default = 25];
  smoother_config.qp_spline_.regularization_weight_ = 0.1; // [default = 0.1];
  smoother_config.qp_spline_.second_derivative_weight_ =
      10.0; // [default = 0.0];
  smoother_config.qp_spline_.third_derivative_weight_ =
      100.0; // [default = 100];
}

bool ReferenceLineProvider::Init(
    const std::vector<ReferenceLinePointDerived> &trajtory) {
  SetTrajectoryPoints(trajtory);
  linear_interpolation_interval(ref_trajectory, anchor_points);
  smoother_ = std::make_shared<QpSplineReferenceLineSmoother>(smoother_config);
  smoother_->SetAnchorPoints(anchor_points);
  smoother_vx.clear();
  smoother_vy.clear();
  if (!smoother_->Smooth()) {
    MSD_LOG(ERROR, "Failed to smooth reference line with anchor points");
  }
  smoother_->GetRefPoint(smoother_vx, smoother_vy, smooth_points_out);
  // frame_trans_from_car2enu_smoother(smoother_vx, smoother_vy);
  // frame_trans_from_car2enu(smoother_vx, smoother_vy);
  smoother_vx_pre = smoother_vx;
  smoother_vy_pre = smoother_vy;

  return true;
}

bool ReferenceLineProvider::GetReferenceLine(bool borrow_flag) {
  if (borrow_flag || smoother_vx_pre.size() == 0) {
    linear_interpolation_interval(ref_trajectory, anchor_points);
    // smoother_ =
    //     std::make_shared<QpSplineReferenceLineSmoother>(smoother_config);
    if (nullptr == smoother_) {
      smoother_ =
          std::make_shared<QpSplineReferenceLineSmoother>(smoother_config);
    } else {
      smoother_->clear();
    }
    smoother_->SetAnchorPoints(anchor_points);
    smoother_vx.clear();
    smoother_vy.clear();
    smooth_points_out.clear();
    if (!smoother_->Smooth()) {
      MSD_LOG(ERROR, "Failed to smooth reference line with anchor points");
    }
    smoother_->GetRefPoint(smoother_vx, smoother_vy, smooth_points_out);
    smoother_->GetSolveStatus(solve_status);
    smoother_vx_pre.clear();
    smoother_vy_pre.clear();
    smooth_points_out_pre.clear();
    smoother_vx_pre = smoother_vx;
    smoother_vy_pre = smoother_vy;
  } else {
    AnchorPoint anchor_start;
    std::vector<AnchorPoint> anchor_points_back;
    (void)GetStartPoint(anchor_start);
    (void)GetAnchorPoints(anchor_start, anchor_points_back);
    (void)GetSmootherPoint(anchor_points_back);
    for (int i = 0; i < smoother_vx.size(); i++) {
      MSD_LOG(INFO, "%lf  %lf", smoother_vx[i], smoother_vy[i]);
    }
    frame_trans_from_car2enu(smoother_vx, smoother_vy);
    // ExtendPoints();
    smoother_vx_pre.clear();
    smoother_vy_pre.clear();
    smoother_vx_pre = smoother_vx;
    smoother_vy_pre = smoother_vy;
  }

  return true;
}

bool ReferenceLineProvider::GetStartPoint(AnchorPoint &anchor_start) {
  // current navi fusion to s
  double navi_x = navi_x_;
  double navi_y = navi_y_;
  Point2D frenet_point;
  Point2D start_point_cart;
  Point2D start_point_local;
  // Point2D ego_cart;

  // ego_pose car2enu
  // ego_cart.x = navi_x_;
  // ego_cart.y = navi_y_;
  std::vector<double> s_list_car;
  s_list_car.push_back(0.0);
  // frame_trans_from_enu2car(ego_cart, ego_local);
  for (int i = 1; i < ref_trajectory.size(); i++) {
    if ((ref_trajectory[i]).enu_point.x < 0.) {
      continue;
    }
    auto x_diff =
        (ref_trajectory[i]).enu_point.x - (ref_trajectory[i - 1]).enu_point.x;
    auto y_diff =
        (ref_trajectory[i]).enu_point.y - (ref_trajectory[i - 1]).enu_point.y;
    double s_diff = sqrt(x_diff * x_diff + y_diff * y_diff);
    s_list_car.push_back(s_list_car.back() + s_diff);
    if (s_list_car.back() > LOOKAHEAD_DISTANCE) {
      start_point_local.x = (ref_trajectory[i]).enu_point.x;
      start_point_local.y = (ref_trajectory[i]).enu_point.y;

      anchor_start.path_point.x = start_point_local.x; //
      anchor_start.path_point.y = start_point_local.y; //
      anchor_start.path_point.s = 0.0;
      anchor_start.path_point.theta = frenet_coord_pre->GetRefCurveHeading(
                                          frenet_point.x + LOOKAHEAD_DISTANCE) -
                                      navi_theta_;
      anchor_start.lateral_bound = 1e-6;
      anchor_start.longitudinal_bound = 1e-6;
      break;
    }
  }

  return true;
}

bool ReferenceLineProvider::GetAnchorPoints(
    const AnchorPoint &anchor_start, std::vector<AnchorPoint> &anchor_points) {
  // get start
  // calculation projection
  int index_pre = 0;
  int index_sucee = 0;
  for (int i = 0; i < (int)ref_trajectory.size() - 1; i++) {
    Point2D p3;
    p3.x = anchor_start.path_point.x;
    p3.y = anchor_start.path_point.y;

    auto t = findproject(
        Point2D{ref_trajectory[i].enu_point.x, ref_trajectory[i].enu_point.y},
        Point2D{ref_trajectory[i + 1].enu_point.x,
                ref_trajectory[i + 1].enu_point.y},
        p3);
    if (t >= 0 && t <= 1) {
      index_pre = i;
      index_sucee = i + 1;
      break;
    }
  }
  // cut ref_trajectory;
  ReferenceLinePointDerived pp_start;
  pp_start.enu_point.x = anchor_start.path_point.x;
  pp_start.enu_point.y = anchor_start.path_point.y;
  // pp_start.z = anchor_start.path_point.z;
  std::vector<ReferenceLinePointDerived> smoother_traj(
      ref_trajectory.begin() + index_sucee,
      ref_trajectory.begin() + ref_trajectory.size());
  smoother_traj.insert(smoother_traj.begin(), 1, pp_start);

  MSD_LOG(INFO, "final_point: %lf  %lf", smoother_traj.back().enu_point.x,
          smoother_traj.back().enu_point.y);
  // archor

  linear_interpolation_interval(smoother_traj, anchor_points);
  // set_range_of_Anchor(anchor_points);

  return true;
}

bool ReferenceLineProvider::GetSmootherPoint(
    const std::vector<AnchorPoint> &anchor_points) {
  smoother_ = std::make_shared<QpSplineReferenceLineSmoother>(smoother_config);
  smoother_->SetAnchorPoints(anchor_points);
  smoother_vx.clear();
  smoother_vy.clear();
  if (!smoother_->Smooth()) {
    MSD_LOG(ERROR, "Failed to smooth reference line with anchor points");
  }
  smoother_->GetRefPoint(smoother_vx, smoother_vy, smooth_points_out);
  // frame_trans_from_car2enu_smoother(smoother_vx, smoother_vy);

  // for(int i = 0; i < smoother_vx.size(); i++){
  //   std::cout << "smoother: " <<  smoother_vx[i] << " " << smoother_vy[i] <<
  //   std::endl;
  // }

  return true;
}

double ReferenceLineProvider::findproject(const Point2D &p1, const Point2D &p2,
                                          const Point2D &p3) {
  return ((p3.x - p1.x) * (p2.x - p1.x) + (p3.y - p1.y) * (p2.y - p1.y)) /
         ((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
  // t = np.clip(t, 0, 1);
  // x_p = x0 + (x1 - x0) * t;
  // y_p = y0 + (y1 - y0) * t;
}

double ReferenceLineProvider::GetDistance(double x1, double x2, double y1,
                                          double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

Point2D ReferenceLineProvider::find_projection(const Point2D &p1,
                                               const Point2D &p2,
                                               const Point2D &p, double &t) {
  Point2D p_projection;
  t = ((p.x - p1.x) * (p2.x - p1.x) + (p.y - p1.y) * (p2.y - p.y)) /
      ((p2.y - p1.y) * (p2.y - p1.y) + (p2.y - p1.y) * (p2.y - p1.y));
  // t = np.clip(t, 0, 1)
  p_projection.x = p1.x + (p2.x - p1.x) * t;
  p_projection.y = p1.y + (p2.y - p1.y) * t;
  return p_projection;
}

void ReferenceLineProvider::linear_interpolation_interval(
    std::vector<ReferenceLinePointDerived> &trajectory,
    std::vector<AnchorPoint> &anchor_points) {
  anchor_points.clear();
  std::vector<double> reference_s;

  double interval = 3.0;
  reference_s.push_back(0.0);
  for (size_t i = 1; i < trajectory.size(); i++) {
    auto cur_point = trajectory[i];
    auto last_point = trajectory[i - 1];
    auto x_diff = cur_point.enu_point.x - last_point.enu_point.x;
    auto y_diff = cur_point.enu_point.y - last_point.enu_point.y;
    double s_diff = sqrt(x_diff * x_diff + y_diff * y_diff);
    reference_s.push_back(reference_s.back() + s_diff);
  }

  int num_of_anchors =
      std::max(2, static_cast<int>(reference_s.back() / interval + 0.5));
  std::vector<double> anchor_s;
  uniform_slice(0.0, reference_s.back(), num_of_anchors - 1, &anchor_s);

  // get heading
  std::vector<double> ref_headings;
  for (int i = 0; i < (int)trajectory.size() - 1; i++) {
    double theta =
        std::atan2(trajectory[i + 1].enu_point.y - trajectory[i].enu_point.y,
                   trajectory[i + 1].enu_point.x - trajectory[i].enu_point.x);
    ref_headings.push_back(theta);
  }
  ref_headings.push_back(ref_headings.back());
  // std::cout << " anchor_s.size() : " << anchor_s.size() << std::endl;
  for (const double s : anchor_s) {
    int index_ref_s;
    double index_ref_s_offset;
    double theta_ref;
    ReferenceLinePointDerived p_cur;
    get_reference_point_traj(reference_s, s, index_ref_s, index_ref_s_offset);

    interpolation_refpoint(
        trajectory[index_ref_s],
        trajectory[std::min((unsigned long)(index_ref_s + 1),
                            trajectory.size() - 1)],
        index_ref_s_offset, p_cur, ref_headings[index_ref_s],
        ref_headings[std::min((unsigned long)(index_ref_s + 1),
                              trajectory.size() - 1)],
        theta_ref);
    AnchorPoint anchor;
    anchor.path_point.x = p_cur.enu_point.x;
    anchor.path_point.y = p_cur.enu_point.y;
    anchor.path_point.z = 0.0; // p_cur.z;
    anchor.path_point.theta = theta_ref;
    anchor.path_point.s = s;
    anchor.lateral_bound = smooth_lat_bound_; // default : 0.4
    anchor.longitudinal_bound = 0.1;          // default : 1.0
    anchor_points.push_back(anchor);
  }

  // set_range_of_trajectory_smoother();
  // set_range_of_Anchor(anchor_points);
  anchor_points.front().longitudinal_bound = 1e-6;
  anchor_points.front().lateral_bound = 1e-6;
  anchor_points.front().enforced = true;
  // anchor_points.at(1).longitudinal_bound = 1e-6;
  // anchor_points.at(1).lateral_bound = 1e-6;
  // anchor_points.at(1).enforced = true;
  // anchor_points.at(2).longitudinal_bound = 1e-6;
  // anchor_points.at(2).lateral_bound = 1e-6;
  // anchor_points.at(2).enforced = true;
  // anchor_points.at(3).longitudinal_bound = 1e-6;
  // anchor_points.at(3).lateral_bound = 1e-6;
  // anchor_points.at(3).enforced = true;
  if (anchor_points.size() > 5) {
    anchor_points.at(anchor_points.size() - 4).longitudinal_bound = 1e-6;
    anchor_points.at(anchor_points.size() - 4).lateral_bound = 1e-6;
    // anchor_points.at(anchor_points.size() - 4).enforced = true;
    anchor_points.at(anchor_points.size() - 3).longitudinal_bound = 1e-6;
    anchor_points.at(anchor_points.size() - 3).lateral_bound = 1e-6;
    // anchor_points.at(anchor_points.size() - 3).enforced = true;
    anchor_points.at(anchor_points.size() - 2).longitudinal_bound = 1e-6;
    anchor_points.at(anchor_points.size() - 2).lateral_bound = 1e-6;
    // anchor_points.at(anchor_points.size() - 2).enforced = true;
  }
  anchor_points.back().longitudinal_bound = 1e-6;
  anchor_points.back().lateral_bound = 1e-6;
  anchor_points.back().enforced = true;

  double s_offset = anchor_points[0].path_point.s;
  for (int i = 0; i < anchor_points.size(); i++) {
    anchor_points[i].path_point.s = anchor_points[i].path_point.s - s_offset;
  }
}

void ReferenceLineProvider::interpolation_refpoint(
    const ReferenceLinePointDerived &p1, const ReferenceLinePointDerived &p2,
    double s_offset, ReferenceLinePointDerived &ref_point, double heading_p1,
    double heading_p2, double &heading) {
  double distance = sqrt(
      (p1.enu_point.x - p2.enu_point.x) * (p1.enu_point.x - p2.enu_point.x) +
      (p1.enu_point.y - p2.enu_point.y) * (p1.enu_point.y - p2.enu_point.y));
  if (std::fabs(distance) > 1e-8) {
    double t = s_offset / distance;
    ref_point.enu_point.x =
        p1.enu_point.x + t * (p2.enu_point.x - p1.enu_point.x);
    ref_point.enu_point.y =
        p1.enu_point.y + t * (p2.enu_point.y - p1.enu_point.y);
    // ref_point.z =  p1.z + t * (p2.z - p1.z);
    heading = heading_p1 + t * (heading_p2 - heading_p1);
  } else {
    ref_point.enu_point.x = p1.enu_point.x;
    ref_point.enu_point.y = p1.enu_point.y;
    // ref_point.z =  p1.z + t * (p2.z - p1.z);
    heading = heading_p1;
  }
}

void ReferenceLineProvider::get_reference_point_traj(std::vector<double> &ref_s,
                                                     double s_ref,
                                                     int &index_ref_s,
                                                     double &ref_s_offset) {
  if (s_ref <= 0.0) {
    index_ref_s = 0;
    ref_s_offset = 0;
    // return {0, 0.0};
  }
  if (s_ref >= ref_s.back()) {
    index_ref_s = ref_s.size() - 1;
    ref_s_offset = 0.0;
    // return {num_points_ - 1, 0.0};
  }
  for (int i = 0; i < ref_s.size(); i++) {
    if (s_ref < ref_s[i]) {
      index_ref_s = i - 1;
      break;
    }
  }
  ref_s_offset = s_ref - ref_s[index_ref_s];
}

void ReferenceLineProvider::set_range_of_Anchor(
    std::vector<AnchorPoint> &anchor_points) {
  if (anchor_points.empty()) {
    MSD_LOG(WARN,
            "Set Range of the trajectory: Incoming trajectory is empty!!!");
    return;
  }

  while (true) {
    double x = anchor_points.front().path_point.x;
    double y = anchor_points.front().path_point.y;
    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));
    if (dist_to_origin <= DIST_FROM_BEHIND) {
      break;
    }
    anchor_points.erase(anchor_points.begin());
  }

  while (true) {
    double x = anchor_points.back().path_point.x;
    double y = anchor_points.back().path_point.y;
    double dist_to_origin = std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0));
    if (dist_to_origin <= DIST_TO_FRONT) {
      break;
    }
    anchor_points.pop_back();
  }
}

void ReferenceLineProvider::set_range_of_smoother(std::vector<double> &vx,
                                                  std::vector<double> &vy) {
  if (vx.empty()) {
    MSD_LOG(WARN,
            "Set Range of the trajectory: Incoming trajectory is empty!!!");
    return;
  }

  while (true) {
    double x = vx.front();
    double y = vy.front();
    double dist_to_origin =
        std::sqrt(std::pow(x - navi_x_, 2.0) + std::pow(y - navi_y_, 2.0));
    if (dist_to_origin <= DIST_FROM_BEHIND) {
      break;
    }
    vx.erase(vx.begin());
    vy.erase(vy.begin());
  }

  while (true) {
    double x = vx.back();
    double y = vy.back();
    double dist_to_origin =
        std::sqrt(std::pow(x - navi_x_, 2.0) + std::pow(y - navi_y_, 2.0));
    if (dist_to_origin <= DIST_TO_FRONT) {
      break;
    }
    vx.pop_back();
    vy.pop_back();
  }
}

// @zyl
void ReferenceLineProvider::frame_trans_from_car2enu(std::vector<double> &vx,
                                                     std::vector<double> &vy) {

  if (vx.size() > 0 && (vx.size() != vy.size())) {
    return;
  }

  Eigen::Vector3d car_point, enu_point;
  for (int i = 0; i < vx.size(); i++) {
    car_point.x() = vx[i];
    car_point.y() = vy[i];
    car_point.z() = 0.0;

    enu_point = car2enu_ * car_point;
    vx[i] = enu_point.x();
    vy[i] = enu_point.y();
  }
}

void ReferenceLineProvider::frame_trans_from_car2enu(Point2D &p_enu,
                                                     const Point2D &p_car) {

  Eigen::Vector3d car_point, enu_point;
  car_point.x() = p_car.x;
  car_point.y() = p_car.y;
  car_point.z() = 0.0;

  enu_point = car2enu_ * car_point;
  p_enu.x = enu_point.x();
  p_enu.y = enu_point.y();
}

void ReferenceLineProvider::frame_trans_from_enu2car(std::vector<double> &vx,
                                                     std::vector<double> &vy) {
  if (vx.size() > 0 && (vx.size() != vy.size())) {
    return;
  }

  Eigen::Vector3d car_point, enu_point;
  for (int i = 0; i < vx.size(); i++) {
    car_point.x() = vx[i];
    car_point.y() = vy[i];
    car_point.z() = 0.0;

    enu_point = enu2car_ * car_point;
    vx[i] = enu_point.x();
    vy[i] = enu_point.y();
  }
}

void ReferenceLineProvider::frame_trans_from_enu2car(const Point2D &p_enu,
                                                     Point2D &p_car) {
  Eigen::Vector3d car_point, enu_point;
  car_point.x() = p_enu.x;
  car_point.y() = p_enu.y;
  car_point.z() = 0.0;

  enu_point = enu2car_ * car_point;
  p_car.x = enu_point.x();
  p_car.y = enu_point.y();
}

// zyl
#if 0
void ReferenceLineProvider::frame_trans_from_car2enu(std::vector<double> &vx, std::vector<double> &vy) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform("/car", "/enu", ros::Time(0), transform);
  } catch (tf2::TransformException &ex) {
    MSD_LOG(WARN, "%s", ex.what());
    return;
  }
  for (int i = 0; i < vx.size(); i++) {
    geometry_msgs::PointStamped car_point;
    geometry_msgs::PointStamped enu_point;
    car_point.header.frame_id = "car";
    car_point.header.stamp = ros::Time(0);
    car_point.point.x = vx[i];
    car_point.point.y = vy[i];
    car_point.point.z = 0.0;

    listener_.transformPoint("enu", car_point, enu_point);
    vx[i] = enu_point.point.x;
    vy[i] = enu_point.point.y;

  }
}

void ReferenceLineProvider::frame_trans_from_enu2car(std::vector<double> &vx, std::vector<double> &vy) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform("/enu", "/car", ros::Time(0), transform);
  } catch (tf2::TransformException &ex) {
    MSD_LOG(WARN, "%s", ex.what());
    return;
  }
  for (int i = 0; i < vx.size(); i++) {
    geometry_msgs::PointStamped car_point;
    geometry_msgs::PointStamped enu_point;
    enu_point.header.frame_id = "enu";
    enu_point.header.stamp = ros::Time(0);
    enu_point.point.x = vx[i];
    enu_point.point.y = vy[i];
    enu_point.point.z = 0.0;

    listener_.transformPoint("car", enu_point, car_point);
    vx[i] = car_point.point.x;
    vy[i] = car_point.point.y;

  }
}

void ReferenceLineProvider::frame_trans_from_enu2car(Point2D &p_enu, Point2D &p_car) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform("/enu", "/car", ros::Time(0), transform);
  } catch (tf2::TransformException &ex) {
    MSD_LOG(WARN, "%s", ex.what());
    return;
  }
  geometry_msgs::PointStamped car_point;
  geometry_msgs::PointStamped enu_point;
  enu_point.header.frame_id = "enu";
  enu_point.header.stamp = ros::Time(0);
  enu_point.point.x = p_enu.x;
  enu_point.point.y = p_enu.y;
  enu_point.point.z = 0.0;

  listener_.transformPoint("car", enu_point, car_point);
  p_car.x = car_point.point.x;
  p_car.y = car_point.point.y;
}

void ReferenceLineProvider::frame_trans_from_car2enu(Point2D &p_enu, Point2D &p_car) {
  try {
    tf::StampedTransform transform;
    listener_.lookupTransform("/car", "/enu", ros::Time(0), transform);
  } catch (tf2::TransformException &ex) {
    MSD_LOG(WARN, "%s", ex.what());
    return;
  }
  geometry_msgs::PointStamped car_point;
  geometry_msgs::PointStamped enu_point;
  car_point.header.frame_id = "car";
  car_point.header.stamp = ros::Time(0);
  car_point.point.x = p_enu.x;
  car_point.point.y = p_enu.y;
  car_point.point.z = 0.0;

  listener_.transformPoint("enu", car_point, enu_point);
  p_car.x = enu_point.point.x;
  p_car.y = enu_point.point.y;
}
#endif
} // namespace msquare
