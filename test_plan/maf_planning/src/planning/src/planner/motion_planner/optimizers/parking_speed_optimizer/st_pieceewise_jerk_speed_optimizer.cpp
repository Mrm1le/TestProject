

#include "planner/motion_planner/optimizers/parking_speed_optimizer/st_pieceewise_jerk_speed_optimizer.h"
#include "planner/motion_planner/optimizers/parking_speed_optimizer/piecewise_jerk_qp.h"
// #include "common/refline/refline_smoother/spline_2d.h"
// #include "planning/common/common.h"
#include "common/config/vehicle_param.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <array>
#include <vector>

namespace msquare {
namespace parking {

void SpeedPlannerCofig::init(double _dt, bool is_reverse){
  dt = _dt;
  v_lo = 0.0;
  v_up = is_reverse ? msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse : msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward;
  a_up = (msquare::CarParams::GetInstance()->car_config.lon_config.acc + 1.0);
  a_lo = -(msquare::CarParams::GetInstance()->car_config.lon_config.dec);

  s_w = msquare::CarParams::GetInstance()->car_config.lon_config.s_w;
  v_w = msquare::CarParams::GetInstance()->car_config.lon_config.v_w;
  j_lo = msquare::CarParams::GetInstance()->car_config.lon_config.j_lo;
  j_up = msquare::CarParams::GetInstance()->car_config.lon_config.j_up;
  a_w = msquare::CarParams::GetInstance()->car_config.lon_config.a_w;
  j_w = msquare::CarParams::GetInstance()->car_config.lon_config.j_w;
  s_ref_w = msquare::CarParams::GetInstance()->car_config.lon_config.s_ref_w;
  v_ref_w = msquare::CarParams::GetInstance()->car_config.lon_config.v_ref_w;

  // std::cout<<"a_lo:"<<a_lo<<" a_up:"<<a_up<<std::endl;
}


bool PiecewiseJerkSpeedOptimizer::makeOptimize(const double init_speed, const double init_a,
                                        std::vector<std::vector<double>>* path, const VecST& vec_st, 
                                        const msquare::parking::VecST& first_st_obs,
                                        const msquare::parking::VecST& second_st_obs) {

  // std::cout<<"Start Speed Qp Optimize"<<config_.dt<<std::endl;
  
  // std::cout << "Ref planning size is:" << vec_st.size() << std::endl;
  // std::cout << "-------------------> QP init_speed  speed is:" << init_speed << std::endl;
  double qp_init_speed  = init_speed;
  if (vec_st.empty()) {
    return false;
  }
  if (qp_init_speed > vec_st[0].v) {
    qp_init_speed  = vec_st[0].v - 0.01;
    // std::cout << "-------------------> QP init_speed  speed is over than limit:" 
    //           << "set init speed to" << qp_init_speed << std::endl;
  }
  Eigen::Vector3d init_state {vec_st.front().s, qp_init_speed, init_a};
  // std::cout << "init state is v: " << qp_init_speed << "  a: " << init_a << std::endl;
  
  const size_t param_num = vec_st.size();
  
  PieceWiseJerk piecewise_jerk_optimizer(param_num, config_.dt, init_state);
  std::vector<std::pair<double, double>> x_bounds;
  std::vector<std::pair<double, double>> dx_bounds;
  std::vector<std::pair<double, double>> ddx_bounds;
  std::vector<double> ref_s_target;
  std::vector<double> ref_s_weight;
  std::vector<double> ref_v_target;
  std::vector<double> ref_v_weight;

  std::vector<double> t_knots;
  for (int i = 0; i < param_num; i++) {
    t_knots.emplace_back(vec_st[i].t);
    // x_bounds.emplace_back(0, vec_st.back().s);
    dx_bounds.emplace_back(0, vec_st[i].v);
    ddx_bounds.emplace_back(config_.a_lo, config_.a_up);

    double ref_s_w = config_.s_ref_w;
    double ref_v_w = config_.v_ref_w;
    double ref_velocity = vec_st[i].v;
    // std::cout << "before target v is:" << ref_velocity << std::endl;
    // MSD_LOG(ERROR, "before target v is: %f", ref_velocity);
    if (first_st_obs.size() > 0 && first_st_obs.size() - 1 >= i) {
      if (std::fabs(first_st_obs[i].v) < ref_velocity) {
        ref_velocity = 0.3*ref_velocity + 0.7*first_st_obs[i].v;
        ref_v_w = 100 + ref_v_w;
        ref_s_w = 5;
      }
      if (second_st_obs.size() > 0 && second_st_obs.size() - 1 >= i) {
        if (std::fabs(second_st_obs[i].v) < ref_velocity)
        ref_velocity = 0.8*ref_velocity + 0.2*second_st_obs[i].v;
      }
    }

    // std::cout << "after target v is:" << ref_velocity << std::endl;
    // MSD_LOG(ERROR, "after target v is: %f", ref_velocity);
    ref_s_target.emplace_back(vec_st[i].s);
    ref_s_weight.emplace_back(ref_s_w);
    ref_v_target.emplace_back(ref_velocity);
    ref_v_weight.emplace_back(ref_v_w);
  }
  // std::cout << "ref v wight is: " << config_.v_ref_w << std::endl;
  // piecewise_jerk_optimizer.set_x_bounds(x_bounds);
  piecewise_jerk_optimizer.set_dx_bounds(dx_bounds);
  piecewise_jerk_optimizer.set_ddx_bounds(ddx_bounds);
  piecewise_jerk_optimizer.set_dddx_bound(config_.j_lo,config_.j_up);

  piecewise_jerk_optimizer.set_weight_x(config_.s_w);
  piecewise_jerk_optimizer.set_weight_dx(config_.v_w);
  piecewise_jerk_optimizer.set_weight_ddx(config_.a_w);
  piecewise_jerk_optimizer.set_weight_dddx(config_.j_w);
  piecewise_jerk_optimizer.set_x_ref(ref_s_weight, ref_s_target);
  piecewise_jerk_optimizer.set_dx_ref(ref_v_weight, ref_v_target);


  const auto ret = piecewise_jerk_optimizer.Solve();

  const auto& result_x = piecewise_jerk_optimizer.optimal_x();
  const auto& result_dx = piecewise_jerk_optimizer.optimal_dx();
  const auto& result_ddx = piecewise_jerk_optimizer.optimal_ddx();
  if (not ret) {
    // std::cout << "QP Failed" << std::endl;
    return false;
  }
  // std::cout << "Speed QP Success" << std::endl;
  double sum_time = 0;
  path->clear();
  std::vector<std::vector<double>> path1;
  path1.resize(param_num);
  // speed_data->clear();
  // for (size_t i = 0; i < param_num; i++) {
  //   std::cout << "Ref ST: " << vec_st[i].t << " " << vec_st[i].s
  //             << " " << vec_st[i].v << std::endl;
  // }

  // std::cout << "the s_t remain s is:" << vec_st.back().s
  //           << "the QP result s is:" << result_x.back()
  //           << std::endl;

  for (size_t i = 0; i < param_num; i++) {
    path1[i].push_back(result_x[i]);
    path1[i].push_back(result_dx[i]);
    path1[i].push_back(result_ddx[i]);
    // std::cout << "QP result is: "
    //           << " " << vec_st[i].t  << " " << result_x[i]
    //           << " " << result_dx[i] << " " << result_ddx[i]
    //           << std::endl;
    sum_time += config_.dt;
  }
  // uint32_t order = 3;
  // Spline2d spline_2d= Spline2d(t_knots, order);
  // Eigen::VectorXd resolution = piecewise_jerk_optimizer.getSolution();
  // Eigen::MatrixXd solved_params = Eigen::MatrixXd::Zero(resolution.rows(), 1);
  // std::cout << "the resolution size is: " << resolution.rows() << std::endl;
  // for (int i = 0; i < resolution.rows(); ++i) {
  //   solved_params(i, 0) = resolution[i];
  // }
  // spline_2d.set_splines(solved_params, 3);
  *path = path1;
  return true;
}

// void PiecewiseJerkSpeedOptimizer::interpolateSpeedData(SpeedData* speed_data) {
//   SpeedData temp_speed_data = *speed_data;
//   double time = 0;
//   speed_data->clear();
//   while (time <= config_.speed_bounds_decider_config().total_time()) {
//     SpeedPoint speed_point;
//     if (!temp_speed_data.evaluateByTime(time, &speed_point)) {
//       time += config_.dt;
//       continue;
//     }
//     speed_data->emplace_back(speed_point);
//     time += config_.dt;
//   }
// }

} // namespace parking
} // namespace msquare