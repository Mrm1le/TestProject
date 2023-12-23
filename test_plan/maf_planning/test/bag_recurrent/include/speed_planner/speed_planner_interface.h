#ifndef SPEED_PLANNER_INTERFACE_H
#define SPEED_PLANNER_INTERFACE_H

#include "planner/behavior_planner/parking/speed_margin_limiter.h"

namespace speed_planner_interface{

std::vector<msquare::parking::VecSV> marginVelocityLimit(std::string ods_str, std::vector<double> curvatures);

// std::vector<std::vector<double> > getSVByT(std::string ods_str, std::vector<double> curvatures, double dt);

// std::vector<std::vector<double>> solveSpeed(const std::vector<std::vector<double> > &vec_st, double dt, 
//                                             double ego_speed, const double init_a);

// std::vector<std::vector<double>> solveSpeedByConfig(const std::vector<std::vector<double> > &vec_st,
//             double dt, double ego_speed, const double init_a, std::vector<double> config);
// std::vector<msquare::parking::VecSV> marginVelocityLimitDynamic(std::string ods_str, std::vector<double> curvatures, std::vector<double> last_res);
struct MarginDebugWithDebugPara{
    msquare::parking::VecSV debug_vec_sv;
    std::vector<msquare::parking::VecSV> vec_sv;
    std::vector<std::vector<parking_scenario::Point2d>> polygons;
};

MarginDebugWithDebugPara getSpeedMarginByParas(std::vector<double>& xs, 
                                const std::vector<double > ys ,std::vector<double>& thetas,
                                std::vector<parking_scenario::Point2d> obs_pts,
                                std::vector<double> curvatures,double dt,const std::string& para_str);
std::vector<std::vector<double> > getSVByT(std::string ods_str, std::vector<double> curvatures, double dt);

std::vector<std::vector<double>> solveSpeed(const std::vector<std::vector<double> > &vec_st, double dt, 
                                            double ego_speed, const double init_a);

std::vector<std::vector<double>> solveSpeedByConfig(const std::vector<std::vector<double> > &vec_st,
                        const std::vector<std::vector<double> > &first_obs_vec_st,
            const std::vector<std::vector<double> > &second_obs_vec_st,
            double dt, double ego_speed, const double init_a, std::vector<double> config_vector);

}

#endif