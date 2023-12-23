#pragma once
#include "data_driven_planner/common/basic_types.h"
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace msquare {
namespace ddp {

double interpolate(double x1, double y1, double x2, double y2, double x);
double interpolate(double y1, double y2, double ratio);
double interpolate_angle(double x1, double y1, double x2, double y2, double x);
double interpolate_angle(double y1, double y2, double ratio);

class ReferencePath;
void debug_info(const std::shared_ptr<ReferencePath> reference_path);
void debug_info(const TrajectoryPoints &trajectory_points);
void dump_mdebug_info(const TrajectoryPoints &trajectory_points, double z,
                      int r, int g, int b, double width, const std::string id);
void dump_mdebug_planning_failed_msg(const std::string &planning_failed_msg);
void dump_mdebug_polyline(const TrajectoryPoints &trajectory_points, double z,
                          int r, int g, int b, double width,
                          const std::string &id,
                          const std::vector<std::string> &tags = {});
void dump_mdebug_point(const TrajectoryPoints &trajectory_points, double z,
                       int r, int g, int b, double width,
                       const std::string &id);
void dump_mdebug_circle(const TrajectoryPoints &trajectory_points, double z,
                        int r, int g, int b, double radius, double width,
                        const std::string &id);

// class FrenetObstacle;
// bool compare_obstacle_s_descend(const FrenetObstacle &o1,
//                                 const FrenetObstacle &o2);
// bool compare_obstacle_s_ascend(const FrenetObstacle &o1,
//                                const FrenetObstacle &o2);

std::string expand_environment_variable(const std::string &origin_string);

} // namespace ddp
} // namespace msquare
