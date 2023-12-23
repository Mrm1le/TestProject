#include <iostream>
#include <vector>
#include <chrono>
#include <bits/stdc++.h>
#include <algorithm>
#include <iomanip>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/utils/geometry.h"
#include "params.h"

namespace msquare
{

  struct Pose_v{
      Pose2D pose;
      double vel;
      Pose_v(Pose2D p, double v): pose(p), vel(v){}
  };

class DifBetSides{
public:

  DifBetSides(const VehicleParam* vehicle_param, std::vector<Pose_v>& traj, std::vector<planning_math::LineSegment2d>& obstacles, std::vector<planning_math::Vec2d> &uss_points);
  ~DifBetSides(){};


  //obs can be sorted in priority queue
  struct obstacle_priority{
      planning_math::LineSegment2d obstacle;
      double dis;
      obstacle_priority(planning_math::LineSegment2d obs, double d): obstacle(obs), dis(d) {};
  };

  struct cmp{
      bool operator() (obstacle_priority a, obstacle_priority b){
          return a.dis > b.dis;
      }
  };

  struct point_priority{
      planning_math::Vec2d point;
      double dis;
      point_priority(planning_math::Vec2d p, double d): point(p), dis(d) {};
  };

  struct cmp_pt{
      bool operator() (point_priority a, point_priority b){
          return a.dis > b.dis;
      }
  };

  double calculate_diff(const Pose_v& pose_v);

  double calculate_diff();

  Pose2D get_diffest_point(){
      return diffest_point_;
  }

  double get_diffest_value(){
      return diffest_value_;
  }

  double get_bad_rate(){
      return bad_rate_;
  }

private:
    double safe_distance_ = 0.6;
    double kMathEpsilon_ = 1e-10;
    Pose2D diffest_point_;
    double diffest_value_ = 0;
    double bad_rate_ = 0;
    const VehicleParam* vehicle_param_;
    double radius_;
    std::vector<planning_math::LineSegment2d> obstacles_;
    std::vector<planning_math::Vec2d> points_;
    std::vector<Pose_v> traj_;

    planning_math::Vec2d getCenterBox(const Pose2D& current_pose) const{
        double center_to_geometry_center = vehicle_param_->center_to_geometry_center;
        double xx = current_pose.x + center_to_geometry_center * cos(current_pose.theta);
        double yy = current_pose.y + center_to_geometry_center * sin(current_pose.theta);
        return planning_math::Vec2d(xx, yy);
    }

    planning_math::Vec2d getCenterCircle(const Pose2D& current_pose) const{
        double center_to_geometry_center = vehicle_param_->front_edge_to_center - vehicle_param_->front_edge_to_mirror;
        double xx = current_pose.x + center_to_geometry_center * cos(current_pose.theta);
        double yy = current_pose.y + center_to_geometry_center * sin(current_pose.theta);
        return planning_math::Vec2d(xx, yy);
    }
};

}