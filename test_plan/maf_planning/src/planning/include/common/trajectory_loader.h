#ifndef MSQUARE_DECISION_PLANNING_COMMON_TRAJECTORY_LOADER_H_
#define MSQUARE_DECISION_PLANNING_COMMON_TRAJECTORY_LOADER_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace msquare {

struct RefTrajectoryPoint {
  struct Quaternion {
    double x;
    double y;
    double z;
    double w;
  };
  long long int timestamp;
  Quaternion quaternion;
  double x;
  double y;
  double z;
  double yaw;
};

class TrajectoryLoader {
public:
  TrajectoryLoader();
  ~TrajectoryLoader() = default;

  bool LoadTrajectory();

private:
  std::vector<RefTrajectoryPoint> trajectory;

public:
  const std::vector<RefTrajectoryPoint> get_trajectory() { return trajectory; };
};

} // namespace msquare

#endif
